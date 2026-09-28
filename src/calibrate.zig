//! The pace settings that would have predicted the race that was run.
//!
//! gpxz's model slows a runner by exp(k · effort_km): fatigue. If every section's actual
//! time is its planned moving time times a · exp(b · effort_km), then a base pace of a × the
//! planned one and a fatigue coefficient of k + b predict the actual race. So a weighted least
//! squares fit of ln(actual / planned) against each section's effort-weighted midpoint gives
//! a and b directly.
//!
//! "Actual" here is the time between two checkpoints less the stop at the first one: breaks
//! on the trail stay in, since gpxz has nowhere else to put them, while checkpoint stops are
//! the waypoints' own business (`<stopDuration>`, the LifeBase default). The fit is checked
//! end to end by rerunning gpxz's plan with the fitted pace and the actual stops.

const std = @import("std");
const assert = std.debug.assert;
const testing = std.testing;
const gpxz = @import("gpxz");
const Timeline = @import("timeline.zig").Timeline;
const compare = @import("compare.zig");

/// Shorter sections carry more noise (an aid station's bustle, a GPS glitch) than signal.
const section_moving_s_min = 600.0;
/// A section run this much off the planned trace wasn't the section planned: its time says
/// nothing about the pace on it.
const section_off_route_ratio_max = 0.05;
/// A line through fewer points says nothing about its slope.
const sections_used_min = 3;
/// Far more sections than any race has; the fit's points live on the stack.
const sections_max = 256;

pub const Calibration = struct {
    sections_used: u32,
    /// Sections left out for running off the planned trace.
    sections_off_route: u32,
    pace_base_s_per_km: f64,
    fatigue_coefficient: f64,
    /// The average actual stop at the LifeBases, and at the other checkpoints (the start and
    /// finish aside). Null without one.
    life_base_stop_s: ?u32,
    checkpoint_stop_s: ?u32,
    /// The plan rerun with the fitted settings and each checkpoint's actual stop: race time
    /// at each checkpoint.
    duration_s_replanned: []f64,
    /// Over the checkpoints reached after the start: how far each plan was from the race.
    error_s_rms_planned: f64,
    error_s_rms_replanned: f64,
    error_s_max_planned: f64,
    error_s_max_replanned: f64,

    pub fn deinit(self: *Calibration, allocator: std.mem.Allocator) void {
        allocator.free(self.duration_s_replanned);
        self.* = undefined;
    }
};

/// Null when fewer than `sections_used_min` sections are usable, more than `sections_max`
/// exist, or the rerun plan doesn't line up with the original checkpoints.
pub fn compute(
    allocator: std.mem.Allocator,
    data: *const gpxz.GPXData,
    settings: *const gpxz.Settings,
    timeline: *const Timeline,
    checkpoints: []const compare.Checkpoint,
    sections: []const compare.Section,
) !?Calibration {
    const plan = data.plan.?;
    assert(checkpoints.len == plan.len and sections.len + 1 == plan.len);
    const fatigue = settings.fatigue_coefficient;
    const fit = fit_sections(plan, timeline, checkpoints, sections, fatigue) orelse return null;
    var fitted = settings.*;
    fitted.pace_base_s_per_km = settings.pace_base_s_per_km * @exp(fit.line.intercept);
    fitted.fatigue_coefficient = @max(settings.fatigue_coefficient + fit.line.slope, 0);

    assert(fitted.pace_base_s_per_km > 0 and std.math.isFinite(fitted.pace_base_s_per_km));
    assert(fitted.fatigue_coefficient >= 0);

    const replanned = try replan(allocator, data, &fitted, checkpoints) orelse return null;
    assert(replanned.len == checkpoints.len);
    const planned = errors_compute(checkpoints, null);
    const rerun = errors_compute(checkpoints, replanned);
    const stops = stops_average(checkpoints);
    return .{
        .sections_used = fit.used,
        .sections_off_route = fit.off_route,
        .pace_base_s_per_km = fitted.pace_base_s_per_km,
        .fatigue_coefficient = fitted.fatigue_coefficient,
        .life_base_stop_s = stops.life_base_s,
        .checkpoint_stop_s = stops.other_s,
        .duration_s_replanned = replanned,
        .error_s_rms_planned = planned.rms,
        .error_s_rms_replanned = rerun.rms,
        .error_s_max_planned = planned.max,
        .error_s_max_replanned = rerun.max,
    };
}

const WeightedPoint = struct { x: f64, y: f64, weight: f64 };
const Line = struct { intercept: f64, slope: f64 };
const Fit = struct { line: Line, used: u32, off_route: u32 };

fn fit_sections(
    plan: []const gpxz.PlanEntry,
    timeline: *const Timeline,
    checkpoints: []const compare.Checkpoint,
    sections: []const compare.Section,
    fatigue_coefficient: f64,
) ?Fit {
    assert(fatigue_coefficient >= 0);
    assert(sections.len + 1 == checkpoints.len);
    // The GPX decides how many sections there are: more than fit is no calibration, not a
    // fit on the first few.
    if (sections.len > sections_max) return null;
    var points: [sections_max]WeightedPoint = undefined;
    var used: u32 = 0;
    var off_route: u32 = 0;
    for (sections, 0..) |*section, index| {
        assert(used < sections_max);
        if (section.moving_s_planned < section_moving_s_min) continue;
        const start = checkpoints[index].duration_s_actual orelse continue;
        const end = checkpoints[index + 1].duration_s_actual orelse continue;
        if (section.distance_m_off_route > section.distance_m * section_off_route_ratio_max) {
            off_route += 1;
            continue;
        }
        const duration_s = (end - start) - (checkpoints[index].stop_s_actual orelse 0);
        if (!(duration_s > 0)) continue;
        const efforts = timeline.distance_km_effort;
        points[used] = .{
            .x = (efforts[plan[index].index] + efforts[plan[index + 1].index]) / 2.0,
            .y = @log(duration_s / section.moving_s_planned),
            .weight = section.moving_s_planned,
        };
        used += 1;
    }
    assert(used + off_route <= sections.len);
    if (used < sections_used_min) return null;
    var line = weighted_least_squares(points[0..used], null);
    // Negative fatigue has no meaning in the model: pin the coefficient at 0 and refit the
    // pace alone.
    if (fatigue_coefficient + line.slope < 0) {
        line = weighted_least_squares(points[0..used], -fatigue_coefficient);
    }
    assert(fatigue_coefficient + line.slope >= -1e-12);
    return .{ .line = line, .used = used, .off_route = off_route };
}

/// Fits y = intercept + slope · x. With `slope_fixed`, fits the intercept alone.
fn weighted_least_squares(points: []const WeightedPoint, slope_fixed: ?f64) Line {
    assert(points.len > 0);
    var weight_sum: f64 = 0;
    var x_mean: f64 = 0;
    var y_mean: f64 = 0;
    for (points) |point| {
        assert(point.weight > 0);
        weight_sum += point.weight;
        x_mean += point.weight * point.x;
        y_mean += point.weight * point.y;
    }
    x_mean /= weight_sum;
    y_mean /= weight_sum;
    var covariance: f64 = 0;
    var variance: f64 = 0;
    for (points) |point| {
        covariance += point.weight * (point.x - x_mean) * (point.y - y_mean);
        variance += point.weight * (point.x - x_mean) * (point.x - x_mean);
    }
    const slope = slope_fixed orelse if (variance > 0) covariance / variance else 0.0;
    const line: Line = .{ .intercept = y_mean - slope * x_mean, .slope = slope };
    assert(std.math.isFinite(line.intercept) and std.math.isFinite(line.slope));
    return line;
}

/// Reruns gpxz's plan with `fitted` and, at every checkpoint reached, the stop actually
/// made there. Returns each checkpoint's race time, or null when the rerun doesn't resolve
/// the same checkpoints. The caller owns the result.
fn replan(
    allocator: std.mem.Allocator,
    data: *const gpxz.GPXData,
    fitted: *const gpxz.Settings,
    checkpoints: []const compare.Checkpoint,
) !?[]f64 {
    const plan = data.plan.?;
    assert(plan.len == checkpoints.len);
    const waypoints = try allocator.dupe(gpxz.Waypoint, data.waypoints);
    defer allocator.free(waypoints);
    for (plan, checkpoints) |*entry, *checkpoint| {
        const stop_s = checkpoint.stop_s_actual orelse continue;
        // gpxz's plan entries borrow their waypoint's name: the same slice is the same
        // waypoint, even where two share a name (a loop's start and finish).
        for (waypoints) |*waypoint| {
            if (waypoint.name.ptr != entry.name.ptr) continue;
            waypoint.stop_s = @intFromFloat(@round(stop_s));
        }
    }
    const replanned = try gpxz.calibration.plan_compute(
        allocator,
        &data.trace,
        waypoints,
        fitted,
    ) orelse return null;
    defer allocator.free(replanned);
    if (replanned.len != plan.len) return null;

    const durations = try allocator.alloc(f64, replanned.len);
    for (replanned, plan, durations) |*entry, *original, *duration| {
        assert(entry.index == original.index);
        duration.* = entry.duration_s_arrival;
    }
    assert(durations[0] == 0);
    return durations;
}

const Stops = struct { life_base_s: ?u32, other_s: ?u32 };

/// The average actual stop at LifeBases and at the other checkpoints between start and finish.
fn stops_average(checkpoints: []const compare.Checkpoint) Stops {
    if (checkpoints.len < 3) return .{ .life_base_s = null, .other_s = null };
    var life_base: Average = .{};
    var other: Average = .{};
    for (checkpoints[1 .. checkpoints.len - 1]) |*checkpoint| {
        const stop_s = checkpoint.stop_s_actual orelse continue;
        const type_name = checkpoint.type_name orelse "";
        if (std.mem.eql(u8, type_name, gpxz.gpx_data.type_life_base)) {
            life_base.add(stop_s);
        } else {
            other.add(stop_s);
        }
    }
    assert(life_base.count + other.count <= checkpoints.len - 2);
    return .{ .life_base_s = life_base.seconds(), .other_s = other.seconds() };
}

const Average = struct {
    sum: f64 = 0,
    count: u32 = 0,

    fn add(self: *Average, value: f64) void {
        assert(value >= 0);
        self.sum += value;
        self.count += 1;
    }

    fn seconds(self: *const Average) ?u32 {
        assert(self.sum >= 0);
        if (self.count == 0) return null;
        return @intFromFloat(@round(self.sum / @as(f64, @floatFromInt(self.count))));
    }
};

const Errors = struct { rms: f64, max: f64 };

/// Arrival errors against the actual race, for the original plan (`replanned` null) or a
/// rerun one. The start is skipped: both clocks are 0 there by construction.
fn errors_compute(checkpoints: []const compare.Checkpoint, replanned: ?[]const f64) Errors {
    assert(checkpoints.len > 0);
    if (replanned) |durations| assert(durations.len == checkpoints.len);
    var sum_squared: f64 = 0;
    var max: f64 = 0;
    var count: u32 = 0;
    for (checkpoints[1..], 1..) |*checkpoint, index| {
        const actual = checkpoint.duration_s_actual orelse continue;
        const predicted = if (replanned) |durations|
            durations[index]
        else
            checkpoint.duration_s_planned;
        const error_s = @abs(actual - predicted);
        sum_squared += error_s * error_s;
        max = @max(max, error_s);
        count += 1;
    }
    if (count == 0) return .{ .rms = 0, .max = 0 };
    const count_float: f64 = @floatFromInt(count);
    const errors: Errors = .{ .rms = @sqrt(sum_squared / count_float), .max = max };
    assert(errors.rms <= errors.max + 1e-9);
    return errors;
}

test "weighted_least_squares: recovers a line, and a fixed slope" {
    const points = [_]WeightedPoint{
        .{ .x = 0, .y = 1, .weight = 1 },
        .{ .x = 1, .y = 3, .weight = 2 },
        .{ .x = 2, .y = 5, .weight = 1 },
    };
    const line = weighted_least_squares(&points, null);
    try testing.expectApproxEqAbs(@as(f64, 1), line.intercept, 1e-12);
    try testing.expectApproxEqAbs(@as(f64, 2), line.slope, 1e-12);
    const pinned = weighted_least_squares(&points, 0);
    try testing.expectApproxEqAbs(@as(f64, 3), pinned.intercept, 1e-12);
    try testing.expectEqual(@as(f64, 0), pinned.slope);

    // All at one x: no slope to find.
    const vertical = [_]WeightedPoint{
        .{ .x = 4, .y = 1, .weight = 1 },
        .{ .x = 4, .y = 3, .weight = 1 },
    };
    try testing.expectEqual(@as(f64, 0), weighted_least_squares(&vertical, null).slope);
}

test "Average: none, one, several" {
    var average: Average = .{};
    try testing.expectEqual(@as(?u32, null), average.seconds());
    average.add(60);
    try testing.expectEqual(@as(?u32, 60), average.seconds());
    average.add(121);
    try testing.expectEqual(@as(?u32, 91), average.seconds());
}

fn checkpoint_test(
    type_name: []const u8,
    duration_s_actual: ?f64,
    stop_s_actual: ?f64,
) compare.Checkpoint {
    return .{
        .name = "",
        .type_name = type_name,
        .distance_m = 0,
        .elevation_gain_m = 0,
        .duration_s_planned = 0,
        .duration_s_actual = duration_s_actual,
        .delta_s = null,
        .stop_s_planned = 0,
        .stop_s_actual = stop_s_actual,
        .epoch_s_arrival_actual = null,
        .epoch_s_cutoff = null,
        .margin_s_planned = null,
        .margin_s_actual = null,
    };
}

test "stops_average: LifeBases apart, the start and finish left out" {
    const life_base = gpxz.gpx_data.type_life_base;
    var checkpoints = [_]compare.Checkpoint{
        checkpoint_test("Start", 0, 600),
        checkpoint_test(life_base, 3600, 1800),
        checkpoint_test("TimeBarrier", 7200, 120),
        checkpoint_test(life_base, 10800, 2400),
        checkpoint_test("TimeBarrier", null, null),
        checkpoint_test("Arrival", null, null),
    };
    const stops = stops_average(&checkpoints);
    try testing.expectEqual(@as(?u32, 2100), stops.life_base_s);
    try testing.expectEqual(@as(?u32, 120), stops.other_s);

    // Start and finish only: nothing between them to average.
    const ends = stops_average(checkpoints[0..2]);
    try testing.expectEqual(@as(?u32, null), ends.life_base_s);
    try testing.expectEqual(@as(?u32, null), ends.other_s);
}

test "errors_compute: the start skipped, checkpoints never reached skipped" {
    var checkpoints = [_]compare.Checkpoint{
        checkpoint_test("Start", 0, null),
        checkpoint_test("TimeBarrier", 1300, null),
        checkpoint_test("TimeBarrier", 2000, null),
        checkpoint_test("Arrival", null, null),
    };
    checkpoints[1].duration_s_planned = 1000;
    checkpoints[2].duration_s_planned = 2400;
    checkpoints[3].duration_s_planned = 3600;
    // Errors of 300 and 400 s: rms 353.55, max 400.
    const planned = errors_compute(&checkpoints, null);
    try testing.expectApproxEqAbs(@as(f64, 353.553), planned.rms, 1e-3);
    try testing.expectEqual(@as(f64, 400), planned.max);
    const exact = errors_compute(&checkpoints, &.{ 0, 1300, 2000, 9999 });
    try testing.expectEqual(@as(f64, 0), exact.rms);
    try testing.expectEqual(@as(f64, 0), exact.max);
    // Only the start reached: no error to measure.
    const none = errors_compute(checkpoints[0..1], null);
    try testing.expectEqual(@as(f64, 0), none.rms);
}
