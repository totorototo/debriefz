//! The race as series, for drawing it: the plan's profile with both clocks at fixed steps
//! along it, and the runner's track thinned out for a map. The report's other parts are
//! summaries; these are what a chart or a map needs between them.
//!
//! Both are downsampled here rather than by the reader: a day-long activity at 1 Hz is a
//! hundred thousand samples, and a GPX trace tens of thousands of points.

const std = @import("std");
const assert = std.debug.assert;
const testing = std.testing;
const gpxz = @import("gpxz");
const Sample = @import("activity.zig").Sample;
const Match = @import("match.zig").Match;
const Actual = @import("actual.zig").Actual;
const Timeline = @import("timeline.zig").Timeline;

/// A point along the plan, every `profile_m` from the first checkpoint and at the last.
pub const ProfilePoint = struct {
    /// Along the plan, from the first checkpoint.
    distance_m: f64,
    elevation_m: f64,
    latitude: f64,
    longitude: f64,
    duration_s_planned: f64,
    /// Race time on first reaching this point; null if the runner never did.
    duration_s_actual: ?f64,
    /// Average over the stretch from the point before; null at the first point, where
    /// either end wasn't reached, or without a heart rate there.
    heart_rate_bpm_average: ?f64,
};

/// Where the runner was, every `track_m` of odometer.
pub const TrackPoint = struct {
    latitude: f64,
    longitude: f64,
    /// Race time: negative before the first checkpoint.
    duration_s: f64,
    /// Progress along the plan, from the first checkpoint; null before the runner first got
    /// on route. Doesn't move while off route.
    distance_m: ?f64,
    on_route: bool,
};

/// The span of the trace the plan covers, and when the runner started it.
pub const Span = struct {
    index_first: usize,
    index_last: usize,
    epoch_s_origin: f64,
    /// Points this close to the end count as reached as the finish does: a runner who
    /// finished within this of it without passing it (see `Actual.epoch_s_reaching_within`).
    finish_tolerance_m: f64,
};

/// The plan's profile every `step_m` from `span.index_first` to `span.index_last`, the last
/// point included whatever the step. The caller owns the result.
pub fn profile(
    allocator: std.mem.Allocator,
    trace: *const gpxz.Trace,
    timeline: *const Timeline,
    actual: *const Actual,
    span: *const Span,
    step_m: f64,
) ![]ProfilePoint {
    assert(span.index_first < span.index_last and span.index_last < trace.points.len);
    assert(step_m > 0);
    const distances = trace.distances_m_cumulative;
    const origin_m = distances[span.index_first];
    const length_m = distances[span.index_last] - origin_m;
    assert(length_m > 0);
    // Every whole step, then the end unless the last step landed on it.
    const steps: usize = @intFromFloat(@floor(length_m / step_m));
    const ends_on_step = @as(f64, @floatFromInt(steps)) * step_m == length_m;
    const count = steps + 1 + @intFromBool(!ends_on_step);

    const points = try allocator.alloc(ProfilePoint, count);
    var index = span.index_first;
    for (points, 0..) |*point, number| {
        const distance_m = if (number == count - 1)
            distances[span.index_last]
        else
            origin_m + step_m * @as(f64, @floatFromInt(number));
        // Bounded by the last index: every point lies at or before it.
        while (index + 1 < span.index_last and distances[index + 1] < distance_m) index += 1;
        assert(distances[index] <= distance_m and distance_m <= distances[index + 1]);
        point.* = point_at(trace, timeline, actual, span, index, distance_m);
        point.distance_m -= origin_m;
    }
    heart_rates_fill(points, actual, span.epoch_s_origin);
    assert(points[0].distance_m == 0);
    assert(points[count - 1].distance_m == length_m);
    return points;
}

/// The profile at `distance_m` (from the trace's first point), between points `index` and
/// `index + 1`. Leaves the heart rate for `heart_rates_fill`.
fn point_at(
    trace: *const gpxz.Trace,
    timeline: *const Timeline,
    actual: *const Actual,
    span: *const Span,
    index: usize,
    distance_m: f64,
) ProfilePoint {
    const distances = trace.distances_m_cumulative;
    const span_m = distances[index + 1] - distances[index];
    const fraction = if (span_m > 0) (distance_m - distances[index]) / span_m else 0.0;
    assert(fraction >= 0 and fraction <= 1);
    const before = trace.points[index];
    const after = trace.points[index + 1];
    // Near the end, reached as the finish is, so the last point agrees with the finish.
    const end_m = distances[span.index_last];
    assert(distance_m <= end_m);
    const tolerance_m = if (end_m - distance_m <= span.finish_tolerance_m)
        span.finish_tolerance_m
    else
        0.0;
    const epoch_s = actual.epoch_s_reaching_within(distance_m, tolerance_m);
    return .{
        .distance_m = distance_m,
        .elevation_m = before[2] + fraction * (after[2] - before[2]),
        .latitude = before[0] + fraction * (after[0] - before[0]),
        .longitude = before[1] + fraction * (after[1] - before[1]),
        .duration_s_planned = timeline.duration_s_at(trace, index, distance_m),
        .duration_s_actual = if (epoch_s) |value| value - span.epoch_s_origin else null,
        .heart_rate_bpm_average = null,
    };
}

fn heart_rates_fill(points: []ProfilePoint, actual: *const Actual, epoch_s_origin: f64) void {
    assert(points.len >= 2);
    assert(points[0].heart_rate_bpm_average == null);
    for (points[1..], points[0 .. points.len - 1]) |*point, *previous| {
        const from = previous.duration_s_actual orelse continue;
        const to = point.duration_s_actual orelse continue;
        // Progress never goes back, so neither does the time it was first reached.
        assert(to >= from);
        point.heart_rate_bpm_average = actual.heart_rate_bpm_average(
            epoch_s_origin + from,
            epoch_s_origin + to,
        );
    }
}

/// The samples, thinned to one every `step_m` of odometer. The first and the last are kept,
/// and so are the samples on both sides of every change between on and off route, so a
/// deviation starts and ends where it did. `origin_m` is the first checkpoint's distance
/// along the trace. The caller owns the result.
pub fn track(
    allocator: std.mem.Allocator,
    samples: []const Sample,
    match: *const Match,
    origin_m: f64,
    epoch_s_origin: f64,
    step_m: f64,
) ![]TrackPoint {
    assert(samples.len == match.on_route.len and samples.len == match.odometer_m.len);
    assert(samples.len > 0 and step_m > 0);
    var points: std.ArrayList(TrackPoint) = .empty;
    errdefer points.deinit(allocator);

    var odometer_m_kept = match.odometer_m[0];
    for (samples, 0..) |*sample, index| {
        const on_route = match.on_route[index];
        const is_end = index == 0 or index == samples.len - 1;
        const is_edge = (index > 0 and match.on_route[index - 1] != on_route) or
            (index + 1 < samples.len and match.on_route[index + 1] != on_route);
        const is_step = match.odometer_m[index] - odometer_m_kept >= step_m;
        if (!is_end and !is_edge and !is_step) continue;
        odometer_m_kept = match.odometer_m[index];
        try points.append(allocator, .{
            .latitude = sample.latitude,
            .longitude = sample.longitude,
            .duration_s = @as(f64, @floatFromInt(sample.epoch_s)) - epoch_s_origin,
            .distance_m = if (match.progress_m[index]) |value| value - origin_m else null,
            .on_route = on_route,
        });
    }
    assert(points.items.len >= @min(samples.len, 2));
    assert(points.items.len <= samples.len);
    return points.toOwnedSlice(allocator);
}

fn sample_test(epoch_s: i64, latitude: f64, heart_rate_bpm: ?u8) Sample {
    return .{
        .epoch_s = epoch_s,
        .latitude = latitude,
        .longitude = 6.0,
        .altitude_m = null,
        .distance_m = null,
        .heart_rate_bpm = heart_rate_bpm,
    };
}

test "profile: every step, the end, and both clocks" {
    const allocator = testing.allocator;
    // Four points 0.001° of latitude apart (about 111 m each), climbing 10 m each.
    const coordinates = [_][3]f64{
        .{ 45.000, 6.0, 100 }, .{ 45.001, 6.0, 110 }, .{ 45.002, 6.0, 120 }, .{ 45.003, 6.0, 130 },
    };
    var trace = try gpxz.Trace.init(allocator, &coordinates);
    defer trace.deinit(allocator);
    const distances = trace.distances_m_cumulative;
    var durations = [_]f64{ 0, 60, 120, 180 };
    var efforts = [_]f64{ 0, 0, 0, 0 };
    const timeline: Timeline = .{
        .duration_s_arrival = &durations,
        .distance_km_effort = &efforts,
    };

    // The runner reaches each trace point 70 s apart, and stops short of the last.
    var samples = [_]Sample{
        sample_test(1000, 45.000, 120), sample_test(1070, 45.001, 130),
        sample_test(1140, 45.002, 140), sample_test(1210, 45.0025, 150),
    };
    var progress = [_]?f64{ distances[0], distances[1], distances[2], distances[2] + 50 };
    var offsets = [_]f64{ 0, 0, 0, 0 };
    var on_route = [_]bool{ true, true, true, true };
    var odometer = [_]f64{ 0, distances[1], distances[2], distances[2] + 50 };
    var stopped = [_]bool{ false, false, false, false };
    const match: Match = .{
        .progress_m = &progress,
        .offset_m = &offsets,
        .on_route = &on_route,
        .odometer_m = &odometer,
        .stopped = &stopped,
        .samples_off_route = 0,
        .distance_m_off_route = 0,
        .offset_m_max_on_route = 0,
        .deviations = &.{},
    };
    const actual = Actual.init(&samples, &match);
    var span: Span = .{
        .index_first = 0,
        .index_last = 3,
        .epoch_s_origin = 1000,
        .finish_tolerance_m = 0,
    };

    const points = try profile(allocator, &trace, &timeline, &actual, &span, 100);
    defer allocator.free(points);

    // About 333 m: 0, 100, 200, 300, and the end.
    try testing.expectEqual(@as(usize, 5), points.len);
    try testing.expectEqual(@as(f64, 200), points[2].distance_m);
    try testing.expectEqual(distances[3], points[4].distance_m);
    try testing.expectEqual(@as(f64, 100), points[0].elevation_m);
    try testing.expectEqual(@as(f64, 130), points[4].elevation_m);
    // The points are evenly spaced, so each value grows in proportion to the distance.
    const fraction = 200 / distances[1];
    try testing.expectApproxEqAbs(100 + 10 * fraction, points[2].elevation_m, 1e-6);
    try testing.expectEqual(@as(f64, 180), points[4].duration_s_planned);
    try testing.expectApproxEqAbs(60 * fraction, points[2].duration_s_planned, 1e-6);
    try testing.expectEqual(@as(?f64, 0), points[0].duration_s_actual);
    try testing.expectApproxEqAbs(70 * fraction, points[2].duration_s_actual.?, 1e-6);
    // Never reached: no time, and no heart rate on the way there.
    try testing.expectEqual(@as(?f64, null), points[4].duration_s_actual);
    try testing.expectEqual(@as(?f64, null), points[4].heart_rate_bpm_average);
    try testing.expectEqual(@as(?f64, null), points[0].heart_rate_bpm_average);
    // 0 to ~100 m lies inside the first interval, which ends after it: no heart rate there.
    // ~100 to ~200 m spans the end of that interval, at a sample with 130 bpm.
    try testing.expectEqual(@as(?f64, null), points[1].heart_rate_bpm_average);
    try testing.expectEqual(@as(?f64, 130), points[2].heart_rate_bpm_average);

    // Within the finish tolerance of the end, the points are reached when the runner got
    // closest, at the last sample: as the finish checkpoint would be.
    span.finish_tolerance_m = 100;
    const finished = try profile(allocator, &trace, &timeline, &actual, &span, 100);
    defer allocator.free(finished);
    try testing.expectEqual(@as(?f64, 210), finished[3].duration_s_actual);
    try testing.expectEqual(@as(?f64, 210), finished[4].duration_s_actual);
    try testing.expectEqual(points[2].duration_s_actual, finished[2].duration_s_actual);
}

test "profile: a length that is a whole number of steps ends on its last step" {
    const allocator = testing.allocator;
    const coordinates = [_][3]f64{ .{ 45.000, 6.0, 0 }, .{ 45.001, 6.0, 0 } };
    var trace = try gpxz.Trace.init(allocator, &coordinates);
    defer trace.deinit(allocator);
    var durations = [_]f64{ 0, 60 };
    var efforts = [_]f64{ 0, 0 };
    const timeline: Timeline = .{
        .duration_s_arrival = &durations,
        .distance_km_effort = &efforts,
    };
    var samples = [_]Sample{sample_test(0, 45.0, null)};
    var progress = [_]?f64{null};
    var offsets = [_]f64{0};
    var flags = [_]bool{false};
    var odometer = [_]f64{0};
    var stopped = [_]bool{false};
    const match: Match = .{
        .progress_m = &progress,
        .offset_m = &offsets,
        .on_route = &flags,
        .odometer_m = &odometer,
        .stopped = &stopped,
        .samples_off_route = 1,
        .distance_m_off_route = 0,
        .offset_m_max_on_route = 0,
        .deviations = &.{},
    };
    const actual = Actual.init(&samples, &match);
    const span: Span = .{
        .index_first = 0,
        .index_last = 1,
        .epoch_s_origin = 0,
        .finish_tolerance_m = 0,
    };

    const length_m = trace.distances_m_cumulative[1];
    const points = try profile(allocator, &trace, &timeline, &actual, &span, length_m / 2);
    defer allocator.free(points);
    try testing.expectEqual(@as(usize, 3), points.len);
    try testing.expectEqual(length_m, points[2].distance_m);
    // An activity that never got on route: no actual times anywhere.
    for (points) |*point| try testing.expectEqual(@as(?f64, null), point.duration_s_actual);
}

test "track: thinned by odometer, ends and deviation edges kept" {
    const allocator = testing.allocator;
    var samples: [10]Sample = undefined;
    var progress: [10]?f64 = undefined;
    var odometer: [10]f64 = undefined;
    for (&samples, &progress, &odometer, 0..) |*sample, *value, *meters, number| {
        const step: f64 = @floatFromInt(number);
        sample.* = sample_test(@intCast(100 + 10 * number), 45.0 + step * 1e-4, null);
        value.* = 1000 + 20 * step;
        meters.* = 20 * step;
    }
    progress[0] = null;
    // Off route for samples 5 and 6.
    var on_route = [_]bool{ true, true, true, true, true, false, false, true, true, true };
    var offsets = [_]f64{0} ** 10;
    var stopped = [_]bool{false} ** 10;
    const match: Match = .{
        .progress_m = &progress,
        .offset_m = &offsets,
        .on_route = &on_route,
        .odometer_m = &odometer,
        .stopped = &stopped,
        .samples_off_route = 2,
        .distance_m_off_route = 40,
        .offset_m_max_on_route = 0,
        .deviations = &.{},
    };

    const points = try track(allocator, &samples, &match, 1000, 120, 50);
    defer allocator.free(points);

    // 0 (first), 3 (60 m), 4..7 (edges of the deviation), 9 (last): 7 of the 10 samples.
    try testing.expectEqual(@as(usize, 7), points.len);
    try testing.expectEqual(@as(f64, -20), points[0].duration_s);
    try testing.expectEqual(@as(?f64, null), points[0].distance_m);
    try testing.expectEqual(@as(?f64, 60), points[1].distance_m);
    try testing.expect(points[2].on_route and !points[3].on_route);
    try testing.expect(!points[4].on_route and points[5].on_route);
    try testing.expectEqual(@as(f64, 70), points[6].duration_s);

    // A single sample is both the first and the last.
    const one = try track(allocator, samples[0..1], &match_prefix(&match, 1), 1000, 100, 50);
    defer allocator.free(one);
    try testing.expectEqual(@as(usize, 1), one.len);
}

fn match_prefix(match: *const Match, count: usize) Match {
    assert(count <= match.progress_m.len);
    var prefix = match.*;
    prefix.progress_m = match.progress_m[0..count];
    prefix.offset_m = match.offset_m[0..count];
    prefix.on_route = match.on_route[0..count];
    prefix.odometer_m = match.odometer_m[0..count];
    prefix.stopped = match.stopped[0..count];
    return prefix;
}
