//! Plan against actual: per checkpoint, per section, per climb and per kilometer, plus the
//! pace settings that would have predicted the race that was run.
//!
//! Times come in two clocks. Race time counts from the moment the runner crossed the first
//! checkpoint, so a late start line doesn't read as slow running. Cutoff margins use the
//! wall clock, since a barrier closes at a fixed time for everyone.

const std = @import("std");
const assert = std.debug.assert;
const testing = std.testing;
const gpxz = @import("gpxz");
const Activity = @import("activity.zig").Activity;
const Match = @import("match.zig").Match;
const Actual = @import("actual.zig").Actual;
const timeline_module = @import("timeline.zig");
const Timeline = timeline_module.Timeline;
const calibrate = @import("calibrate.zig");
const series = @import("series.zig");

pub const Settings = struct {
    /// Time spent within this distance of a checkpoint after reaching it is time at the
    /// checkpoint: wide enough for GPS jitter under an aid station's roof.
    checkpoint_radius_m: f64 = 75.0,
    /// The runner has left a checkpoint once this far past it along the trace.
    checkpoint_leave_m: f64 = 500.0,
    /// A runner this close to the finish without passing it has finished.
    finish_tolerance_m: f64 = 150.0,
    /// Per-kilometer splits every this many meters.
    split_m: f64 = 1000.0,
    /// A profile point every this many meters along the plan: fine enough to draw a climb.
    profile_m: f64 = 100.0,
    /// A track point every this many meters of odometer: fine enough for a map.
    track_m: f64 = 50.0,
};

pub const CompareError = error{
    /// The GPX has fewer than two typed waypoints (Start, TimeBarrier, LifeBase, Arrival), so
    /// gpxz built no plan to compare against.
    PlanMissing,
    /// No activity sample came within the off-route distance of the route.
    ActivityNotOnRoute,
};

pub const Checkpoint = struct {
    name: []const u8,
    type_name: ?[]const u8,
    distance_m: f64,
    elevation_gain_m: f64,
    duration_s_planned: f64,
    /// Null when the runner never got there.
    duration_s_actual: ?f64,
    /// Actual minus planned: positive is behind the plan.
    delta_s: ?f64,
    stop_s_planned: f64,
    stop_s_actual: ?f64,
    epoch_s_arrival_actual: ?i64,
    epoch_s_cutoff: ?i64,
    /// The cutoff minus the departure, planned and actual: below 0, it was missed.
    margin_s_planned: ?f64,
    margin_s_actual: ?f64,
};

pub const Section = struct {
    from: []const u8,
    to: []const u8,
    distance_m: f64,
    elevation_gain_m: f64,
    elevation_loss_m: f64,
    /// Planned moving time: the next arrival minus this departure.
    moving_s_planned: f64,
    /// Actual time between the two arrivals, less the time spent stopped.
    moving_s_actual: ?f64,
    stopped_s_actual: ?f64,
    /// Actual over planned moving time: above 1, slower than planned.
    pace_ratio: ?f64,
    heart_rate_bpm_average: ?f64,
    /// Odometer distance run off the planned trace within the section.
    distance_m_off_route: f64,
};

/// An off-route stretch, placed on the plan.
pub const Deviation = struct {
    /// Along the plan, from the first checkpoint: where the runner left the trace, and where
    /// they rejoined it. Null at either end of the activity.
    distance_m_left: ?f64,
    distance_m_rejoined: ?f64,
    /// Race time on leaving.
    duration_s_left: f64,
    duration_s: f64,
    distance_m: f64,
    offset_m_max: f64,
};

pub const Climb = struct {
    /// Along the plan, from the first checkpoint, as every other distance in the report:
    /// negative for a climb that starts on a lead-in before it.
    distance_m_start: f64,
    distance_m: f64,
    elevation_gain_m: f64,
    gradient_percent_average: f64,
    elevation_m_summit: f64,
    duration_s_planned: f64,
    duration_s_actual: ?f64,
    /// Vertical meters per hour, planned and actual. Planned is null for a climb outside the
    /// plan (before the first checkpoint or after the last), where no time is planned.
    vam_m_per_h_planned: ?f64,
    vam_m_per_h_actual: ?f64,
    heart_rate_bpm_average: ?f64,
};

/// Race time at each kilometer, planned and actual: the gap curve.
pub const Split = struct {
    distance_m: f64,
    duration_s_planned: f64,
    duration_s_actual: ?f64,
};

pub const Totals = struct {
    distance_m_planned: f64,
    elevation_gain_m_planned: f64,
    distance_m_device: ?f64,
    ascent_m_device: ?f64,
    duration_s_planned: f64,
    /// Race time at the finish; null without one.
    duration_s_actual: ?f64,
    moving_s_actual: ?f64,
    stopped_s_actual: ?f64,
    /// How far along the route the runner got.
    distance_m_reached: f64,
    finished: bool,
    samples: u64,
    samples_off_route: u32,
    distance_m_off_route: f64,
    records_without_position: u32,
    epoch_s_start_planned: ?i64,
    epoch_s_start_actual: i64,
    /// The device's local time minus UTC, for showing wall-clock times.
    utc_offset_s: ?i32,
};

pub const Report = struct {
    name: ?[]const u8,
    settings: gpxz.Settings,
    totals: Totals,
    checkpoints: []Checkpoint,
    sections: []Section,
    climbs: []Climb,
    splits: []Split,
    deviations: []Deviation,
    calibration: ?calibrate.Calibration,
    profile: []series.ProfilePoint,
    track: []series.TrackPoint,

    pub fn deinit(self: *Report, allocator: std.mem.Allocator) void {
        allocator.free(self.deviations);
        allocator.free(self.checkpoints);
        allocator.free(self.sections);
        allocator.free(self.climbs);
        allocator.free(self.splits);
        allocator.free(self.profile);
        allocator.free(self.track);
        if (self.calibration) |*calibration| calibration.deinit(allocator);
        self.* = undefined;
    }
};

/// Everything a comparison reads, bundled so the helpers take one pointer.
const Context = struct {
    data: *const gpxz.GPXData,
    plan: []const gpxz.PlanEntry,
    timeline: *const Timeline,
    actual: *const Actual,
    settings: *const Settings,
    /// When the runner crossed the first checkpoint.
    epoch_s_origin: f64,

    fn trace_distance_m(self: *const Context, index: usize) f64 {
        assert(index < self.data.trace.distances_m_cumulative.len);
        const distance_m = self.data.trace.distances_m_cumulative[index];
        assert(distance_m >= 0);
        return distance_m;
    }

    /// Race time on first reaching trace distance `distance_m`, or null.
    fn duration_s_at(self: *const Context, distance_m: f64, tolerance_m: f64) ?f64 {
        assert(distance_m >= 0);
        assert(tolerance_m >= 0);
        const epoch_s = self.actual.epoch_s_reaching_within(distance_m, tolerance_m) orelse
            return null;
        // Negative only for a distance before the first checkpoint.
        return epoch_s - self.epoch_s_origin;
    }
};

/// Compares `data`'s plan, built with `plan_settings`, against `activity` as matched onto
/// `data.trace`. The caller owns the report; it borrows names from `data`.
pub fn compare(
    allocator: std.mem.Allocator,
    data: *const gpxz.GPXData,
    plan_settings: *const gpxz.Settings,
    activity: *const Activity,
    match: *const Match,
    settings: *const Settings,
) !Report {
    const plan = data.plan orelse return CompareError.PlanMissing;
    if (plan.len < 2) return CompareError.PlanMissing;
    const actual = Actual.init(activity.samples, match);
    const start_m = data.trace.distances_m_cumulative[plan[0].index];
    const epoch_s_origin = actual.epoch_s_reaching(start_m) orelse
        return CompareError.ActivityNotOnRoute;

    var timeline = try timeline_module.compute(allocator, &data.trace, plan, plan_settings);
    defer timeline.deinit(allocator);
    const context: Context = .{
        .data = data,
        .plan = plan,
        .timeline = &timeline,
        .actual = &actual,
        .settings = settings,
        .epoch_s_origin = epoch_s_origin,
    };

    const checkpoints = try checkpoints_compute(allocator, &context);
    errdefer allocator.free(checkpoints);
    const sections = try sections_compute(allocator, &context, checkpoints);
    errdefer allocator.free(sections);
    const climbs = try climbs_compute(allocator, &context);
    errdefer allocator.free(climbs);
    const splits = try splits_compute(allocator, &context);
    errdefer allocator.free(splits);
    const deviations = try deviations_compute(allocator, &context, activity, match);
    errdefer allocator.free(deviations);
    const span: series.Span = .{
        .index_first = plan[0].index,
        .index_last = plan[plan.len - 1].index,
        .epoch_s_origin = epoch_s_origin,
        .finish_tolerance_m = settings.finish_tolerance_m,
    };
    const profile = try series.profile(
        allocator,
        &data.trace,
        &timeline,
        &actual,
        &span,
        settings.profile_m,
    );
    errdefer allocator.free(profile);
    const track = try series.track(
        allocator,
        activity.samples,
        match,
        start_m,
        epoch_s_origin,
        settings.track_m,
    );
    errdefer allocator.free(track);
    const calibration = try calibrate.compute(
        allocator,
        data,
        plan_settings,
        &timeline,
        checkpoints,
        sections,
    );

    const report: Report = .{
        .name = data.metadata.name,
        .settings = plan_settings.*,
        .totals = totals_compute(&context, activity, match, checkpoints),
        .checkpoints = checkpoints,
        .sections = sections,
        .climbs = climbs,
        .splits = splits,
        .deviations = deviations,
        .calibration = calibration,
        .profile = profile,
        .track = track,
    };
    assert(report.checkpoints.len == plan.len);
    assert(report.sections.len == plan.len - 1);
    // Paired with the finish checkpoint: the profile's end is reached by the same rule.
    const finish = report.checkpoints[report.checkpoints.len - 1].duration_s_actual;
    assert(std.meta.eql(finish, report.profile[report.profile.len - 1].duration_s_actual));
    return report;
}

fn checkpoints_compute(allocator: std.mem.Allocator, context: *const Context) ![]Checkpoint {
    const plan = context.plan;
    const checkpoints = try allocator.alloc(Checkpoint, plan.len);
    for (plan, checkpoints, 0..) |*entry, *checkpoint, number| {
        const is_last = number == plan.len - 1;
        const distance_m = context.trace_distance_m(entry.index);
        const tolerance_m = if (is_last) context.settings.finish_tolerance_m else 0.0;
        const duration_s = context.duration_s_at(distance_m, tolerance_m);
        // The finish has no stop; nor does a checkpoint never reached.
        const stop_s: ?f64 = if (duration_s == null or is_last) null else context.actual
            .dwell_s(&.{
            .epoch_s_arrival = context.epoch_s_origin + duration_s.?,
            .latitude = entry.latitude,
            .longitude = entry.longitude,
            .distance_m = distance_m,
            .radius_m = context.settings.checkpoint_radius_m,
            .leave_m = context.settings.checkpoint_leave_m,
        });
        const epoch_s_arrival: ?i64 = if (duration_s) |value|
            @intFromFloat(@round(context.epoch_s_origin + value))
        else
            null;
        checkpoint.* = .{
            .name = entry.name,
            .type_name = entry.type_name,
            .distance_m = entry.distance_m,
            .elevation_gain_m = entry.elevation_gain_m,
            .duration_s_planned = entry.duration_s_arrival,
            .duration_s_actual = duration_s,
            .delta_s = if (duration_s) |value| value - entry.duration_s_arrival else null,
            .stop_s_planned = entry.stop_s,
            .stop_s_actual = stop_s,
            .epoch_s_arrival_actual = epoch_s_arrival,
            .epoch_s_cutoff = entry.epoch_s_cutoff,
            // The start's `<time>` is the start, not a cutoff: no margin there.
            .margin_s_planned = if (number == 0) null else entry.margin_s,
            .margin_s_actual = margin_actual(entry, epoch_s_arrival, stop_s, number),
        };
    }
    assert(checkpoints[0].duration_s_actual.? == 0);
    // Progress never goes back and checkpoints lie in order along the trace, so reaching one
    // means having reached every one before it, and no later. `sections_compute` relies on it.
    for (checkpoints[1..], checkpoints[0 .. checkpoints.len - 1]) |*checkpoint, *previous| {
        const duration_s = checkpoint.duration_s_actual orelse continue;
        assert(duration_s >= previous.duration_s_actual.?);
    }
    return checkpoints;
}

/// The cutoff minus the actual departure. Null at the start (its `<time>` is the start
/// time, not a cutoff), and without a cutoff or an arrival.
fn margin_actual(
    entry: *const gpxz.PlanEntry,
    epoch_s_arrival: ?i64,
    stop_s: ?f64,
    number: usize,
) ?f64 {
    if (stop_s) |value| assert(value >= 0);
    if (number == 0) return null;
    const epoch_s_cutoff = entry.epoch_s_cutoff orelse return null;
    const epoch_s = epoch_s_arrival orelse return null;
    const margin_s = @as(f64, @floatFromInt(epoch_s_cutoff - epoch_s)) - (stop_s orelse 0);
    assert(margin_s <= @as(f64, @floatFromInt(epoch_s_cutoff - epoch_s)));
    return margin_s;
}

fn sections_compute(
    allocator: std.mem.Allocator,
    context: *const Context,
    checkpoints: []const Checkpoint,
) ![]Section {
    const plan = context.plan;
    assert(checkpoints.len == plan.len);
    const sections = try allocator.alloc(Section, plan.len - 1);
    for (sections, plan[0 .. plan.len - 1], plan[1..], 0..) |*section, *from, *to, number| {
        const moving_s_planned = to.duration_s_arrival - from.duration_s_departure;
        assert(moving_s_planned >= 0);
        section.* = .{
            .from = from.name,
            .to = to.name,
            .distance_m = to.distance_m - from.distance_m,
            .elevation_gain_m = to.elevation_gain_m - from.elevation_gain_m,
            .elevation_loss_m = to.elevation_loss_m - from.elevation_loss_m,
            .moving_s_planned = moving_s_planned,
            .moving_s_actual = null,
            .stopped_s_actual = null,
            .pace_ratio = null,
            .heart_rate_bpm_average = null,
            .distance_m_off_route = 0,
        };
        const start = checkpoints[number].duration_s_actual orelse continue;
        const end = checkpoints[number + 1].duration_s_actual orelse continue;
        assert(end >= start);
        const epoch_s_start = context.epoch_s_origin + start;
        const epoch_s_end = context.epoch_s_origin + end;
        const stopped_s = @min(
            context.actual.stopped_s_between(epoch_s_start, epoch_s_end),
            end - start,
        );
        const moving_s = (end - start) - stopped_s;
        section.moving_s_actual = moving_s;
        section.stopped_s_actual = stopped_s;
        section.pace_ratio = if (moving_s_planned > 0) moving_s / moving_s_planned else null;
        section.heart_rate_bpm_average = context.actual.heart_rate_bpm_average(
            epoch_s_start,
            epoch_s_end,
        );
        section.distance_m_off_route = context.actual.off_route_m_between(
            epoch_s_start,
            epoch_s_end,
        );
        assert(moving_s >= 0 and moving_s <= end - start);
    }
    return sections;
}

fn climbs_compute(allocator: std.mem.Allocator, context: *const Context) ![]Climb {
    const trace = &context.data.trace;
    // gpxz places climbs along the trace; the report counts from the first checkpoint.
    const origin_m = context.trace_distance_m(context.plan[0].index);
    const durations = context.timeline.duration_s_arrival;
    const climbs = try allocator.alloc(Climb, trace.climbs.len);
    for (trace.climbs, climbs) |*source, *climb| {
        assert(source.index_start < source.index_end);
        const planned_s = durations[source.index_end] - durations[source.index_start];
        assert(planned_s >= 0);
        const start = context.duration_s_at(context.trace_distance_m(source.index_start), 0);
        const end = context.duration_s_at(context.trace_distance_m(source.index_end), 0);
        const actual_s: ?f64 = if (start != null and end != null) end.? - start.? else null;
        climb.* = .{
            .distance_m_start = source.distance_m_start - origin_m,
            .distance_m = source.distance_m,
            .elevation_gain_m = source.elevation_gain_m,
            .gradient_percent_average = source.gradient_percent_average,
            .elevation_m_summit = source.elevation_m_summit,
            .duration_s_planned = planned_s,
            .duration_s_actual = actual_s,
            .vam_m_per_h_planned = vam(source.elevation_gain_m, planned_s),
            .vam_m_per_h_actual = if (actual_s) |value|
                vam(source.elevation_gain_m, value)
            else
                null,
            .heart_rate_bpm_average = if (actual_s != null) context.actual.heart_rate_bpm_average(
                context.epoch_s_origin + start.?,
                context.epoch_s_origin + end.?,
            ) else null,
        };
    }
    // Paired with gpxz: each climb starts where its first trace point lies, less the origin.
    for (trace.climbs, climbs) |*source, *climb| {
        const start_m = context.trace_distance_m(source.index_start) - origin_m;
        assert(@abs(climb.distance_m_start - start_m) < 1e-6);
    }
    assert(climbs.len == trace.climbs.len);
    return climbs;
}

fn deviations_compute(
    allocator: std.mem.Allocator,
    context: *const Context,
    activity: *const Activity,
    match: *const Match,
) ![]Deviation {
    const origin_m = context.trace_distance_m(context.plan[0].index);
    const deviations = try allocator.alloc(Deviation, match.deviations.len);
    for (match.deviations, deviations) |*source, *deviation| {
        assert(source.index_start < source.index_end);
        const epoch_s: f64 = @floatFromInt(activity.samples[source.index_start].epoch_s);
        deviation.* = .{
            .distance_m_left = if (source.progress_m_left) |value| value - origin_m else null,
            .distance_m_rejoined = if (source.progress_m_rejoined) |value|
                value - origin_m
            else
                null,
            .duration_s_left = epoch_s - context.epoch_s_origin,
            .duration_s = source.duration_s,
            .distance_m = source.distance_m,
            .offset_m_max = source.offset_m_max,
        };
        assert(deviation.duration_s >= 0 and deviation.distance_m >= 0);
    }
    return deviations;
}

fn vam(elevation_gain_m: f64, duration_s: f64) ?f64 {
    assert(elevation_gain_m >= 0);
    if (!(duration_s > 0)) return null;
    const vam_m_per_h = elevation_gain_m / (duration_s / 3600.0);
    assert(vam_m_per_h >= 0 and std.math.isFinite(vam_m_per_h));
    return vam_m_per_h;
}

/// Every `split_m` along the plan, from the first checkpoint to the last.
fn splits_compute(allocator: std.mem.Allocator, context: *const Context) ![]Split {
    const plan = context.plan;
    const trace = &context.data.trace;
    const origin_m = context.trace_distance_m(plan[0].index);
    const length_m = context.trace_distance_m(plan[plan.len - 1].index) - origin_m;
    assert(length_m > 0);
    const step_m = context.settings.split_m;
    assert(step_m > 0);
    const count: usize = @intFromFloat(@floor(length_m / step_m));
    const splits = try allocator.alloc(Split, count);
    const distances = trace.distances_m_cumulative;
    const index_last = plan[plan.len - 1].index;
    var index: usize = plan[0].index;
    for (splits, 1..) |*split, number| {
        const distance_m = origin_m + step_m * @as(f64, @floatFromInt(number));
        // Bounded by the last checkpoint: every split lies before it.
        while (index + 1 < index_last and distances[index + 1] < distance_m) index += 1;
        assert(distances[index + 1] >= distance_m);
        split.* = .{
            .distance_m = distance_m - origin_m,
            .duration_s_planned = context.timeline.duration_s_at(trace, index, distance_m),
            .duration_s_actual = context.duration_s_at(distance_m, 0),
        };
    }
    assert(index <= index_last);
    return splits;
}

fn totals_compute(
    context: *const Context,
    activity: *const Activity,
    match: *const Match,
    checkpoints: []const Checkpoint,
) Totals {
    const plan = context.plan;
    assert(checkpoints.len == plan.len);
    assert(activity.samples.len == match.progress_m.len);
    const last = &plan[plan.len - 1];
    const finish = checkpoints[checkpoints.len - 1].duration_s_actual;
    const stopped_s: ?f64 = if (finish) |duration_s| context.actual.stopped_s_between(
        context.epoch_s_origin,
        context.epoch_s_origin + duration_s,
    ) else null;
    const reached = match.progress_m_max().? - context.trace_distance_m(plan[0].index);
    return .{
        .distance_m_planned = last.distance_m,
        .elevation_gain_m_planned = last.elevation_gain_m,
        .distance_m_device = activity.session.distance_m,
        .ascent_m_device = activity.session.ascent_m,
        .duration_s_planned = last.duration_s_arrival,
        .duration_s_actual = finish,
        .moving_s_actual = if (finish) |duration_s| duration_s - stopped_s.? else null,
        .stopped_s_actual = stopped_s,
        .distance_m_reached = @min(@max(reached, 0), last.distance_m),
        .finished = finish != null,
        .samples = activity.samples.len,
        .samples_off_route = match.samples_off_route,
        .distance_m_off_route = match.distance_m_off_route,
        .records_without_position = activity.records_without_position,
        .epoch_s_start_planned = plan[0].epoch_s_cutoff,
        .epoch_s_start_actual = @intFromFloat(@round(context.epoch_s_origin)),
        .utc_offset_s = activity.utc_offset_s,
    };
}

test "vam: meters per hour, and none without time" {
    try testing.expectEqual(@as(?f64, 600), vam(300, 1800));
    try testing.expectEqual(@as(?f64, null), vam(300, 0));
}

test "margin_actual: the start has none, and a stop eats into the margin" {
    var entry: gpxz.PlanEntry = .{
        .name = "TB",
        .type_name = "TimeBarrier",
        .index = 1,
        .latitude = 0,
        .longitude = 0,
        .elevation_m = 0,
        .distance_m = 1000,
        .elevation_gain_m = 0,
        .elevation_loss_m = 0,
        .duration_s_arrival = 600,
        .stop_s = 0,
        .duration_s_departure = 600,
        .epoch_s_arrival = null,
        .epoch_s_cutoff = 10_000,
        .margin_s = null,
    };
    try testing.expectEqual(@as(?f64, null), margin_actual(&entry, 9_000, 60, 0));
    try testing.expectEqual(@as(?f64, 940), margin_actual(&entry, 9_000, 60, 3));
    try testing.expectEqual(@as(?f64, -500), margin_actual(&entry, 10_500, null, 3));
    try testing.expectEqual(@as(?f64, null), margin_actual(&entry, null, 60, 3));
    entry.epoch_s_cutoff = null;
    try testing.expectEqual(@as(?f64, null), margin_actual(&entry, 9_000, 60, 3));
}
