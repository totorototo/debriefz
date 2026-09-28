//! The plan at every trace point: when the runner was meant to get there, and how much
//! effort-weighted distance they would have covered. gpxz's plan gives times only at the
//! checkpoints; climbs and per-kilometer splits need them in between.
//!
//! Each range between two checkpoints is run through gpxz's segment model one step at a
//! time, for the shape, then scaled so it ends exactly at the plan's next arrival. So the
//! timeline agrees with gpxz at every checkpoint, whatever the model does at a LifeBase
//! (gpxz also credits some recovery there, which this pass doesn't redo: effort distances
//! after a LifeBase run slightly high).

const std = @import("std");
const assert = std.debug.assert;
const testing = std.testing;
const gpxz = @import("gpxz");

pub const Timeline = struct {
    /// Per trace point: planned race time on arrival there. Before the first checkpoint it is
    /// 0; at a checkpoint it is the arrival, and the stop comes before the next point.
    duration_s_arrival: []f64,
    /// Per trace point: effort-weighted kilometers from the first checkpoint, as gpxz's
    /// fatigue model counts them.
    distance_km_effort: []f64,

    pub fn deinit(self: *Timeline, allocator: std.mem.Allocator) void {
        allocator.free(self.duration_s_arrival);
        allocator.free(self.distance_km_effort);
        self.* = undefined;
    }
};

pub fn compute(
    allocator: std.mem.Allocator,
    trace: *const gpxz.Trace,
    plan: []const gpxz.PlanEntry,
    settings: *const gpxz.Settings,
) !Timeline {
    settings.assert_valid();
    assert(plan.len >= 2);
    assert(plan[plan.len - 1].index < trace.points.len);
    const count = trace.points.len;
    const durations = try allocator.alloc(f64, count);
    errdefer allocator.free(durations);
    const efforts = try allocator.alloc(f64, count);
    errdefer allocator.free(efforts);

    const first = plan[0].index;
    @memset(durations[0 .. first + 1], 0);
    @memset(efforts[0 .. first + 1], 0);
    const model: gpxz.segment.Model = .{
        .pace_s_per_km = settings.pace_base_s_per_km,
        .fatigue_coefficient = settings.fatigue_coefficient,
        .clock_start_s = plan[0].epoch_s_cutoff,
    };
    var progress: gpxz.segment.Progress = .{};
    for (plan[0 .. plan.len - 1], plan[1..]) |*from, *to| {
        assert(to.index > from.index);
        const weather = settings.weather.find(to.name);
        range_fill(trace, from, to, model, weather, &progress, durations, efforts);
    }
    const last = plan[plan.len - 1].index;
    @memset(durations[last + 1 ..], durations[last]);
    @memset(efforts[last + 1 ..], efforts[last]);

    assert(durations[last] == plan[plan.len - 1].duration_s_arrival);
    assert(durations[count - 1] == durations[last] and efforts[count - 1] == efforts[last]);
    return .{ .duration_s_arrival = durations, .distance_km_effort = efforts };
}

fn range_fill(
    trace: *const gpxz.Trace,
    from: *const gpxz.PlanEntry,
    to: *const gpxz.PlanEntry,
    model: gpxz.segment.Model,
    weather: gpxz.pace_model.WeatherConditions,
    progress: *gpxz.segment.Progress,
    durations: []f64,
    efforts: []f64,
) void {
    assert(from.index < to.index);
    // First pass: the model's own step durations, relative to the departure.
    var elapsed_s: f64 = 0;
    for (from.index..to.index) |index| {
        const metrics = gpxz.segment.metrics_compute(
            trace,
            index,
            index + 1,
            model,
            weather,
            progress,
        );
        elapsed_s += metrics.duration_s;
        durations[index + 1] = elapsed_s;
        efforts[index + 1] = progress.distance_m_effort / 1000.0;
    }
    // Second pass: scale onto the plan, so the range ends at the planned arrival.
    const planned_s = to.duration_s_arrival - from.duration_s_departure;
    assert(planned_s >= 0);
    const scale = if (elapsed_s > 0) planned_s / elapsed_s else 0.0;
    for (from.index + 1..to.index + 1) |index| {
        durations[index] = from.duration_s_departure + durations[index] * scale;
    }
    // The last write is the planned arrival up to rounding; make it exact.
    durations[to.index] = to.duration_s_arrival;
    assert(durations[from.index + 1] >= from.duration_s_departure);
}

const route_gpx =
    \\<gpx><trk><trkseg>
    \\<trkpt lat="45.000" lon="6.0"><ele>1000</ele></trkpt>
    \\<trkpt lat="45.001" lon="6.0"><ele>1010</ele></trkpt>
    \\<trkpt lat="45.002" lon="6.0"><ele>1030</ele></trkpt>
    \\<trkpt lat="45.003" lon="6.0"><ele>1060</ele></trkpt>
    \\<trkpt lat="45.004" lon="6.0"><ele>1070</ele></trkpt>
    \\<trkpt lat="45.005" lon="6.0"><ele>1070</ele></trkpt>
    \\<trkpt lat="45.006" lon="6.0"><ele>1050</ele></trkpt>
    \\</trkseg></trk>
    \\<wpt lat="45.000" lon="6.0"><name>S</name><type>Start</type>
    \\<time>2026-08-21T03:00:00Z</time></wpt>
    \\<wpt lat="45.003" lon="6.0"><name>LB</name><type>LifeBase</type></wpt>
    \\<wpt lat="45.006" lon="6.0"><name>F</name><type>Arrival</type></wpt>
    \\</gpx>
;

test "compute: agrees with the plan at every checkpoint and never goes back" {
    const allocator = testing.allocator;
    const settings: gpxz.Settings = .{ .life_base_stop_s = 600 };
    var data = try gpxz.parse(allocator, route_gpx, &settings);
    defer data.deinit(allocator);
    const plan = data.plan.?;
    try testing.expectEqual(@as(usize, 3), plan.len);

    var timeline = try compute(allocator, &data.trace, plan, &settings);
    defer timeline.deinit(allocator);
    for (plan) |*entry| {
        try testing.expectApproxEqAbs(
            entry.duration_s_arrival,
            timeline.duration_s_arrival[entry.index],
            1e-6,
        );
    }
    const durations = timeline.duration_s_arrival;
    for (durations[1..], durations[0 .. durations.len - 1]) |now, before| {
        try testing.expect(now >= before);
    }
    // The point after the LifeBase comes after its 10-minute stop.
    const life_base = plan[1];
    try testing.expect(durations[life_base.index + 1] > life_base.duration_s_departure);
    // Effort distance grows, and uphill counts for more than the distance.
    const efforts = timeline.distance_km_effort;
    const distance_km = data.trace.distances_m_cumulative[life_base.index] / 1000.0;
    try testing.expect(efforts[life_base.index] > distance_km);
}
