//! Map matching: places every activity sample on the planned trace, as a distance along it.
//!
//! A nearest-point search over the whole trace is wrong on a trail race: loops start and end
//! at the same place, and out-and-back legs run the same path twice. So the search is a window
//! that slides forward with the runner: from a little behind their progress to a little ahead
//! of it, growing with the distance they have moved since their last on-route sample. Within
//! the window, a candidate also pays for straying from where the device odometer says they
//! should be, which picks the right leg of an out-and-back near its turnaround.

const std = @import("std");
const assert = std.debug.assert;
const testing = std.testing;
const gpxz = @import("gpxz");
const Sample = @import("activity.zig").Sample;

const earth_radius_m = 6_371_000.0;

pub const Settings = struct {
    /// Farther than this from the trace, a sample is off route and doesn't move progress.
    off_route_m: f64 = 75.0,
    /// The window starts this far behind progress, for GPS jitter along the trail.
    window_back_m: f64 = 50.0,
    /// And ends this far ahead of it, plus `window_growth` × the distance moved since the
    /// last on-route sample, so a GPS dropout or a detour doesn't lose the runner.
    window_ahead_m: f64 = 300.0,
    window_growth: f64 = 1.5,
    /// A bound on the window, so a long detour costs a bounded search per sample.
    window_ahead_m_max: f64 = 20_000.0,
    /// Meters of cost per meter between a candidate and the odometer's expectation.
    jump_weight: f64 = 0.2,
    /// An off-route stretch at least this long is reported as a deviation: shorter ones are
    /// GPS noise near a switchback.
    deviation_s_min: f64 = 60.0,
    /// Below this speed, averaged over the smoothing window, the runner is stopped.
    stopped_speed_m_per_s: f64 = 0.2,
    /// Samples on each side of an interval when averaging its speed.
    stopped_window_samples: u32 = 5,

    fn assert_valid(self: *const Settings) void {
        assert(self.off_route_m > 0 and self.window_back_m >= 0);
        assert(self.window_ahead_m > 0 and self.window_ahead_m <= self.window_ahead_m_max);
        assert(self.window_growth >= 1.0 and self.jump_weight >= 0);
        assert(self.stopped_speed_m_per_s > 0);
    }
};

/// A stretch run off the planned trace: a detour, a wrong turn, or a course that differed
/// from the GPX on race day.
pub const Deviation = struct {
    /// Samples [index_start, index_end) were off route.
    index_start: usize,
    index_end: usize,
    /// Progress when the runner left the trace, and when they rejoined it; null before the
    /// first on-route sample, or when the activity ended off route.
    progress_m_left: ?f64,
    progress_m_rejoined: ?f64,
    duration_s: f64,
    /// Odometer distance run off route.
    distance_m: f64,
    offset_m_max: f64,
};

pub const Match = struct {
    /// Per sample: the farthest distance along the trace reached so far, from the trace's
    /// first point. Null until the first on-route sample.
    progress_m: []?f64,
    /// Per sample: the distance to the chosen trace point, or infinity when the window held
    /// no segment.
    offset_m: []f64,
    on_route: []bool,
    /// Per sample: distance moved since the first sample. The device odometer when every
    /// sample has one and it never goes backwards, else summed GPS steps.
    odometer_m: []f64,
    /// Per sample: the interval ending at this sample was spent stopped. False at 0.
    stopped: []bool,
    samples_off_route: u32,
    /// Odometer distance covered while off route.
    distance_m_off_route: f64,
    offset_m_max_on_route: f64,
    /// Off-route stretches of at least `Settings.deviation_s_min`, in time order.
    deviations: []Deviation,

    pub fn deinit(self: *Match, allocator: std.mem.Allocator) void {
        allocator.free(self.deviations);
        allocator.free(self.progress_m);
        allocator.free(self.offset_m);
        allocator.free(self.on_route);
        allocator.free(self.odometer_m);
        allocator.free(self.stopped);
        self.* = undefined;
    }

    /// The farthest progress over the whole activity, or null if never on route.
    pub fn progress_m_max(self: *const Match) ?f64 {
        assert(self.progress_m.len > 0);
        var index = self.progress_m.len;
        while (index > 0) {
            index -= 1;
            if (self.progress_m[index]) |progress| {
                assert(progress >= 0);
                return progress;
            }
        }
        return null;
    }
};

/// Matches `samples` onto `trace`. The caller owns the result.
pub fn match(
    allocator: std.mem.Allocator,
    trace: *const gpxz.Trace,
    samples: []const Sample,
    settings: *const Settings,
) !Match {
    settings.assert_valid();
    assert(samples.len > 0);
    assert(trace.points.len >= 2);
    const count = samples.len;

    var result: Match = .{
        .progress_m = try allocator.alloc(?f64, count),
        .offset_m = undefined,
        .on_route = undefined,
        .odometer_m = undefined,
        .stopped = undefined,
        .samples_off_route = 0,
        .distance_m_off_route = 0,
        .offset_m_max_on_route = 0,
        .deviations = &.{},
    };
    errdefer allocator.free(result.progress_m);
    result.offset_m = try allocator.alloc(f64, count);
    errdefer allocator.free(result.offset_m);
    result.on_route = try allocator.alloc(bool, count);
    errdefer allocator.free(result.on_route);
    result.odometer_m = try allocator.alloc(f64, count);
    errdefer allocator.free(result.odometer_m);
    result.stopped = try allocator.alloc(bool, count);
    errdefer allocator.free(result.stopped);

    odometer_compute(samples, result.odometer_m);
    stopped_compute(samples, result.odometer_m, settings, result.stopped);
    samples_place(trace, samples, settings, &result);
    result.deviations = try deviations_compute(allocator, samples, &result, settings);
    assert(result.samples_off_route <= count);
    assert(result.deviations.len <= result.samples_off_route);
    progress_assert_monotonic(samples, result.progress_m);
    return result;
}

/// `Actual`'s binary searches rely on time and progress never going backwards: activity.zig
/// rejects a file whose time does, and `samples_place` only ever raises progress.
fn progress_assert_monotonic(samples: []const Sample, progress_m: []const ?f64) void {
    assert(samples.len == progress_m.len);
    for (samples[1..], progress_m[1..], 0..) |*sample, progress, previous| {
        assert(sample.epoch_s >= samples[previous].epoch_s);
        const before = progress_m[previous] orelse continue;
        // Once on route, progress is never null again.
        assert(progress.? >= before);
    }
}

/// Groups consecutive off-route samples into deviations, keeping those long enough.
fn deviations_compute(
    allocator: std.mem.Allocator,
    samples: []const Sample,
    result: *const Match,
    settings: *const Settings,
) ![]Deviation {
    assert(samples.len == result.on_route.len);
    var deviations: std.ArrayList(Deviation) = .empty;
    errdefer deviations.deinit(allocator);
    var index: usize = 0;
    while (index < samples.len) {
        if (result.on_route[index]) {
            index += 1;
            continue;
        }
        const start = index;
        var offset_m_max: f64 = 0;
        while (index < samples.len and !result.on_route[index]) : (index += 1) {
            const offset_m = result.offset_m[index];
            if (std.math.isFinite(offset_m)) offset_m_max = @max(offset_m_max, offset_m);
        }
        assert(index > start);
        // The stretch runs from the last sample on route to the first one back on it.
        const from = if (start > 0) start - 1 else start;
        const to = if (index < samples.len) index else index - 1;
        const duration_s: f64 = @floatFromInt(samples[to].epoch_s - samples[from].epoch_s);
        if (duration_s < settings.deviation_s_min) continue;
        try deviations.append(allocator, .{
            .index_start = start,
            .index_end = index,
            .progress_m_left = if (start > 0) result.progress_m[start - 1] else null,
            .progress_m_rejoined = if (index < samples.len) result.progress_m[index] else null,
            .duration_s = duration_s,
            .distance_m = result.odometer_m[to] - result.odometer_m[from],
            .offset_m_max = offset_m_max,
        });
    }
    assert(index == samples.len);
    assert(deviations.items.len <= result.samples_off_route);
    return deviations.toOwnedSlice(allocator);
}

/// Where the search stands between samples.
const Cursor = struct {
    /// The segment holding `progress_m`: points[segment] → points[segment + 1].
    segment: usize = 0,
    progress_m: ?f64 = null,
    /// The odometer at the last on-route sample.
    odometer_m_matched: f64 = 0,
};

fn samples_place(
    trace: *const gpxz.Trace,
    samples: []const Sample,
    settings: *const Settings,
    result: *Match,
) void {
    assert(samples.len == result.progress_m.len);
    assert(result.samples_off_route == 0);
    var cursor: Cursor = .{};
    for (samples, 0..) |*sample, index| {
        const moved_m = result.odometer_m[index] - cursor.odometer_m_matched;
        assert(moved_m >= 0);
        const search = window_search(trace, sample, &cursor, moved_m, settings);
        // The cheapest candidate, unless it is off route while another in the window isn't:
        // then the odometer's expectation misled the cost, and the nearest one is right.
        const best = if (search.cheapest.offset_m <= settings.off_route_m)
            search.cheapest
        else
            search.nearest;
        result.offset_m[index] = best.offset_m;
        const on_route = best.offset_m <= settings.off_route_m;
        result.on_route[index] = on_route;
        if (on_route) {
            const progress_m = @max(cursor.progress_m orelse best.distance_m, best.distance_m);
            if (progress_m == best.distance_m) cursor.segment = best.segment;
            cursor.progress_m = progress_m;
            cursor.odometer_m_matched = result.odometer_m[index];
            result.offset_m_max_on_route = @max(result.offset_m_max_on_route, best.offset_m);
        } else {
            result.samples_off_route += 1;
            if (index > 0 and !result.on_route[index - 1]) {
                result.distance_m_off_route += result.odometer_m[index] -
                    result.odometer_m[index - 1];
            }
        }
        result.progress_m[index] = cursor.progress_m;
    }
    assert(result.offset_m_max_on_route <= settings.off_route_m);
}

const Candidate = struct {
    segment: usize,
    /// Along the trace, from its first point.
    distance_m: f64,
    offset_m: f64,
    cost: f64,
};

const Search = struct { cheapest: Candidate, nearest: Candidate };

/// Returns the cheapest and the nearest candidates in the window around `cursor`. The window
/// always holds a segment on a trace of two points or more.
fn window_search(
    trace: *const gpxz.Trace,
    sample: *const Sample,
    cursor: *const Cursor,
    moved_m: f64,
    settings: *const Settings,
) Search {
    const distances = trace.distances_m_cumulative;
    const segment_last = trace.points.len - 2;
    const reference_m = cursor.progress_m orelse 0.0;
    const ahead_m = @min(
        settings.window_ahead_m + settings.window_growth * moved_m,
        settings.window_ahead_m_max,
    );
    const expected_m = reference_m + moved_m;

    var from = @min(cursor.segment, segment_last);
    while (from > 0 and distances[from] > reference_m - settings.window_back_m) from -= 1;

    const none: Candidate = .{
        .segment = from,
        .distance_m = reference_m,
        .offset_m = std.math.inf(f64),
        .cost = std.math.inf(f64),
    };
    var search: Search = .{ .cheapest = none, .nearest = none };
    var segment = from;
    while (segment <= segment_last and distances[segment] <= reference_m + ahead_m) {
        const projection = segment_project(trace, segment, sample.latitude, sample.longitude);
        const candidate: Candidate = .{
            .segment = segment,
            .distance_m = projection.distance_m,
            .offset_m = projection.offset_m,
            .cost = projection.offset_m +
                settings.jump_weight * @abs(projection.distance_m - expected_m),
        };
        if (candidate.cost < search.cheapest.cost) search.cheapest = candidate;
        if (candidate.offset_m < search.nearest.offset_m) search.nearest = candidate;
        segment += 1;
    }
    assert(std.math.isFinite(search.nearest.offset_m));
    assert(search.nearest.offset_m <= search.cheapest.offset_m);
    return search;
}

const Projection = struct { distance_m: f64, offset_m: f64 };

/// Projects a position onto segment points[segment] → points[segment + 1], in a local
/// equirectangular plane: exact enough over one segment of a trail.
fn segment_project(
    trace: *const gpxz.Trace,
    segment: usize,
    latitude: f64,
    longitude: f64,
) Projection {
    assert(segment + 1 < trace.points.len);
    const start = trace.points[segment];
    const end = trace.points[segment + 1];
    const radians = std.math.pi / 180.0;
    // Meters per degree of longitude (x, east) and latitude (y, north), with the segment's
    // start as the origin.
    const scale_x = @cos(start[0] * radians) * radians * earth_radius_m;
    const scale_y = radians * earth_radius_m;
    const end_x = (end[1] - start[1]) * scale_x;
    const end_y = (end[0] - start[0]) * scale_y;
    const point_x = (longitude - start[1]) * scale_x;
    const point_y = (latitude - start[0]) * scale_y;

    const length_squared = end_x * end_x + end_y * end_y;
    const fraction = if (length_squared > 0)
        std.math.clamp((point_x * end_x + point_y * end_y) / length_squared, 0.0, 1.0)
    else
        0.0;
    const gap_x = point_x - fraction * end_x;
    const gap_y = point_y - fraction * end_y;
    const distance_start = trace.distances_m_cumulative[segment];
    const distance_end = trace.distances_m_cumulative[segment + 1];
    const projection: Projection = .{
        .distance_m = distance_start + fraction * (distance_end - distance_start),
        .offset_m = @sqrt(gap_x * gap_x + gap_y * gap_y),
    };
    assert(projection.distance_m >= distance_start and projection.distance_m <= distance_end);
    assert(projection.offset_m >= 0);
    return projection;
}

/// Fills `odometer_m` from the device's distance when it is complete and never decreases,
/// else from summed great-circle steps.
fn odometer_compute(samples: []const Sample, odometer_m: []f64) void {
    assert(samples.len == odometer_m.len);
    assert(samples.len > 0);
    if (device_distance_usable(samples)) {
        const origin = samples[0].distance_m.?;
        for (samples, odometer_m) |*sample, *out| out.* = sample.distance_m.? - origin;
    } else {
        odometer_m[0] = 0;
        for (samples[1..], odometer_m[1..], 0..) |*sample, *out, previous| {
            const step_m = gpxz.gps_point.distance(
                .{ samples[previous].latitude, samples[previous].longitude, 0 },
                .{ sample.latitude, sample.longitude, 0 },
            );
            out.* = odometer_m[previous] + step_m;
        }
    }
    assert(odometer_m[0] == 0);
    assert(odometer_m[odometer_m.len - 1] >= 0);
}

fn device_distance_usable(samples: []const Sample) bool {
    assert(samples.len > 0);
    var previous: f64 = samples[0].distance_m orelse return false;
    for (samples[1..]) |*sample| {
        const distance_m = sample.distance_m orelse return false;
        if (distance_m < previous) return false;
        previous = distance_m;
    }
    assert(previous >= samples[0].distance_m.?);
    return true;
}

/// An interval is stopped when the speed over it and `stopped_window_samples` on each side is
/// below `stopped_speed_m_per_s`. The window smooths GPS jitter at an aid station, and the
/// slowest steep hiking still moves well above the threshold.
fn stopped_compute(
    samples: []const Sample,
    odometer_m: []const f64,
    settings: *const Settings,
    stopped: []bool,
) void {
    assert(samples.len == odometer_m.len and samples.len == stopped.len);
    const window: usize = settings.stopped_window_samples;
    const last = samples.len - 1;
    stopped[0] = false;
    for (1..samples.len) |index| {
        const from = (index - 1) -| window;
        const to = @min(index + window, last);
        assert(from < to);
        const duration_s: f64 = @floatFromInt(samples[to].epoch_s - samples[from].epoch_s);
        const moved_m = odometer_m[to] - odometer_m[from];
        stopped[index] = if (duration_s > 0)
            moved_m / duration_s < settings.stopped_speed_m_per_s
        else
            false;
    }
    assert(!stopped[0]);
}

// Tests run on a synthetic out-and-back: north along a meridian for 2 km, then back.

/// 0.001° of latitude, in meters on gpxz's sphere.
const step_m_per_millidegree: f64 = 111.19;

fn trace_out_and_back(allocator: std.mem.Allocator) !gpxz.Trace {
    var points: [41][3]f64 = undefined;
    for (&points, 0..) |*point, index| {
        const leg: f64 = @floatFromInt(if (index <= 20) index else 40 - index);
        // A little eastward on the way back, like two sides of one path.
        const east: f64 = if (index <= 20) 0.0 else 0.00005;
        point.* = .{ 45.0 + leg * 0.001, 6.0 + east, 1000.0 + leg };
    }
    return gpxz.Trace.init(allocator, &points);
}

fn sample_at(epoch_s: i64, latitude: f64, longitude: f64, distance_m: ?f64) Sample {
    return .{
        .epoch_s = epoch_s,
        .latitude = latitude,
        .longitude = longitude,
        .altitude_m = null,
        .distance_m = distance_m,
        .heart_rate_bpm = null,
    };
}

test "segment_project: along, before and after a segment" {
    var trace = try trace_out_and_back(testing.allocator);
    defer trace.deinit(testing.allocator);
    const middle = segment_project(&trace, 0, 45.0005, 6.0);
    try testing.expectApproxEqAbs(step_m_per_millidegree / 2, middle.distance_m, 0.5);
    try testing.expectApproxEqAbs(0, middle.offset_m, 1e-6);
    const before = segment_project(&trace, 0, 44.999, 6.0);
    try testing.expectEqual(@as(f64, 0), before.distance_m);
    try testing.expectApproxEqAbs(step_m_per_millidegree, before.offset_m, 0.5);
    const beside = segment_project(&trace, 0, 45.0005, 6.001);
    try testing.expectApproxEqAbs(78.6, beside.offset_m, 0.5);
}

test "match: an out-and-back is followed out, round the turn, and back" {
    const allocator = testing.allocator;
    var trace = try trace_out_and_back(allocator);
    defer trace.deinit(allocator);

    // One sample every 50 m, out and back on the return side of the path.
    var samples: [81]Sample = undefined;
    for (&samples, 0..) |*sample, index| {
        const along: f64 = @floatFromInt(if (index <= 40) index else 80 - index);
        const east: f64 = if (index <= 40) 0.0 else 0.00005;
        const odometer = @as(f64, @floatFromInt(index)) * step_m_per_millidegree / 2;
        sample.* = sample_at(@intCast(index * 20), 45.0 + along * 0.0005, 6.0 + east, odometer);
    }
    var result = try match(allocator, &trace, &samples, &.{});
    defer result.deinit(allocator);

    try testing.expectEqual(@as(u32, 0), result.samples_off_route);
    // Halfway out and halfway back pass the same latitude, 10 steps north: the return leg is
    // never mistaken for the way out.
    const halfway_m = 10 * step_m_per_millidegree;
    try testing.expectApproxEqAbs(halfway_m, result.progress_m[20].?, 5.0);
    try testing.expectApproxEqAbs(3 * halfway_m, result.progress_m[60].?, 5.0);
    try testing.expectApproxEqAbs(trace.distance_m, result.progress_m_max().?, 5.0);
    for (result.progress_m[1..], result.progress_m[0 .. samples.len - 1]) |now, before| {
        try testing.expect(now.? >= before.?);
    }
}

test "match: a detour is off route and progress resumes after it" {
    const allocator = testing.allocator;
    var trace = try trace_out_and_back(allocator);
    defer trace.deinit(allocator);

    var samples: [30]Sample = undefined;
    for (&samples, 0..) |*sample, index| {
        const along = @as(f64, @floatFromInt(index)) * 0.0005;
        // Samples 10 to 14 are 400 m east of the trail.
        const east: f64 = if (index >= 10 and index < 15) 0.005 else 0.0;
        sample.* = sample_at(@intCast(index * 20), 45.0 + along, 6.0 + east, null);
    }
    var result = try match(allocator, &trace, &samples, &.{});
    defer result.deinit(allocator);

    try testing.expectEqual(@as(u32, 5), result.samples_off_route);
    try testing.expect(!result.on_route[12]);
    // Progress holds through the detour, then picks up where the runner rejoined.
    try testing.expectEqual(result.progress_m[9].?, result.progress_m[14].?);
    try testing.expectApproxEqAbs(15 * step_m_per_millidegree / 2, result.progress_m[15].?, 10);
    try testing.expect(result.distance_m_off_route > 0);

    // Five samples 20 s apart, from the last on route to the first back: 120 s.
    try testing.expectEqual(@as(usize, 1), result.deviations.len);
    const deviation = result.deviations[0];
    try testing.expectEqual(@as(usize, 10), deviation.index_start);
    try testing.expectEqual(@as(usize, 15), deviation.index_end);
    try testing.expectEqual(@as(f64, 120), deviation.duration_s);
    try testing.expectEqual(result.progress_m[9], deviation.progress_m_left);
    try testing.expectEqual(result.progress_m[15], deviation.progress_m_rejoined);
    try testing.expectApproxEqAbs(393, deviation.offset_m_max, 5);

    // Raised past the stretch's duration, the same detour is not a deviation.
    var strict = try match(allocator, &trace, &samples, &.{ .deviation_s_min = 121 });
    defer strict.deinit(allocator);
    try testing.expectEqual(@as(usize, 0), strict.deviations.len);
}

test "match: never on route leaves progress null" {
    const allocator = testing.allocator;
    var trace = try trace_out_and_back(allocator);
    defer trace.deinit(allocator);
    const samples = [_]Sample{
        sample_at(0, 46.0, 7.0, 0),
        sample_at(1, 46.0, 7.0, 0),
    };
    var result = try match(allocator, &trace, &samples, &.{});
    defer result.deinit(allocator);
    try testing.expectEqual(@as(u32, 2), result.samples_off_route);
    try testing.expectEqual(@as(?f64, null), result.progress_m_max());
}

test "odometer_compute: device distance, else GPS steps" {
    var odometer: [3]f64 = undefined;
    const device = [_]Sample{
        sample_at(0, 45.0, 6.0, 10),
        sample_at(1, 45.001, 6.0, 12),
        sample_at(2, 45.002, 6.0, 20),
    };
    odometer_compute(&device, &odometer);
    try testing.expectEqualSlices(f64, &.{ 0, 2, 10 }, &odometer);

    // A missing value, or one going backwards, and the device can't be trusted.
    var backwards = device;
    backwards[2].distance_m = 11;
    odometer_compute(&backwards, &odometer);
    try testing.expectApproxEqAbs(2 * step_m_per_millidegree, odometer[2], 0.5);
    var missing = device;
    missing[1].distance_m = null;
    try testing.expect(!device_distance_usable(&missing));
}

test "stopped_compute: a pause is stopped, walking is not" {
    var samples: [30]Sample = undefined;
    var odometer: [30]f64 = undefined;
    for (&samples, &odometer, 0..) |*sample, *out, index| {
        // Moving at 1 m/s except samples 10 to 19, standing still.
        const moving_m: usize = if (index < 10) index else if (index < 20) 10 else index - 10;
        const moving: f64 = @floatFromInt(moving_m);
        sample.* = sample_at(@intCast(index), 45.0, 6.0, moving);
        out.* = moving;
    }
    var stopped: [30]bool = undefined;
    stopped_compute(&samples, &odometer, &.{}, &stopped);
    try testing.expect(!stopped[0]);
    try testing.expect(!stopped[3]);
    try testing.expect(stopped[15]);
    try testing.expect(!stopped[27]);
}
