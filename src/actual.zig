//! Questions asked of a matched activity: when did the runner first reach a distance along
//! the trace, how long were they stopped between two times or around a checkpoint, and what
//! was their heart rate meanwhile. Progress never decreases, so "first reached" is a binary
//! search.

const std = @import("std");
const assert = std.debug.assert;
const testing = std.testing;
const Sample = @import("activity.zig").Sample;
const Match = @import("match.zig").Match;
const gpxz = @import("gpxz");

/// A day at 1 Hz: no checkpoint stop is longer, and the scan stays bounded.
const dwell_samples_max = 24 * 3600;

/// Where and when a runner reached a checkpoint, for `Actual.dwell_s`.
pub const Dwell = struct {
    epoch_s_arrival: f64,
    latitude: f64,
    longitude: f64,
    /// Along the trace, from its first point.
    distance_m: f64,
    radius_m: f64,
    leave_m: f64,
};

pub const Actual = struct {
    samples: []const Sample,
    match: *const Match,

    pub fn init(samples: []const Sample, match: *const Match) Actual {
        assert(samples.len == match.progress_m.len);
        assert(samples.len == match.stopped.len);
        assert(samples.len == match.on_route.len and samples.len == match.odometer_m.len);
        return .{ .samples = samples, .match = match };
    }

    /// The Unix time, interpolated between samples, at which progress first reached
    /// `distance_m`; null if it never did.
    pub fn epoch_s_reaching(self: *const Actual, distance_m: f64) ?f64 {
        const progress = self.match.progress_m;
        var low: usize = 0;
        var high: usize = progress.len;
        while (low < high) {
            const middle = low + (high - low) / 2;
            const reached = if (progress[middle]) |value| value >= distance_m else false;
            if (reached) high = middle else low = middle + 1;
        }
        if (low == progress.len) return null;
        const after = progress[low].?;
        const epoch_s_after: f64 = @floatFromInt(self.samples[low].epoch_s);
        if (low == 0) return epoch_s_after;
        const before = progress[low - 1] orelse return epoch_s_after;
        assert(before < distance_m and distance_m <= after);
        const epoch_s_before: f64 = @floatFromInt(self.samples[low - 1].epoch_s);
        const fraction = (distance_m - before) / (after - before);
        const epoch_s = epoch_s_before + fraction * (epoch_s_after - epoch_s_before);
        assert(epoch_s >= epoch_s_before and epoch_s <= epoch_s_after);
        return epoch_s;
    }

    /// Like `epoch_s_reaching`, but a runner who got within `tolerance_m` of the distance
    /// without passing it (the finish arch sits a few meters short of the trace's end, or
    /// the watch stopped under it) counts as reaching it when they got closest.
    pub fn epoch_s_reaching_within(self: *const Actual, distance_m: f64, tolerance_m: f64) ?f64 {
        assert(tolerance_m >= 0);
        if (self.epoch_s_reaching(distance_m)) |epoch_s| return epoch_s;
        const farthest = self.match.progress_m_max() orelse return null;
        // Progress never reached `distance_m`, or the search above would have found it.
        assert(farthest < distance_m);
        if (farthest < distance_m - tolerance_m) return null;
        return self.epoch_s_reaching(farthest).?;
    }

    /// Stopped time over the intervals ending in (`epoch_s_from`, `epoch_s_to`].
    pub fn stopped_s_between(self: *const Actual, epoch_s_from: f64, epoch_s_to: f64) f64 {
        assert(epoch_s_from <= epoch_s_to);
        var stopped_s: f64 = 0;
        var index = self.index_after(epoch_s_from);
        while (index < self.samples.len) : (index += 1) {
            const epoch_s: f64 = @floatFromInt(self.samples[index].epoch_s);
            if (epoch_s > epoch_s_to) break;
            if (index > 0 and self.match.stopped[index]) stopped_s += self.interval_s(index);
        }
        // Whole intervals are counted, so the first may start up to one interval early.
        assert(stopped_s >= 0);
        return stopped_s;
    }

    /// Time at a checkpoint: from `epoch_s_arrival` to the last sample within `radius_m` of
    /// its position before the runner moves on (progress past `distance_m` + `leave_m`).
    /// Stopped time alone misses the slow walking around an aid station; this doesn't. A
    /// runner passing straight through spends the time to cover `radius_m`.
    pub fn dwell_s(self: *const Actual, dwell: *const Dwell) f64 {
        assert(dwell.radius_m > 0 and dwell.leave_m > dwell.radius_m);
        const point: [3]f64 = .{ dwell.latitude, dwell.longitude, 0 };
        var index = self.index_after(dwell.epoch_s_arrival);
        var epoch_s_last = dwell.epoch_s_arrival;
        var scanned: u64 = 0;
        while (index < self.samples.len and scanned < dwell_samples_max) : (index += 1) {
            if (self.match.progress_m[index]) |progress| {
                if (progress > dwell.distance_m + dwell.leave_m) break;
            }
            const sample = &self.samples[index];
            const here: [3]f64 = .{ sample.latitude, sample.longitude, 0 };
            if (gpxz.gps_point.distance(point, here) <= dwell.radius_m) {
                epoch_s_last = @floatFromInt(sample.epoch_s);
            }
            scanned += 1;
        }
        const duration_s = @max(epoch_s_last - dwell.epoch_s_arrival, 0);
        assert(duration_s >= 0);
        return duration_s;
    }

    /// Odometer distance run off route over the intervals ending in (from, to].
    pub fn off_route_m_between(self: *const Actual, epoch_s_from: f64, epoch_s_to: f64) f64 {
        assert(epoch_s_from <= epoch_s_to);
        const odometer = self.match.odometer_m;
        var distance_m: f64 = 0;
        var index = @max(self.index_after(epoch_s_from), 1);
        while (index < self.samples.len) : (index += 1) {
            const epoch_s: f64 = @floatFromInt(self.samples[index].epoch_s);
            if (epoch_s > epoch_s_to) break;
            if (self.match.on_route[index]) continue;
            distance_m += odometer[index] - odometer[index - 1];
        }
        assert(distance_m >= 0);
        return distance_m;
    }

    /// Time-weighted average heart rate over the intervals ending in (from, to], or null
    /// without a heart rate there.
    pub fn heart_rate_bpm_average(self: *const Actual, epoch_s_from: f64, epoch_s_to: f64) ?f64 {
        assert(epoch_s_from <= epoch_s_to);
        var weighted: f64 = 0;
        var weight_s: f64 = 0;
        var index = @max(self.index_after(epoch_s_from), 1);
        while (index < self.samples.len) : (index += 1) {
            const epoch_s: f64 = @floatFromInt(self.samples[index].epoch_s);
            if (epoch_s > epoch_s_to) break;
            const heart_rate = self.samples[index].heart_rate_bpm orelse continue;
            const duration_s = self.interval_s(index);
            weighted += @as(f64, @floatFromInt(heart_rate)) * duration_s;
            weight_s += duration_s;
        }
        if (weight_s == 0) return null;
        const average = weighted / weight_s;
        assert(average > 0 and average < 255);
        return average;
    }

    /// The first sample after `epoch_s`.
    fn index_after(self: *const Actual, epoch_s: f64) usize {
        assert(std.math.isFinite(epoch_s));
        var low: usize = 0;
        var high: usize = self.samples.len;
        while (low < high) {
            const middle = low + (high - low) / 2;
            const after = @as(f64, @floatFromInt(self.samples[middle].epoch_s)) > epoch_s;
            if (after) high = middle else low = middle + 1;
        }
        assert(low <= self.samples.len);
        if (low < self.samples.len) {
            assert(@as(f64, @floatFromInt(self.samples[low].epoch_s)) > epoch_s);
        }
        return low;
    }

    fn interval_s(self: *const Actual, index: usize) f64 {
        assert(index > 0 and index < self.samples.len);
        const seconds = self.samples[index].epoch_s - self.samples[index - 1].epoch_s;
        assert(seconds >= 0);
        return @floatFromInt(seconds);
    }
};

fn sample_test(epoch_s: i64, heart_rate_bpm: ?u8) Sample {
    return .{
        .epoch_s = epoch_s,
        .latitude = 0,
        .longitude = 0,
        .altitude_m = null,
        .distance_m = null,
        .heart_rate_bpm = heart_rate_bpm,
    };
}

const Fixture = struct {
    samples: [6]Sample,
    progress: [6]?f64,
    stopped: [6]bool,
};

fn fixture_match(fixture: *Fixture, odometer: []f64, on_route: []bool) Match {
    return .{
        .progress_m = &fixture.progress,
        .offset_m = odometer,
        .on_route = on_route,
        .odometer_m = odometer,
        .stopped = &fixture.stopped,
        .samples_off_route = 0,
        .distance_m_off_route = 0,
        .offset_m_max_on_route = 0,
        .deviations = &.{},
    };
}

test "Actual: reaching, stopping and heart rate on a small run" {
    var fixture: Fixture = .{
        .samples = .{
            sample_test(100, null), sample_test(110, 120), sample_test(120, 130),
            sample_test(130, 140),  sample_test(140, 150), sample_test(150, 160),
        },
        // Not on route at first; stands still between 120 and 140 at 200 m.
        .progress = .{ null, 0, 200, 200, 200, 400 },
        .stopped = .{ false, false, false, true, true, false },
    };
    // 10 m per interval; off route over the two intervals ending at samples 4 and 5.
    var odometer = [_]f64{ 0, 10, 20, 30, 40, 50 };
    var on_route = [_]bool{ false, true, true, true, false, false };
    const match = fixture_match(&fixture, &odometer, &on_route);
    const actual = Actual.init(&fixture.samples, &match);

    try testing.expectEqual(@as(?f64, 110), actual.epoch_s_reaching(0));
    try testing.expectEqual(@as(?f64, 115), actual.epoch_s_reaching(100));
    try testing.expectEqual(@as(?f64, 120), actual.epoch_s_reaching(200));
    try testing.expectEqual(@as(?f64, 145), actual.epoch_s_reaching(300));
    try testing.expectEqual(@as(?f64, 150), actual.epoch_s_reaching(400));
    try testing.expectEqual(@as(?f64, null), actual.epoch_s_reaching(400.5));
    try testing.expectEqual(@as(?f64, 150), actual.epoch_s_reaching_within(450, 50));
    try testing.expectEqual(@as(?f64, null), actual.epoch_s_reaching_within(451, 50));

    try testing.expectEqual(@as(f64, 20), actual.stopped_s_between(100, 150));
    try testing.expectEqual(@as(f64, 10), actual.stopped_s_between(130, 150));
    try testing.expectEqual(@as(f64, 0), actual.stopped_s_between(140, 150));
    // Every sample sits at 0°, 0°: within any radius of it until progress moves on.
    var dwell: Dwell = .{
        .epoch_s_arrival = 120,
        .latitude = 0,
        .longitude = 0,
        .distance_m = 200,
        .radius_m = 50,
        .leave_m = 100,
    };
    try testing.expectEqual(@as(f64, 20), actual.dwell_s(&dwell));
    dwell.latitude = 1.0;
    try testing.expectEqual(@as(f64, 0), actual.dwell_s(&dwell));
    dwell.latitude = 0;
    dwell.leave_m = 300;
    try testing.expectEqual(@as(f64, 30), actual.dwell_s(&dwell));

    try testing.expectEqual(@as(?f64, 140), actual.heart_rate_bpm_average(100, 150));
    try testing.expectEqual(@as(?f64, 160), actual.heart_rate_bpm_average(140, 150));
    try testing.expectEqual(@as(?f64, null), actual.heart_rate_bpm_average(150, 150));

    try testing.expectEqual(@as(f64, 20), actual.off_route_m_between(100, 150));
    try testing.expectEqual(@as(f64, 0), actual.off_route_m_between(100, 130));
    try testing.expectEqual(@as(f64, 10), actual.off_route_m_between(140, 150));
}
