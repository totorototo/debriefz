//! The race as run, as a GPX track: the activity's samples with the noise of a race day
//! dropped. It only ever drops samples, never moves or retimes one, so every point in the file
//! is one the device recorded, at the time it recorded it: the distance gets shorter, the
//! race's start, finish and duration stay as they were.
//!
//! A sample is dropped when it is
//!   1. inside a checkpoint stop: between the arrival and the departure (refilling, the bag
//!      drop, the toilets), only those two are kept, so the stop reads as a pause there;
//!   2. a step back along the course: on route without moving the runner's progress past
//!      the farthest point already reached (a turnaround, or GPS drift while walking);
//!   3. jitter: closer than `Settings.spacing_m` to the last sample kept (standing still).
//! Off-route samples are kept: a wrong turn was run, and the file says so.

const std = @import("std");
const assert = std.debug.assert;
const testing = std.testing;
const gpxz = @import("gpxz");
const Sample = @import("activity.zig").Sample;
const Match = @import("match.zig").Match;
const Checkpoint = @import("compare.zig").Checkpoint;

pub const Settings = struct {
    /// Closer than this to the last sample kept is standing still.
    spacing_m: f64 = 3.0,

    fn assert_valid(self: *const Settings) void {
        assert(self.spacing_m >= 0 and std.math.isFinite(self.spacing_m));
    }
};

/// A checkpoint stop, in Unix time: from the arrival to the departure.
pub const Stop = struct {
    epoch_s_arrival: f64,
    epoch_s_departure: f64,
};

/// The stops at the checkpoints the runner reached, in race order: the arrival and the dwell
/// `compare` measured there. The caller owns the result.
pub fn stops_from_checkpoints(
    allocator: std.mem.Allocator,
    checkpoints: []const Checkpoint,
) ![]Stop {
    var stops: std.ArrayList(Stop) = .empty;
    errdefer stops.deinit(allocator);
    for (checkpoints) |*checkpoint| {
        const arrival = checkpoint.epoch_s_arrival_actual orelse continue;
        // The finish has no stop.
        const stop_s = checkpoint.stop_s_actual orelse continue;
        assert(stop_s >= 0);
        const epoch_s_arrival: f64 = @floatFromInt(arrival);
        try stops.append(allocator, .{
            .epoch_s_arrival = epoch_s_arrival,
            .epoch_s_departure = epoch_s_arrival + stop_s,
        });
    }
    if (stops.items.len >= 2) {
        for (stops.items[1..], stops.items[0 .. stops.items.len - 1]) |*stop, *previous| {
            assert(stop.epoch_s_arrival >= previous.epoch_s_arrival);
        }
    }
    return stops.toOwnedSlice(allocator);
}

/// The indexes of the samples kept, in time order. The first and the last sample are always
/// kept, so the race keeps its start and its finish. The caller owns the result.
pub fn samples_keep(
    allocator: std.mem.Allocator,
    samples: []const Sample,
    match: *const Match,
    stops: []const Stop,
    settings: *const Settings,
) ![]u32 {
    settings.assert_valid();
    assert(samples.len > 0);
    assert(samples.len == match.progress_m.len and samples.len == match.on_route.len);
    // activity.zig refuses more samples than this.
    assert(samples.len <= std.math.maxInt(u32));

    const verdicts = try allocator.alloc(Verdict, samples.len);
    defer allocator.free(verdicts);
    @memset(verdicts, .free);
    for (stops) |*stop| stop_mark(samples, stop, verdicts);
    verdicts[0] = .keep;
    verdicts[samples.len - 1] = .keep;

    var kept: std.ArrayList(u32) = .empty;
    errdefer kept.deinit(allocator);
    for (samples, verdicts, 0..) |*sample, verdict, index| {
        const keep = switch (verdict) {
            .keep => true,
            .drop => false,
            .free => sample_advances(match, index) and
                distance_m(&samples[kept.getLast()], sample) >= settings.spacing_m,
        };
        if (keep) try kept.append(allocator, @intCast(index));
    }
    assert(kept.items[0] == 0);
    assert(kept.getLast() == samples.len - 1);
    for (kept.items[1..], kept.items[0 .. kept.items.len - 1]) |index, previous| {
        assert(index > previous);
    }
    return kept.toOwnedSlice(allocator);
}

const Verdict = enum { free, keep, drop };

/// Keeps the first sample at or after the arrival and the last one at or before the departure,
/// and drops every sample between them.
fn stop_mark(samples: []const Sample, stop: *const Stop, verdicts: []Verdict) void {
    assert(stop.epoch_s_departure >= stop.epoch_s_arrival);
    const arrival = index_at_or_after(samples, stop.epoch_s_arrival) orelse return;
    var departure = arrival;
    while (departure + 1 < samples.len and
        epoch_s(&samples[departure + 1]) <= stop.epoch_s_departure) : (departure += 1)
    {}
    assert(departure >= arrival);
    verdicts[arrival] = .keep;
    verdicts[departure] = .keep;
    for (verdicts[arrival + 1 .. departure]) |*verdict| verdict.* = .drop;
}

fn index_at_or_after(samples: []const Sample, epoch_s_target: f64) ?usize {
    var low: usize = 0;
    var high: usize = samples.len;
    while (low < high) {
        const middle = low + (high - low) / 2;
        if (epoch_s(&samples[middle]) >= epoch_s_target) high = middle else low = middle + 1;
    }
    return if (low == samples.len) null else low;
}

/// Whether the sample takes the runner somewhere new: off route, before the route, or on it
/// past the farthest point reached before. `progress_m` is that farthest point, so a sample
/// moves the runner on exactly when it raises it.
fn sample_advances(match: *const Match, index: usize) bool {
    assert(index > 0);
    if (!match.on_route[index]) return true;
    const progress = match.progress_m[index] orelse return true;
    const before = match.progress_m[index - 1] orelse return true;
    assert(progress >= before);
    return progress > before;
}

fn epoch_s(sample: *const Sample) f64 {
    return @floatFromInt(sample.epoch_s);
}

fn distance_m(a: *const Sample, b: *const Sample) f64 {
    const from: [3]f64 = .{ a.latitude, a.longitude, 0 };
    return gpxz.gps_point.distance(from, .{ b.latitude, b.longitude, 0 });
}

/// The course's waypoints, each checkpoint with the actual arrival and departure as its
/// `<desc>`, on the activity's wall clock: "Arrived Sat 05:03, left Sat 05:44". `<time>`
/// stays the cutoff, as gpxz reads it. The strings are shared with `waypoints` except the
/// descriptions, which the caller frees with `waypoints_free`.
pub fn waypoints_annotate(
    allocator: std.mem.Allocator,
    waypoints: []const gpxz.Waypoint,
    checkpoints: []const Checkpoint,
    utc_offset_s: i32,
) ![]gpxz.Waypoint {
    const result = try allocator.alloc(gpxz.Waypoint, waypoints.len);
    errdefer allocator.free(result);
    @memcpy(result, waypoints);
    for (result) |*waypoint| waypoint.description = null;
    errdefer for (result) |*waypoint| if (waypoint.description) |text| allocator.free(text);

    // The checkpoints are the typed waypoints that resolved onto the trace, in the same order.
    var next: usize = 0;
    for (result) |*waypoint| {
        if (next == checkpoints.len) break;
        const checkpoint = &checkpoints[next];
        if (waypoint.type_name == null or !std.mem.eql(u8, waypoint.name, checkpoint.name)) {
            continue;
        }
        next += 1;
        waypoint.description = try description_make(allocator, checkpoint, utc_offset_s);
    }
    assert(next == checkpoints.len);
    return result;
}

pub fn waypoints_free(allocator: std.mem.Allocator, waypoints: []gpxz.Waypoint) void {
    for (waypoints) |*waypoint| if (waypoint.description) |text| allocator.free(text);
    allocator.free(waypoints);
}

fn description_make(
    allocator: std.mem.Allocator,
    checkpoint: *const Checkpoint,
    utc_offset_s: i32,
) ![]u8 {
    const arrival = checkpoint.epoch_s_arrival_actual orelse
        return allocator.dupe(u8, "Not reached");
    var out: std.Io.Writer.Allocating = .init(allocator);
    errdefer out.deinit();
    const writer = &out.writer;
    writer.writeAll("Arrived ") catch return error.OutOfMemory;
    clock_write(writer, arrival, utc_offset_s) catch return error.OutOfMemory;
    if (checkpoint.stop_s_actual) |stop_s| {
        const departure = arrival + @as(i64, @intFromFloat(@round(stop_s)));
        writer.writeAll(", left ") catch return error.OutOfMemory;
        clock_write(writer, departure, utc_offset_s) catch return error.OutOfMemory;
    }
    return out.toOwnedSlice();
}

/// `Sat 05:03`: the day of the week and the time, on the wall clock `offset_s` from UTC.
fn clock_write(writer: *std.Io.Writer, epoch_s_value: i64, offset_s: i32) !void {
    const day_s = 24 * 3600;
    const local = epoch_s_value + offset_s;
    const days = @divFloor(local, day_s);
    const seconds: u64 = @intCast(@mod(local, day_s));
    // 1970-01-01 was a Thursday.
    const weekdays = [_][]const u8{ "Thu", "Fri", "Sat", "Sun", "Mon", "Tue", "Wed" };
    const weekday = weekdays[@intCast(@mod(days, 7))];
    try writer.print("{s} {d:0>2}:{d:0>2}", .{ weekday, seconds / 3600, seconds % 3600 / 60 });
}

/// Writes the GPX file: `name`, the waypoints, and one track of the kept samples, in UTC.
pub fn write(
    writer: *std.Io.Writer,
    name: []const u8,
    waypoints: []const gpxz.Waypoint,
    samples: []const Sample,
    kept: []const u32,
) !void {
    assert(kept.len > 0);
    try writer.writeAll(
        \\<?xml version="1.0" encoding="UTF-8"?>
        \\<gpx version="1.1" creator="debriefz" xmlns="http://www.topografix.com/GPX/1/1">
        \\<metadata>
        \\  <name>
    );
    try gpxz.xml_text.escape_write(writer, name);
    try writer.writeAll("</name>\n  <desc>The race as recorded: checkpoint stops, steps back " ++
        "along the course and standing-still jitter removed.</desc>\n  <time>");
    try time_write(writer, samples[kept[0]].epoch_s);
    try writer.writeAll("</time>\n</metadata>\n");
    for (waypoints) |*waypoint| {
        try gpxz.gpx_write.waypoint_write(writer, waypoint, "\n");
        try writer.writeByte('\n');
    }
    try writer.writeAll("<trk>\n  <name>");
    try gpxz.xml_text.escape_write(writer, name);
    try writer.writeAll("</name>\n  <trkseg>\n");
    for (kept) |index| {
        const sample = &samples[index];
        try writer.print("    <trkpt lat=\"{d:.7}\" lon=\"{d:.7}\">", .{
            sample.latitude,
            sample.longitude,
        });
        if (sample.altitude_m) |altitude_m| try writer.print("<ele>{d:.1}</ele>", .{altitude_m});
        try writer.writeAll("<time>");
        try time_write(writer, sample.epoch_s);
        try writer.writeAll("</time></trkpt>\n");
    }
    try writer.writeAll("  </trkseg>\n</trk>\n</gpx>\n");
}

fn time_write(writer: *std.Io.Writer, epoch_s_value: i64) !void {
    const text = try gpxz.time.timestamp_to_iso8601(.{ .epoch_s = epoch_s_value });
    try writer.writeAll(text.slice());
}

/// A runner going east along the equator, one sample a second, 10 m apart (about 9e-5°).
fn sample_at(epoch_s_value: i64, meters_east: f64) Sample {
    return .{
        .epoch_s = epoch_s_value,
        .latitude = 0,
        .longitude = meters_east / 111_195.0,
        .altitude_m = 100,
        .distance_m = null,
        .heart_rate_bpm = null,
    };
}

fn match_make(progress_m: []?f64, on_route: []bool) Match {
    assert(progress_m.len == on_route.len);
    return .{
        .progress_m = progress_m,
        .offset_m = &.{},
        .on_route = on_route,
        .odometer_m = &.{},
        .stopped = &.{},
        .samples_off_route = 0,
        .distance_m_off_route = 0,
        .offset_m_max_on_route = 0,
        .deviations = &.{},
    };
}

test "samples_keep: steps back are dropped, the samples kept untouched" {
    // Out 0..40 m, back to 20 m (a turnaround), then on to 60 m.
    const meters = [_]f64{ 0, 10, 20, 30, 40, 30, 20, 30, 40, 50, 60 };
    var samples: [meters.len]Sample = undefined;
    var progress: [meters.len]?f64 = undefined;
    var on_route: [meters.len]bool = @splat(true);
    var farthest: f64 = 0;
    for (meters, &samples, &progress, 0..) |meters_east, *sample, *progress_m, second| {
        sample.* = sample_at(1_000 + @as(i64, @intCast(second)), meters_east);
        farthest = @max(farthest, meters_east);
        progress_m.* = farthest;
    }
    const match = match_make(&progress, &on_route);
    const kept = try samples_keep(testing.allocator, &samples, &match, &.{}, &.{});
    defer testing.allocator.free(kept);
    try testing.expectEqualSlices(u32, &.{ 0, 1, 2, 3, 4, 9, 10 }, kept);
}

test "samples_keep: off-route samples are kept, jitter isn't" {
    // 1 m steps are jitter; the off-route sample at 2 is kept though it doesn't advance.
    const meters = [_]f64{ 0, 1, 2, 12, 13, 25 };
    var samples: [meters.len]Sample = undefined;
    for (meters, &samples, 0..) |meters_east, *sample, second| {
        sample.* = sample_at(1_000 + @as(i64, @intCast(second)), meters_east);
    }
    var progress = [_]?f64{ null, 0, 0, 12, 13, 25 };
    var on_route = [_]bool{ false, true, false, true, true, true };
    const match = match_make(&progress, &on_route);
    const kept = try samples_keep(testing.allocator, &samples, &match, &.{}, &.{});
    defer testing.allocator.free(kept);
    // 1 and 2 are within 3 m of 0, and 4 of 3: jitter.
    try testing.expectEqualSlices(u32, &.{ 0, 3, 5 }, kept);

    // Without spacing, 1 joins the route and 2 is off it: both kept.
    const none: Settings = .{ .spacing_m = 0 };
    const all = try samples_keep(testing.allocator, &samples, &match, &.{}, &none);
    defer testing.allocator.free(all);
    try testing.expectEqualSlices(u32, &.{ 0, 1, 2, 3, 4, 5 }, all);

    // On route without advancing: dropped, whatever the spacing.
    var stuck = [_]?f64{ null, 0, 0, 12, 12, 25 };
    var all_on: [meters.len]bool = @splat(true);
    const match_stuck = match_make(&stuck, &all_on);
    const advancing = try samples_keep(testing.allocator, &samples, &match_stuck, &.{}, &none);
    defer testing.allocator.free(advancing);
    try testing.expectEqualSlices(u32, &.{ 0, 1, 3, 5 }, advancing);
}

test "samples_keep: a stop keeps its arrival and departure only" {
    // Arrives at 20 m at t=1002, wanders until 1006, leaves at 1007.
    const meters = [_]f64{ 0, 10, 20, 60, 90, 40, 21, 30, 40 };
    var samples: [meters.len]Sample = undefined;
    var progress: [meters.len]?f64 = undefined;
    var on_route: [meters.len]bool = @splat(true);
    var farthest: f64 = 0;
    for (meters, &samples, &progress, 0..) |meters_east, *sample, *progress_m, second| {
        sample.* = sample_at(1_000 + @as(i64, @intCast(second)), meters_east);
        farthest = @max(farthest, meters_east);
        progress_m.* = farthest;
    }
    const match = match_make(&progress, &on_route);
    const stops = [_]Stop{.{ .epoch_s_arrival = 1_001.5, .epoch_s_departure = 1_006 }};
    const kept = try samples_keep(testing.allocator, &samples, &match, &stops, &.{});
    defer testing.allocator.free(kept);
    // 7 and 8 are back short of the 90 m reached while wandering: steps back too.
    try testing.expectEqualSlices(u32, &.{ 0, 1, 2, 6, 8 }, kept);

    // A stop past the activity's end marks nothing; one sample is always kept, once.
    const late = [_]Stop{.{ .epoch_s_arrival = 2_000, .epoch_s_departure = 2_100 }};
    const kept_late = try samples_keep(testing.allocator, &samples, &match, &late, &.{});
    defer testing.allocator.free(kept_late);
    try testing.expectEqualSlices(u32, &.{ 0, 1, 2, 3, 4, 8 }, kept_late);
    const single = try samples_keep(testing.allocator, samples[0..1], &match_make(
        progress[0..1],
        on_route[0..1],
    ), &stops, &.{});
    defer testing.allocator.free(single);
    try testing.expectEqualSlices(u32, &.{0}, single);
}

fn checkpoint_make(name: []const u8, arrival: ?i64, stop_s: ?f64) Checkpoint {
    return .{
        .name = name,
        .type_name = "TimeBarrier",
        .distance_m = 0,
        .elevation_gain_m = 0,
        .duration_s_planned = 0,
        .duration_s_actual = null,
        .delta_s = null,
        .stop_s_planned = 0,
        .stop_s_actual = stop_s,
        .epoch_s_arrival_actual = arrival,
        .epoch_s_cutoff = null,
        .margin_s_planned = null,
        .margin_s_actual = null,
    };
}

test "stops_from_checkpoints: reached checkpoints with a stop" {
    const checkpoints = [_]Checkpoint{
        checkpoint_make("Start", 1_000, 60),
        checkpoint_make("Aid", 2_000, 300.4),
        checkpoint_make("Finish", 3_000, null),
        checkpoint_make("Never", null, null),
    };
    const stops = try stops_from_checkpoints(testing.allocator, &checkpoints);
    defer testing.allocator.free(stops);
    try testing.expectEqual(@as(usize, 2), stops.len);
    try testing.expectEqual(@as(f64, 1_060), stops[0].epoch_s_departure);
    try testing.expectEqual(@as(f64, 2_300.4), stops[1].epoch_s_departure);
    const empty = try stops_from_checkpoints(testing.allocator, &.{});
    try testing.expectEqual(@as(usize, 0), empty.len);
}

test "waypoints_annotate: typed waypoints get the actual times, others nothing" {
    // 2026-08-21T03:03:00Z is a Friday, 05:03 at UTC+2.
    const arrival: i64 = 1_787_281_380;
    const waypoints = [_]gpxz.Waypoint{
        .{ .latitude = 0, .longitude = 0, .name = "Start", .type_name = "Start" },
        .{ .latitude = 0, .longitude = 1, .name = "Aid", .description = "water" },
        .{ .latitude = 0, .longitude = 2, .name = "Aid", .type_name = "LifeBase" },
        .{ .latitude = 0, .longitude = 3, .name = "Finish", .type_name = "Arrival" },
    };
    const checkpoints = [_]Checkpoint{
        checkpoint_make("Start", arrival, 59),
        checkpoint_make("Aid", arrival + 22 * 3600, 1800),
        checkpoint_make("Finish", null, null),
    };
    const result = try waypoints_annotate(testing.allocator, &waypoints, &checkpoints, 7200);
    defer waypoints_free(testing.allocator, result);
    try testing.expectEqualStrings("Arrived Fri 05:03, left Fri 05:03", result[0].description.?);
    try testing.expect(result[1].description == null);
    try testing.expectEqualStrings("Arrived Sat 03:03, left Sat 03:33", result[2].description.?);
    try testing.expectEqualStrings("Not reached", result[3].description.?);
    try testing.expectEqualStrings("Aid", result[2].name);
}

test "clock_write: before 1970 and across midnight" {
    var out: std.Io.Writer.Allocating = .init(testing.allocator);
    defer out.deinit();
    try clock_write(&out.writer, -60, 0);
    try out.writer.writeByte(' ');
    try clock_write(&out.writer, 23 * 3600 + 30 * 60, 3600);
    try testing.expectEqualStrings("Wed 23:59 Fri 00:30", out.written());
}

test "write: a GPX gpxz reads back, the samples' times as recorded" {
    const samples = [_]Sample{ sample_at(1_787_281_427, 0), sample_at(1_787_281_430, 50) };
    const waypoints = [_]gpxz.Waypoint{
        .{ .latitude = 0, .longitude = 0, .name = "A & B", .type_name = "Start" },
    };
    var out: std.Io.Writer.Allocating = .init(testing.allocator);
    defer out.deinit();
    try write(&out.writer, "Race <as run>", &waypoints, &samples, &.{ 0, 1 });
    const text = out.written();
    try testing.expect(std.mem.indexOf(u8, text, "<name>Race &lt;as run&gt;</name>") != null);
    try testing.expect(std.mem.indexOf(u8, text, "<time>2026-08-21T03:03:50Z</time>") != null);

    var data = try gpxz.parse(testing.allocator, text, &.{});
    defer data.deinit(testing.allocator);
    try testing.expectEqual(@as(usize, 1), data.waypoints.len);
    try testing.expectEqualStrings("A & B", data.waypoints[0].name);
    try testing.expectEqual(@as(usize, 2), data.points_full_resolution.len / 3);
}
