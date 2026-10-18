//! The plan as a file: `plan.json`, written by gpxz (through Regimen or `gpxz --plan`), read
//! here in place of pace options that had to match the plan's by hand. The plan names its
//! course by the SHA-256 of the GPX bytes and records the model that made it, so a debrief
//! runs against the plan the runner saw, on the course it was made for, or not at all.
//!
//! debriefz still recomputes the plan from the course: the timeline needs the model between
//! checkpoints, which plan.json doesn't hold. `entries_check` makes sure that recomputation is
//! the plan in the file.

const std = @import("std");
const assert = std.debug.assert;
const testing = std.testing;
const gpxz = @import("gpxz");

/// How far a recomputed checkpoint time may drift from the file's and still be the same plan.
/// Not zero: the file may come from Regimen's WASM build, whose `@exp` need not round like the
/// native one, and the pace model sums thousands of steps. A second is far below anything a
/// debrief shows (minutes) and far above that rounding.
pub const duration_s_tolerance = 1.0;

pub const Error = gpxz.plan.ReadError || error{CourseMismatch};

/// A plan.json read, validated and checked against the course: the model to run gpxz with,
/// and the file's checkpoints for `entries_check`. The caller frees it with `deinit`.
pub const PlanFile = struct {
    parsed: std.json.Parsed(gpxz.Plan),
    settings: gpxz.Settings,

    pub fn deinit(self: *PlanFile) void {
        self.parsed.deinit();
        self.* = undefined;
    }
};

/// Reads `plan_bytes` as plan.json and checks it was made for the course in `course_bytes`.
pub fn load(
    allocator: std.mem.Allocator,
    plan_bytes: []const u8,
    course_bytes: []const u8,
) Error!PlanFile {
    const parsed = try gpxz.plan_read(allocator, plan_bytes);
    errdefer parsed.deinit();
    const plan = &parsed.value;
    assert(gpxz.plan.plan_valid(plan));
    if (!gpxz.plan.course_matches(plan, course_bytes)) return error.CourseMismatch;

    // plan_valid has already checked pace > 0 and fatigue >= 0, both finite.
    const settings: gpxz.Settings = .{
        .pace_base_s_per_km = plan.model.pace_base_s_per_km,
        .fatigue_coefficient = plan.model.fatigue_coefficient,
        .life_base_stop_s = plan.model.life_base_stop_s,
    };
    settings.assert_valid();
    return .{ .parsed = parsed, .settings = settings };
}

/// Whether gpxz, run on the course with the file's model, gives the file's plan: the same
/// checkpoints in the same order, at the same times. A mismatch means the file was made by a
/// gpxz whose model has changed since, and a debrief against today's plan would compare the
/// race with a plan the runner never saw.
pub fn entries_check(
    file: []const gpxz.PlanEntry,
    recomputed: []const gpxz.PlanEntry,
) error{PlanStale}!void {
    if (file.len != recomputed.len) return error.PlanStale;
    for (file, recomputed) |*expected, *actual| {
        if (!std.mem.eql(u8, expected.name, actual.name)) return error.PlanStale;
        if (expected.index != actual.index) return error.PlanStale;
        const arrival_s = @abs(expected.duration_s_arrival - actual.duration_s_arrival);
        const departure_s = @abs(expected.duration_s_departure - actual.duration_s_departure);
        if (!(arrival_s <= duration_s_tolerance)) return error.PlanStale;
        if (!(departure_s <= duration_s_tolerance)) return error.PlanStale;
    }
    assert(file.len == recomputed.len);
}

const test_course =
    \\<?xml version="1.0"?>
    \\<gpx><metadata><name>Two checkpoints</name></metadata>
    \\<wpt lat="45.0000" lon="6.0000"><name>Start</name><type>Start</type></wpt>
    \\<wpt lat="45.0090" lon="6.0000"><name>Finish</name><type>Arrival</type></wpt>
    \\<trk><trkseg>
    \\<trkpt lat="45.0000" lon="6.0000"><ele>1000</ele></trkpt>
    \\<trkpt lat="45.0030" lon="6.0000"><ele>1150</ele></trkpt>
    \\<trkpt lat="45.0060" lon="6.0000"><ele>1100</ele></trkpt>
    \\<trkpt lat="45.0090" lon="6.0000"><ele>1250</ele></trkpt>
    \\</trkseg></trk></gpx>
;

/// plan.json for `test_course` with `settings`, as gpxz writes it.
fn plan_json_test(settings: *const gpxz.Settings) ![]u8 {
    var data = try gpxz.parse(testing.allocator, test_course, settings);
    defer data.deinit(testing.allocator);
    const plan = gpxz.plan_build(
        test_course,
        data.metadata.name,
        data.trace.distance_m,
        data.plan.?,
        settings,
    );
    var out: std.Io.Writer.Allocating = .init(testing.allocator);
    errdefer out.deinit();
    try gpxz.plan_write(&out.writer, &plan);
    return out.toOwnedSlice();
}

test "load: the model comes back as settings, for the course it names" {
    const settings: gpxz.Settings = .{
        .pace_base_s_per_km = 430,
        .fatigue_coefficient = 0.004,
        .life_base_stop_s = 900,
    };
    const json = try plan_json_test(&settings);
    defer testing.allocator.free(json);
    var file = try load(testing.allocator, json, test_course);
    defer file.deinit();
    try testing.expectEqual(settings.pace_base_s_per_km, file.settings.pace_base_s_per_km);
    try testing.expectEqual(settings.fatigue_coefficient, file.settings.fatigue_coefficient);
    try testing.expectEqual(settings.life_base_stop_s, file.settings.life_base_stop_s);
    try testing.expectEqual(@as(usize, 2), file.parsed.value.entries.len);
}

test "load: another course, or this one edited, is CourseMismatch" {
    const json = try plan_json_test(&.{});
    defer testing.allocator.free(json);
    const edited = test_course[0 .. test_course.len - 1];
    try testing.expectError(error.CourseMismatch, load(testing.allocator, json, edited));
    try testing.expectError(error.CourseMismatch, load(testing.allocator, json, ""));
}

test "load: gpxz's read errors pass through" {
    try testing.expectError(error.SyntaxError, load(testing.allocator, "nope", test_course));
    const v2 = "{\"schema_version\": 2}";
    try testing.expectError(error.SchemaVersionUnsupported, load(testing.allocator, v2, ""));
}

test "entries_check: gpxz's own plan passes; a changed model, name or count is stale" {
    const settings: gpxz.Settings = .{ .pace_base_s_per_km = 450 };
    const json = try plan_json_test(&settings);
    defer testing.allocator.free(json);
    var file = try load(testing.allocator, json, test_course);
    defer file.deinit();
    const entries = file.parsed.value.entries;

    var same = try gpxz.parse(testing.allocator, test_course, &file.settings);
    defer same.deinit(testing.allocator);
    try entries_check(entries, same.plan.?);

    // As if the model had changed: the same course at another pace is another plan.
    var other = try gpxz.parse(testing.allocator, test_course, &.{ .pace_base_s_per_km = 451 });
    defer other.deinit(testing.allocator);
    try testing.expectError(error.PlanStale, entries_check(entries, other.plan.?));
    try testing.expectError(error.PlanStale, entries_check(entries, same.plan.?[0..1]));

    var renamed = try testing.allocator.dupe(gpxz.PlanEntry, same.plan.?);
    defer testing.allocator.free(renamed);
    renamed[1].name = "Elsewhere";
    try testing.expectError(error.PlanStale, entries_check(entries, renamed));
}

test "entries_check: drift up to the tolerance passes, past it is stale" {
    const json = try plan_json_test(&.{});
    defer testing.allocator.free(json);
    var file = try load(testing.allocator, json, test_course);
    defer file.deinit();
    const entries = file.parsed.value.entries;
    var drifted = try testing.allocator.dupe(gpxz.PlanEntry, entries);
    defer testing.allocator.free(drifted);

    drifted[1].duration_s_arrival += duration_s_tolerance;
    drifted[1].duration_s_departure += duration_s_tolerance;
    try entries_check(entries, drifted);
    drifted[1].duration_s_departure += 0.001;
    try testing.expectError(error.PlanStale, entries_check(entries, drifted));
    drifted[1].duration_s_departure = std.math.nan(f64);
    try testing.expectError(error.PlanStale, entries_check(entries, drifted));
}
