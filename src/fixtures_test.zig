//! Tests against real files: fitz's SDK example activity, and gpxz's GRP 160 route with a
//! synthetic runner whose race is known by construction. Expected values come from outside
//! debriefz: Garmin's FIT SDK for the activity, and the runner's own recipe for the race.

const std = @import("std");
const testing = std.testing;
const assert = std.debug.assert;
const gpxz = @import("gpxz");
const debriefz = @import("debriefz");

const activity_fit = @embedFile("Activity.fit");
const route_gpx = @embedFile("grp-160-2026.gpx");
const race_fit = @embedFile("grp-160-2026.fit");

test "Activity.fit: samples and session match Garmin's FIT SDK" {
    var activity = try debriefz.activity.parse(testing.allocator, activity_fit);
    defer activity.deinit(testing.allocator);

    try testing.expectEqual(@as(usize, 14), activity.samples.len);
    try testing.expectEqual(@as(u32, 0), activity.records_without_position);
    const first = activity.samples[0];
    try testing.expectEqual(@as(i64, 1_334_006_546), first.epoch_s);
    try testing.expectApproxEqAbs(@as(f64, 41.513926070183516), first.latitude, 1e-12);
    try testing.expectApproxEqAbs(@as(f64, -73.14859078265727), first.longitude, 1e-12);
    try testing.expectApproxEqAbs(@as(f64, 0.02), first.distance_m.?, 1e-9);
    try testing.expectApproxEqAbs(@as(f64, 278.2), first.altitude_m.?, 1e-9);
    try testing.expectEqual(@as(?u8, null), first.heart_rate_bpm);
    const last = activity.samples[13];
    try testing.expectEqual(@as(i64, 1_334_006_559), last.epoch_s);
    try testing.expectApproxEqAbs(@as(f64, 5.73), last.distance_m.?, 1e-9);

    const session = activity.session;
    try testing.expectEqual(@as(?i64, 1_334_006_546), session.epoch_s_start);
    try testing.expectApproxEqAbs(@as(f64, 13.749), session.elapsed_s.?, 1e-9);
    try testing.expectApproxEqAbs(@as(f64, 5.73), session.distance_m.?, 1e-9);
    // local_timestamp 702926691 against timestamp 702941091: US Eastern daylight time.
    try testing.expectEqual(@as(?i32, -4 * 3600), activity.utc_offset_s);
}

/// The synthetic runner: 10 % slower than the plan when moving, and 30 minutes at each
/// LifeBase instead of the plan's hour.
const runner_pace_ratio = 1.1;
const runner_life_base_stop_s = 1800.0;
const runner_sample_interval_s = 5;
const runner_epoch_s_start: i64 = 1_787_281_200;

const Knot = struct { epoch_s: f64, latitude: f64, longitude: f64, distance_m: f64 };

/// A knot per trace point: the runner's time there, then a second one after any stop. The
/// caller owns the result.
fn runner_knots(
    allocator: std.mem.Allocator,
    data: *const gpxz.GPXData,
    settings: *const gpxz.Settings,
) ![]Knot {
    const plan = data.plan.?;
    var timeline = try debriefz.timeline.compute(allocator, &data.trace, plan, settings);
    defer timeline.deinit(allocator);
    var knots: std.ArrayList(Knot) = .empty;
    errdefer knots.deinit(allocator);

    var stops_planned_s: f64 = 0;
    var stops_actual_s: f64 = 0;
    var checkpoint: usize = 0;
    const first = plan[0].index;
    const last = plan[plan.len - 1].index;
    for (first..last + 1) |index| {
        const moving_s = timeline.duration_s_arrival[index] - stops_planned_s;
        const point = data.trace.points[index];
        var knot: Knot = .{
            .epoch_s = @as(f64, @floatFromInt(runner_epoch_s_start)) +
                runner_pace_ratio * moving_s + stops_actual_s,
            .latitude = point[0],
            .longitude = point[1],
            .distance_m = data.trace.distances_m_cumulative[index],
        };
        try knots.append(allocator, knot);
        if (checkpoint < plan.len and plan[checkpoint].index == index) {
            stops_planned_s += plan[checkpoint].stop_s;
            if (plan[checkpoint].stop_s > 0) {
                stops_actual_s += runner_life_base_stop_s;
                knot.epoch_s += runner_life_base_stop_s;
                try knots.append(allocator, knot);
            }
            checkpoint += 1;
        }
    }
    assert(checkpoint == plan.len);
    return knots.toOwnedSlice(allocator);
}

/// Samples every `runner_sample_interval_s` along the knots. The caller owns the result.
fn runner_samples(allocator: std.mem.Allocator, knots: []const Knot) ![]debriefz.Sample {
    var samples: std.ArrayList(debriefz.Sample) = .empty;
    errdefer samples.deinit(allocator);
    assert(knots.len >= 2);
    const start = knots[0].epoch_s;
    const end = knots[knots.len - 1].epoch_s;
    var knot: usize = 0;
    var epoch_s = start;
    while (epoch_s <= end) : (epoch_s += runner_sample_interval_s) {
        while (knot + 2 < knots.len and knots[knot + 1].epoch_s < epoch_s) knot += 1;
        const before = knots[knot];
        const after = knots[knot + 1];
        assert(before.epoch_s <= epoch_s and epoch_s <= after.epoch_s);
        const span_s = after.epoch_s - before.epoch_s;
        const fraction = if (span_s > 0) (epoch_s - before.epoch_s) / span_s else 0;
        try samples.append(allocator, .{
            .epoch_s = @intFromFloat(epoch_s),
            .latitude = before.latitude + fraction * (after.latitude - before.latitude),
            .longitude = before.longitude + fraction * (after.longitude - before.longitude),
            .altitude_m = null,
            .distance_m = before.distance_m + fraction * (after.distance_m - before.distance_m),
            .heart_rate_bpm = 140,
        });
    }
    assert(samples.items.len > 0);
    return samples.toOwnedSlice(allocator);
}

test "grp-160: a runner 10 % off the plan is measured and calibrated as such" {
    const allocator = testing.allocator;
    const settings: gpxz.Settings = .{};
    var data = try gpxz.parse(allocator, route_gpx, &settings);
    defer data.deinit(allocator);
    const knots = try runner_knots(allocator, &data, &settings);
    defer allocator.free(knots);
    const samples = try runner_samples(allocator, knots);
    defer allocator.free(samples);
    var activity: debriefz.Activity = .{
        .samples = samples,
        .records_without_position = 0,
        .session = .{},
        .utc_offset_s = 7200,
    };

    var match = try debriefz.match.match(allocator, &data.trace, activity.samples, &.{});
    defer match.deinit(allocator);
    try testing.expectEqual(@as(u32, 0), match.samples_off_route);
    try testing.expectEqual(@as(usize, 0), match.deviations.len);
    var report = try debriefz.compare.compare(allocator, &data, &settings, &activity, &match, &.{});
    defer report.deinit(allocator);

    const plan = data.plan.?;
    var life_bases_behind: f64 = 0;
    for (report.checkpoints, plan) |*checkpoint, *entry| {
        const moving_s = entry.duration_s_arrival - planned_stops_before(plan, entry);
        const expected_s = runner_pace_ratio * moving_s + life_bases_behind;
        try testing.expectApproxEqAbs(expected_s, checkpoint.duration_s_actual.?, 15);
        if (entry.stop_s > 0) {
            life_bases_behind += runner_life_base_stop_s;
            // The dwell also counts the few steps out of the checkpoint's radius.
            try testing.expectApproxEqAbs(runner_life_base_stop_s, checkpoint.stop_s_actual.?, 120);
        }
    }
    try testing.expect(report.totals.finished);
    for (report.sections) |*section| {
        try testing.expectApproxEqAbs(runner_pace_ratio, section.pace_ratio.?, 0.02);
        try testing.expectEqual(@as(?f64, 140), section.heart_rate_bpm_average);
    }
    try expect_stages_runner(&report, plan);

    // Splits every kilometer to the finish, never earlier than the one before, on both clocks.
    const splits = report.splits;
    const distance_km_planned = report.totals.distance_m_planned / 1000;
    try testing.expectEqual(@as(usize, @intFromFloat(@floor(distance_km_planned))), splits.len);
    for (splits[1..], splits[0 .. splits.len - 1]) |*split, *previous| {
        try testing.expectApproxEqAbs(previous.distance_m + 1000, split.distance_m, 1e-6);
        try testing.expect(split.duration_s_planned >= previous.duration_s_planned);
        try testing.expect(split.duration_s_actual.? >= previous.duration_s_actual.?);
    }
    try expect_series_runner(&report, plan, samples.len);

    // Every climb was run, and planned: the runner's time on each is known.
    try testing.expect(report.climbs.len > 0);
    for (report.climbs) |*climb| {
        try testing.expect(climb.vam_m_per_h_planned.? > 0);
        try testing.expect(climb.duration_s_actual.? > 0);
    }

    const calibration = report.calibration.?;
    try testing.expectEqual(@as(u32, 0), calibration.sections_off_route);
    const pace_expected = settings.pace_base_s_per_km * runner_pace_ratio;
    try testing.expectApproxEqRel(pace_expected, calibration.pace_base_s_per_km, 0.01);
    const fatigue = calibration.fatigue_coefficient;
    try testing.expectApproxEqAbs(settings.fatigue_coefficient, fatigue, 1e-4);
    // Replanned with the fitted pace and the actual stops, the plan meets the race.
    const finish_s = report.totals.duration_s_actual.?;
    try testing.expect(calibration.error_s_max_replanned < 0.02 * finish_s);
    try testing.expect(calibration.error_s_rms_replanned < calibration.error_s_rms_planned);
}

/// The stages of the synthetic runner: one per LifeBase plus one, from the GPX's waypoint
/// types; each run at the recipe's pace ratio and 140 bpm; and together the sections, both
/// clocks adding up.
fn expect_stages_runner(report: *const debriefz.Report, plan: []const gpxz.PlanEntry) !void {
    assert(plan.len >= 2);
    var life_bases: usize = 0;
    for (plan) |*entry| {
        const type_name = entry.type_name orelse continue;
        life_bases += @intFromBool(std.mem.eql(u8, type_name, "LifeBase"));
    }
    try testing.expect(life_bases > 0);
    const stages = report.stages;
    try testing.expectEqual(life_bases + 1, stages.len);
    try testing.expectEqualStrings(plan[0].name, stages[0].from);
    try testing.expectEqualStrings(plan[plan.len - 1].name, stages[stages.len - 1].to);

    var section_index: u32 = 0;
    for (stages, 1..) |*stage, number| {
        try testing.expectEqual(section_index, stage.section_index_first);
        try testing.expect(stage.section_index_end > stage.section_index_first);
        // Every stage ends at a LifeBase, the last at the finish.
        const end_type = if (number == stages.len) "Arrival" else "LifeBase";
        try testing.expectEqualStrings(end_type, plan[stage.section_index_end].type_name.?);
        try testing.expectApproxEqAbs(runner_pace_ratio, stage.pace_ratio.?, 0.02);
        try testing.expectEqual(@as(?f64, 140), stage.heart_rate_bpm_average);
        var planned_s: f64 = 0;
        var actual_s: f64 = 0;
        var distance_m: f64 = 0;
        const grouped = report.sections[stage.section_index_first..stage.section_index_end];
        for (grouped) |*section| {
            planned_s += section.moving_s_planned;
            actual_s += section.moving_s_actual.?;
            distance_m += section.distance_m;
        }
        try testing.expectApproxEqAbs(planned_s, stage.moving_s_planned, 1e-6);
        try testing.expectApproxEqAbs(actual_s, stage.moving_s_actual.?, 1e-6);
        try testing.expectApproxEqAbs(distance_m, stage.distance_m, 1e-6);
        section_index = stage.section_index_end;
    }
    try testing.expectEqual(@as(u32, @intCast(report.sections.len)), section_index);
}

/// The profile and the track of the synthetic runner, from its recipe: every point reached
/// on both clocks, 140 bpm throughout, never off route, and the finish where gpxz puts it.
fn expect_series_runner(
    report: *const debriefz.Report,
    plan: []const gpxz.PlanEntry,
    samples_count: usize,
) !void {
    assert(plan.len >= 2);
    const profile = report.profile;
    const length_m = report.totals.distance_m_planned;
    const steps: usize = @intFromFloat(@floor(length_m / 100));
    try testing.expect(profile.len == steps + 1 or profile.len == steps + 2);
    try testing.expectEqual(@as(f64, 0), profile[0].distance_m);
    try testing.expectApproxEqAbs(length_m, profile[profile.len - 1].distance_m, 1e-6);
    for (profile[1..], profile[0 .. profile.len - 1]) |*point, *previous| {
        try testing.expect(point.distance_m > previous.distance_m);
        try testing.expect(point.duration_s_planned >= previous.duration_s_planned);
        try testing.expect(point.duration_s_actual.? >= previous.duration_s_actual.?);
        const heart_rate = point.heart_rate_bpm_average orelse continue;
        try testing.expectEqual(@as(f64, 140), heart_rate);
    }
    const finish = &profile[profile.len - 1];
    const last = &plan[plan.len - 1];
    try testing.expectApproxEqAbs(last.duration_s_arrival, finish.duration_s_planned, 1e-6);
    const moving_s = last.duration_s_arrival - planned_stops_before(plan, last);
    var life_bases: f64 = 0;
    for (plan) |*entry| {
        if (entry.stop_s > 0) life_bases += 1;
    }
    const expected_s = runner_pace_ratio * moving_s + life_bases * runner_life_base_stop_s;
    try testing.expectApproxEqAbs(expected_s, finish.duration_s_actual.?, 15);

    // A point every 50 to 50 + one sample's worth of meters, the runner never off route.
    const track = report.track;
    try testing.expect(track.len <= samples_count);
    const count_min: usize = @intFromFloat(@floor(length_m / 80));
    const count_max: usize = @intFromFloat(@ceil(length_m / 50) + 2);
    try testing.expect(track.len >= count_min and track.len <= count_max);
    for (track) |*point| try testing.expect(point.on_route);
    try testing.expectEqual(@as(f64, 0), track[0].duration_s);
}

/// A route whose start checkpoint lies 1 km into the trace: a descending lead-in, a 2 km
/// climb at 8 %, and a descent to the arrival. The caller owns the result.
fn lead_in_gpx(allocator: std.mem.Allocator) ![]u8 {
    var gpx: std.ArrayList(u8) = .empty;
    errdefer gpx.deinit(allocator);
    const step_degrees = 0.0005; // About 55.6 m of latitude.
    const lead_in = 18;
    const climb = 36;
    const count = lead_in + 2 * climb + 1;
    try gpx.appendSlice(allocator,
        \\<?xml version="1.0" encoding="UTF-8"?>
        \\<gpx version="1.1" xmlns="http://www.topografix.com/GPX/1/1">
        \\<metadata><name>Lead-in</name></metadata>
        \\
    );
    const waypoints = [_]struct { index: usize, name: []const u8, type_name: []const u8 }{
        .{ .index = lead_in, .name = "Start", .type_name = "Start" },
        .{ .index = count - 1, .name = "Arrival", .type_name = "Arrival" },
    };
    for (waypoints) |waypoint| {
        try gpx.print(allocator,
            \\<wpt lat="{d:.6}" lon="0.100000"><name>{s}</name><type>{s}</type></wpt>
            \\
        , .{
            45.0 + step_degrees * @as(f64, @floatFromInt(waypoint.index)),
            waypoint.name,
            waypoint.type_name,
        });
    }
    try gpx.appendSlice(allocator, "<trk><trkseg>\n");
    for (0..count) |index| {
        const elevation_m: f64 = if (index <= lead_in)
            1100 - 50 * @as(f64, @floatFromInt(index)) / lead_in
        else if (index <= lead_in + climb)
            1050 + 160 * @as(f64, @floatFromInt(index - lead_in)) / climb
        else
            1210 - 160 * @as(f64, @floatFromInt(index - lead_in - climb)) / climb;
        const latitude = 45.0 + step_degrees * @as(f64, @floatFromInt(index));
        try gpx.print(allocator,
            \\<trkpt lat="{d:.6}" lon="0.100000"><ele>{d:.1}</ele></trkpt>
            \\
        , .{ latitude, elevation_m });
    }
    try gpx.appendSlice(allocator, "</trkseg></trk></gpx>\n");
    return gpx.toOwnedSlice(allocator);
}

test "a lead-in before the start: climbs count from the start checkpoint, as gpxz's plan" {
    const allocator = testing.allocator;
    const gpx = try lead_in_gpx(allocator);
    defer allocator.free(gpx);
    const settings: gpxz.Settings = .{};
    var data = try gpxz.parse(allocator, gpx, &settings);
    defer data.deinit(allocator);
    const plan = data.plan.?;
    const origin_m = data.trace.distances_m_cumulative[plan[0].index];
    // The start checkpoint sits about 1 km into the trace, where gpxz's plan counts from.
    try testing.expectApproxEqAbs(@as(f64, 1000), origin_m, 5);
    try testing.expectEqual(@as(f64, 0), plan[0].distance_m);
    try testing.expect(data.trace.climbs.len >= 1);

    const knots = try runner_knots(allocator, &data, &settings);
    defer allocator.free(knots);
    const samples = try runner_samples(allocator, knots);
    defer allocator.free(samples);
    var activity: debriefz.Activity = .{
        .samples = samples,
        .records_without_position = 0,
        .session = .{},
        .utc_offset_s = null,
    };
    var match = try debriefz.match.match(allocator, &data.trace, activity.samples, &.{});
    defer match.deinit(allocator);
    var report = try debriefz.compare.compare(allocator, &data, &settings, &activity, &match, &.{});
    defer report.deinit(allocator);

    // Expected from gpxz alone: its climb, less its plan's origin.
    try testing.expectEqual(data.trace.climbs.len, report.climbs.len);
    for (data.trace.climbs, report.climbs) |*source, *climb| {
        const expected_m = source.distance_m_start - origin_m;
        try testing.expectApproxEqAbs(expected_m, climb.distance_m_start, 1e-6);
        try testing.expect(climb.distance_m_start < report.totals.distance_m_planned);
    }
    // And on the same axis as the profile, which starts at the start checkpoint.
    try testing.expectEqual(@as(f64, 0), report.profile[0].distance_m);
}

fn planned_stops_before(plan: []const gpxz.PlanEntry, entry: *const gpxz.PlanEntry) f64 {
    assert(plan.len > 0);
    var stops_s: f64 = 0;
    for (plan) |*other| {
        if (other.index >= entry.index) break;
        stops_s += other.stop_s;
    }
    assert(stops_s >= 0);
    return stops_s;
}

test "grp-160-2026.fit: records and session match Garmin's FIT SDK" {
    var activity = try debriefz.activity.parse(testing.allocator, race_fit);
    defer activity.deinit(testing.allocator);

    try testing.expectEqual(@as(usize, 176_625), activity.samples.len);
    try testing.expectEqual(@as(u32, 2_143), activity.records_without_position);
    const first = activity.samples[0];
    try testing.expectEqual(@as(i64, 1_787_281_427), first.epoch_s);
    try testing.expectApproxEqAbs(@as(f64, 42.82978671602905), first.latitude, 1e-12);
    try testing.expectApproxEqAbs(@as(f64, 0.3274140693247318), first.longitude, 1e-12);
    try testing.expectEqual(@as(?u8, 91), first.heart_rate_bpm);
    const last = activity.samples[activity.samples.len - 1];
    try testing.expectEqual(@as(i64, 1_787_460_298), last.epoch_s);
    try testing.expectApproxEqAbs(@as(f64, 182_906.7), last.distance_m.?, 1e-6);

    const session = activity.session;
    try testing.expectEqual(@as(?i64, 1_787_281_427), session.epoch_s_start);
    try testing.expectApproxEqAbs(@as(f64, 178_871.776), session.elapsed_s.?, 1e-6);
    try testing.expectApproxEqAbs(@as(f64, 182_906.7), session.distance_m.?, 1e-6);
    try testing.expectEqual(@as(?f64, 11_377), session.ascent_m);
    try testing.expectEqual(@as(?i32, 2 * 3600), activity.utc_offset_s);
}

/// Per checkpoint, in plan order: the first sample within 60 m of the waypoint, read with
/// Garmin's FIT SDK, searching for each one only once the runner was 500 m clear of the one
/// before (Col de Sencours and its return are the same place).
const race_arrivals_epoch_s_sdk = [_]i64{
    1_787_281_427, 1_787_292_202, 1_787_307_359, 1_787_315_302, 1_787_320_521,
    1_787_326_061, 1_787_337_591, 1_787_346_300, 1_787_369_907, 1_787_386_617,
    1_787_405_510, 1_787_420_092, 1_787_430_138, 1_787_444_790, 1_787_460_207,
};

test "grp-160-2026.fit: arrivals agree with the SDK's, loops and out-and-backs included" {
    const allocator = testing.allocator;
    const settings: gpxz.Settings = .{};
    var data = try gpxz.parse(allocator, route_gpx, &settings);
    defer data.deinit(allocator);
    var activity = try debriefz.activity.parse(allocator, race_fit);
    defer activity.deinit(allocator);
    var match = try debriefz.match.match(allocator, &data.trace, activity.samples, &.{});
    defer match.deinit(allocator);
    var report = try debriefz.compare.compare(allocator, &data, &settings, &activity, &match, &.{});
    defer report.deinit(allocator);

    try testing.expectEqual(race_arrivals_epoch_s_sdk.len, report.checkpoints.len);
    try testing.expect(report.totals.finished);
    for (report.checkpoints, race_arrivals_epoch_s_sdk) |*checkpoint, epoch_s_sdk| {
        const difference_s = checkpoint.epoch_s_arrival_actual.? - epoch_s_sdk;
        // debriefz waits for the point on the trace, the SDK side for the 60 m radius, so it
        // is never earlier. Hautacam's waypoint sits off the trail: up to 7 minutes apart.
        const tolerance_s: i64 = if (std.mem.eql(u8, checkpoint.name, "Hautacam")) 600 else 180;
        try testing.expect(difference_s >= 0 and difference_s <= tolerance_s);
    }

    // The track keeps both ends of every stretch off route, so each deviation shows on it.
    var stretches_off_route: usize = 0;
    for (report.track[1..], report.track[0 .. report.track.len - 1]) |*point, *previous| {
        if (previous.on_route and !point.on_route) stretches_off_route += 1;
    }
    try testing.expect(stretches_off_route >= report.deviations.len);
    try testing.expect(report.track.len < activity.samples.len / 10);
}
