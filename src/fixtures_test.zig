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

    // Splits every kilometer to the finish, never earlier than the one before, on both clocks.
    const splits = report.splits;
    const distance_km_planned = report.totals.distance_m_planned / 1000;
    try testing.expectEqual(@as(usize, @intFromFloat(@floor(distance_km_planned))), splits.len);
    for (splits[1..], splits[0 .. splits.len - 1]) |*split, *previous| {
        try testing.expectApproxEqAbs(previous.distance_m + 1000, split.distance_m, 1e-6);
        try testing.expect(split.duration_s_planned >= previous.duration_s_planned);
        try testing.expect(split.duration_s_actual.? >= previous.duration_s_actual.?);
    }
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
}
