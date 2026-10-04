const std = @import("std");
const assert = std.debug.assert;
const gpxz = @import("gpxz");
const debriefz = @import("debriefz");

/// Multi-day ultra GPX files are a few MiB, and a 50-hour FIT at 1 Hz about 15 MiB.
const file_size_max = 256 * 1024 * 1024;

/// Two paths plus three flags with values fit well within this.
const arguments_max = 16;

const usage =
    \\usage: debriefz [--json] [--pace <s/km>] [--fatigue <k>] [--life-base-stop <s>]
    \\                <plan.gpx> <activity.fit>
    \\  Compares the plan gpxz builds from plan.gpx with the activity in activity.fit.
    \\  --json             print the report as JSON on stdout instead of the text summary
    \\  --pace             the plan's flat-terrain base pace in s/km (default 500 = 8:20/km)
    \\  --fatigue          the plan's fatigue coefficient (default 0.002)
    \\  --life-base-stop   the plan's stop at each LifeBase in seconds (default 3600)
    \\  The three plan options must match the ones the plan was made with.
    \\
;

const Options = struct {
    json: bool = false,
    settings: gpxz.Settings = .{},
    path_plan: []const u8,
    path_activity: []const u8,
};

pub fn main(init: std.process.Init) !void {
    const allocator = init.gpa;
    const io = init.io;

    const arguments = try init.minimal.args.toSlice(init.arena.allocator());
    const options = options_parse(arguments) orelse {
        // A usage error is the user's mistake, not a crash: exit 2 without an error trace.
        std.debug.print(usage, .{});
        std.process.exit(2);
    };

    const plan_bytes = file_read(io, allocator, options.path_plan);
    defer allocator.free(plan_bytes);
    const activity_bytes = file_read(io, allocator, options.path_activity);
    defer allocator.free(activity_bytes);

    var data = gpxz.parse(allocator, plan_bytes, &options.settings) catch |err| switch (err) {
        error.OutOfMemory => return err,
        else => |parse_error| fail(options.path_plan, parse_error),
    };
    defer data.deinit(allocator);
    var activity = debriefz.activity.parse(allocator, activity_bytes) catch |err| switch (err) {
        error.OutOfMemory => return err,
        else => |parse_error| fail(options.path_activity, parse_error),
    };
    defer activity.deinit(allocator);
    if (data.trace.points.len < 2) fail(options.path_plan, error.TraceTooShort);

    var match = try debriefz.match.match(allocator, &data.trace, activity.samples, &.{});
    defer match.deinit(allocator);
    var report = debriefz.compare.compare(
        allocator,
        &data,
        &options.settings,
        &activity,
        &match,
        &.{},
    ) catch |err| switch (err) {
        error.OutOfMemory => return err,
        error.PlanMissing => fail(options.path_plan, err),
        error.ActivityNotOnRoute => fail(options.path_activity, err),
    };
    defer report.deinit(allocator);

    var stdout_buffer: [4096]u8 = undefined;
    var stdout_writer = std.Io.File.stdout().writer(io, &stdout_buffer);
    output_write(&stdout_writer.interface, &report, options.json) catch |err| {
        // The reader closed the pipe early (`debriefz ... | head`): not a failure.
        if (stdout_writer.err) |write_err| {
            if (write_err == error.BrokenPipe) return;
        }
        return err;
    };
}

/// A bad input file is the user's input, not a bug: report it and exit 1 without a trace.
fn fail(path: []const u8, err: anyerror) noreturn {
    std.debug.print("debriefz: {s}: {t}\n", .{ path, err });
    std.process.exit(1);
}

fn file_read(io: std.Io, allocator: std.mem.Allocator, path: []const u8) []u8 {
    assert(path.len > 0);
    const limit: std.Io.Limit = .limited(file_size_max);
    const bytes = std.Io.Dir.cwd().readFileAlloc(io, path, allocator, limit) catch |err|
        fail(path, err);
    assert(bytes.len <= file_size_max);
    return bytes;
}

fn output_write(writer: *std.Io.Writer, report: *const debriefz.Report, json: bool) !void {
    assert(report.checkpoints.len >= 2);
    assert(report.sections.len + 1 == report.checkpoints.len);
    if (json) {
        const options: std.json.Stringify.Options = .{ .whitespace = .indent_2 };
        try std.json.Stringify.value(report.*, options, writer);
        try writer.writeByte('\n');
    } else {
        try summary_write(writer, report);
    }
    try writer.flush();
}

/// Returns null on any usage error: too many arguments, unknown flag, missing or unparsable
/// value, or not exactly two paths.
fn options_parse(arguments: []const [:0]const u8) ?Options {
    assert(arguments.len >= 1);
    if (arguments.len > arguments_max) return null;

    var options: Options = .{ .path_plan = "", .path_activity = "" };
    var paths: [2][]const u8 = undefined;
    var path_count: usize = 0;
    var index: usize = 1;
    while (index < arguments.len) : (index += 1) {
        const argument = arguments[index];
        if (std.mem.eql(u8, argument, "--json")) {
            options.json = true;
        } else if (std.mem.startsWith(u8, argument, "--")) {
            index += 1;
            if (index >= arguments.len) return null;
            if (!option_value_parse(&options, argument, arguments[index])) return null;
        } else {
            if (path_count == paths.len or argument.len == 0) return null;
            paths[path_count] = argument;
            path_count += 1;
        }
    }
    if (path_count != paths.len) return null;
    options.path_plan = paths[0];
    options.path_activity = paths[1];
    assert(options.settings.pace_base_s_per_km > 0);
    assert(options.settings.fatigue_coefficient >= 0);
    return options;
}

/// Sets the option `flag` names from `value`. Returns false for an unknown flag or a value
/// that doesn't parse or is out of range.
fn option_value_parse(options: *Options, flag: []const u8, value: []const u8) bool {
    assert(std.mem.startsWith(u8, flag, "--"));
    assert(!std.mem.eql(u8, flag, "--json"));
    if (std.mem.eql(u8, flag, "--pace")) {
        const pace = std.fmt.parseFloat(f64, value) catch return false;
        if (!(pace > 0) or !std.math.isFinite(pace)) return false;
        options.settings.pace_base_s_per_km = pace;
    } else if (std.mem.eql(u8, flag, "--fatigue")) {
        const fatigue = std.fmt.parseFloat(f64, value) catch return false;
        if (!(fatigue >= 0) or !std.math.isFinite(fatigue)) return false;
        options.settings.fatigue_coefficient = fatigue;
    } else if (std.mem.eql(u8, flag, "--life-base-stop")) {
        options.settings.life_base_stop_s = std.fmt.parseInt(u32, value, 10) catch return false;
    } else {
        return false;
    }
    return true;
}

const Writer = std.Io.Writer;
const Error = Writer.Error;

fn summary_write(writer: *Writer, report: *const debriefz.Report) Error!void {
    try writer.print("{s} — plan vs actual\n", .{report.name orelse "(unnamed route)"});
    try totals_write(writer, report);
    try checkpoints_write(writer, report);
    try intervals_write(writer, "sections", report.sections);
    try intervals_write(writer, "stages", report.stages);
    try climbs_write(writer, report.climbs);
    try descents_write(writer, report.descents);
    try deviations_write(writer, report.deviations);
    if (report.calibration) |*calibration| {
        try calibration_write(writer, report, calibration);
    }
}

fn totals_write(writer: *Writer, report: *const debriefz.Report) Error!void {
    const totals = &report.totals;
    const offset_s = totals.utc_offset_s orelse 0;
    try writer.writeAll("  start      ");
    if (totals.epoch_s_start_planned) |planned| {
        try clock_write(writer, planned, offset_s);
        try writer.writeAll(" planned, ");
    }
    try clock_write(writer, totals.epoch_s_start_actual, offset_s);
    try writer.writeAll(" actual");
    try utc_offset_write(writer, totals.utc_offset_s);
    try writer.writeAll("\n  finish     ");
    try duration_write(writer, totals.duration_s_planned);
    try writer.writeAll(" planned, ");
    if (totals.duration_s_actual) |actual| {
        try duration_write(writer, actual);
        try writer.writeAll(" actual (");
        try delta_write(writer, actual - totals.duration_s_planned);
        try writer.writeAll(")\n  moving     ");
        try duration_write(writer, totals.moving_s_actual.?);
        try writer.writeAll(", stopped ");
        try duration_write(writer, totals.stopped_s_actual.?);
    } else {
        try writer.print("not reached (got to km {d:.1})", .{totals.distance_m_reached / 1000});
    }
    try writer.print("\n  distance   {d:.1} km planned", .{totals.distance_m_planned / 1000});
    if (totals.distance_m_device) |device| try writer.print(", {d:.1} km device", .{device / 1000});
    try writer.print("\n  D+         {d:.0} m planned", .{totals.elevation_gain_m_planned});
    if (totals.ascent_m_device) |ascent| try writer.print(", {d:.0} m device", .{ascent});
    try writer.print("\n  off route  {d} of {d} samples, {d:.2} km\n", .{
        totals.samples_off_route,
        totals.samples,
        totals.distance_m_off_route / 1000,
    });
}

fn checkpoints_write(writer: *Writer, report: *const debriefz.Report) Error!void {
    assert(report.checkpoints.len >= 2);
    try writer.print("\ncheckpoints ({d})\n", .{report.checkpoints.len});
    try writer.writeAll("      km  checkpoint                     plan  actual   delta" ++
        "    stop plan/actual    margin plan/actual\n");
    for (report.checkpoints) |*checkpoint| {
        try writer.print("  {d:>6.1}  {s:<28} ", .{
            checkpoint.distance_m / 1000,
            name_clip(checkpoint.name, 28),
        });
        try padding_write(writer, checkpoint.name, 28);
        try duration_write_width(writer, checkpoint.duration_s_planned);
        try writer.writeAll("  ");
        try optional_duration_write(writer, checkpoint.duration_s_actual);
        try writer.writeAll("  ");
        try optional_delta_write(writer, checkpoint.delta_s);
        try writer.writeAll("   ");
        try duration_write_width(writer, checkpoint.stop_s_planned);
        try writer.writeAll(" / ");
        try optional_duration_write(writer, checkpoint.stop_s_actual);
        try writer.writeAll("   ");
        try optional_delta_write(writer, checkpoint.margin_s_planned);
        try writer.writeAll(" / ");
        try optional_delta_write(writer, checkpoint.margin_s_actual);
        try writer.writeByte('\n');
    }
}

/// Sections or stages: a `Stage` has every field of a `Section` this prints.
fn intervals_write(writer: *Writer, comptime title: []const u8, intervals: anytype) Error!void {
    comptime assert(std.mem.eql(u8, title, "sections") or std.mem.eql(u8, title, "stages"));
    try writer.print("\n" ++ title ++ " ({d}): moving time, stops excluded\n", .{intervals.len});
    try writer.writeAll("      km     D+     D-    plan  actual  ratio  stopped   HR  " ++
        title[0 .. title.len - 1] ++ "\n");
    for (intervals) |*section| {
        try writer.print("  {d:>6.1}  {d:>5.0}  {d:>5.0}  ", .{
            section.distance_m / 1000,
            section.elevation_gain_m,
            section.elevation_loss_m,
        });
        try duration_write_width(writer, section.moving_s_planned);
        try writer.writeAll("  ");
        try optional_duration_write(writer, section.moving_s_actual);
        if (section.pace_ratio) |ratio| {
            try writer.print("  {d:>5.2}", .{ratio});
        } else try writer.writeAll("      -");
        try writer.writeAll("    ");
        try optional_duration_write(writer, section.stopped_s_actual);
        if (section.heart_rate_bpm_average) |heart_rate| {
            try writer.print("  {d:>3.0}", .{heart_rate});
        } else try writer.writeAll("    -");
        try writer.print("  {s} → {s}\n", .{ section.from, section.to });
    }
}

fn climbs_write(writer: *Writer, climbs: []const debriefz.compare.Climb) Error!void {
    try writer.print("\nclimbs ({d})\n", .{climbs.len});
    try writer.writeAll("       km    len    D+   top    plan  actual   VAM plan/actual   HR\n");
    for (climbs, 1..) |*climb, number| {
        try writer.print("  {d:>2}. {d:>5.1}  {d:>4.1}k  {d:>4.0}  {d:>4.0}  ", .{
            number,
            climb.distance_m_start / 1000,
            climb.distance_m / 1000,
            climb.elevation_gain_m,
            climb.elevation_m_summit,
        });
        try duration_write_width(writer, climb.duration_s_planned);
        try writer.writeAll("  ");
        try optional_duration_write(writer, climb.duration_s_actual);
        if (climb.vam_m_per_h_planned) |value| {
            try writer.print("   {d:>5.0} / ", .{value});
        } else try writer.writeAll("       - / ");
        if (climb.vam_m_per_h_actual) |value| {
            try writer.print("{d:<5.0}", .{value});
        } else try writer.writeAll("-    ");
        if (climb.heart_rate_bpm_average) |heart_rate| {
            try writer.print("  {d:>3.0}", .{heart_rate});
        } else try writer.writeAll("    -");
        try writer.writeByte('\n');
    }
}

fn descents_write(writer: *Writer, descents: []const debriefz.compare.Descent) Error!void {
    try writer.print("\ndescents ({d})\n", .{descents.len});
    try writer.writeAll("       km    len    D-   top    plan  actual  m/h plan/actual   HR\n");
    for (descents, 1..) |*descent, number| {
        try writer.print("  {d:>2}. {d:>5.1}  {d:>4.1}k  {d:>4.0}  {d:>4.0}  ", .{
            number,
            descent.distance_m_start / 1000,
            descent.distance_m / 1000,
            descent.elevation_loss_m,
            descent.elevation_m_top,
        });
        try duration_write_width(writer, descent.duration_s_planned);
        try writer.writeAll("  ");
        try optional_duration_write(writer, descent.duration_s_actual);
        if (descent.descent_m_per_h_planned) |value| {
            try writer.print("   {d:>5.0} / ", .{value});
        } else try writer.writeAll("       - / ");
        if (descent.descent_m_per_h_actual) |value| {
            try writer.print("{d:<5.0}", .{value});
        } else try writer.writeAll("-    ");
        if (descent.heart_rate_bpm_average) |heart_rate| {
            try writer.print("  {d:>3.0}", .{heart_rate});
        } else try writer.writeAll("    -");
        try writer.writeByte('\n');
    }
}

fn deviations_write(writer: *Writer, deviations: []const debriefz.compare.Deviation) Error!void {
    if (deviations.len == 0) return;
    try writer.print("\noff the planned trace ({d})\n", .{deviations.len});
    try writer.writeAll("    left km  rejoined km      at    time   run km   max off\n");
    for (deviations) |*deviation| {
        try optional_km_write(writer, deviation.distance_m_left);
        try writer.writeAll("       ");
        try optional_km_write(writer, deviation.distance_m_rejoined);
        try writer.writeAll("  ");
        try duration_write_width(writer, @max(deviation.duration_s_left, 0));
        try writer.writeAll("  ");
        try duration_write_width(writer, deviation.duration_s);
        try writer.print("  {d:>7.2}  {d:>6.0} m\n", .{
            deviation.distance_m / 1000,
            deviation.offset_m_max,
        });
    }
}

fn optional_km_write(writer: *Writer, distance_m: ?f64) Error!void {
    const value = distance_m orelse return writer.writeAll("          -");
    try writer.print("  {d:>9.1}", .{value / 1000});
}

fn calibration_write(
    writer: *Writer,
    report: *const debriefz.Report,
    calibration: *const debriefz.calibrate.Calibration,
) Error!void {
    const settings = &report.settings;
    const replanned = calibration.duration_s_replanned;
    assert(replanned.len == report.checkpoints.len);
    try writer.print("\ncalibration (fitted on {d} sections", .{calibration.sections_used});
    if (calibration.sections_off_route > 0) {
        try writer.print(", {d} left out for running off the trace", .{
            calibration.sections_off_route,
        });
    }
    try writer.writeAll(")\n  pace       ");
    try pace_write(writer, settings.pace_base_s_per_km);
    try writer.writeAll(" → ");
    try pace_write(writer, calibration.pace_base_s_per_km);
    try writer.print("\n  fatigue    {d:.4} → {d:.4}\n  stops      LifeBase ", .{
        settings.fatigue_coefficient,
        calibration.fatigue_coefficient,
    });
    try duration_write(writer, @floatFromInt(settings.life_base_stop_s));
    try writer.writeAll(" planned, ");
    try optional_seconds_write(writer, calibration.life_base_stop_s);
    try writer.writeAll(" actual; other checkpoints ");
    try optional_seconds_write(writer, calibration.checkpoint_stop_s);
    try writer.writeAll(" actual\n  error      rms ");
    try duration_write(writer, calibration.error_s_rms_planned);
    try writer.writeAll(" → ");
    try duration_write(writer, calibration.error_s_rms_replanned);
    try writer.writeAll(", max ");
    try duration_write(writer, calibration.error_s_max_planned);
    try writer.writeAll(" → ");
    try duration_write(writer, calibration.error_s_max_replanned);
    try writer.writeAll(" (replanned finish ");
    try duration_write(writer, replanned[replanned.len - 1]);
    try writer.print(")\n  next time  gpxz --pace {d:.0} --fatigue {d:.4}", .{
        calibration.pace_base_s_per_km,
        calibration.fatigue_coefficient,
    });
    if (calibration.life_base_stop_s) |stop_s| try writer.print(" --life-base-stop {d}", .{stop_s});
    if (calibration.checkpoint_stop_s) |stop_s| {
        try writer.print(", and <stopDuration>{d}</stopDuration> on the other checkpoints", .{
            stop_s,
        });
    }
    try writer.writeByte('\n');
}

fn optional_seconds_write(writer: *Writer, seconds: ?u32) Error!void {
    const value = seconds orelse return writer.writeAll("-");
    try duration_write(writer, @floatFromInt(value));
}

/// Seconds as `HhMM`, rounded to the minute.
fn duration_write(writer: *Writer, seconds: f64) Error!void {
    assert(std.math.isFinite(seconds));
    assert(seconds >= 0);
    const minutes_total: u64 = @intFromFloat(@round(seconds / 60.0));
    try writer.print("{d}h{d:0>2}", .{ minutes_total / 60, minutes_total % 60 });
}

/// `duration_write` right-aligned in 6 columns, enough up to 99h59.
fn duration_write_width(writer: *Writer, seconds: f64) Error!void {
    var buffer: [24]u8 = undefined;
    var fixed: Writer = .fixed(&buffer);
    // A u64 of minutes is at most 18 digits of hours, plus "h" and two digits: it fits.
    duration_write(&fixed, seconds) catch unreachable;
    try writer.print("{s:>6}", .{fixed.buffered()});
}

fn optional_duration_write(writer: *Writer, seconds: ?f64) Error!void {
    if (seconds) |value| return duration_write_width(writer, value);
    try writer.writeAll("     -");
}

/// A signed duration, `+1h05` or `-0h12`: positive is later than planned.
fn delta_write(writer: *Writer, seconds: f64) Error!void {
    assert(std.math.isFinite(seconds));
    try writer.writeByte(if (seconds < 0) '-' else '+');
    try duration_write(writer, @abs(seconds));
}

fn optional_delta_write(writer: *Writer, seconds: ?f64) Error!void {
    const value = seconds orelse return writer.writeAll("     -");
    var buffer: [24]u8 = undefined;
    var fixed: Writer = .fixed(&buffer);
    // `duration_write_width`'s bound, plus the sign.
    delta_write(&fixed, value) catch unreachable;
    try writer.print("{s:>6}", .{fixed.buffered()});
}

/// Wall-clock `HH:MM` at `epoch_s` shifted by `offset_s`.
fn clock_write(writer: *Writer, epoch_s: i64, offset_s: i32) Error!void {
    const day_s = 24 * 3600;
    const seconds: u64 = @intCast(@mod(epoch_s + offset_s, day_s));
    assert(seconds < day_s);
    try writer.print("{d:0>2}:{d:0>2}", .{ seconds / 3600, seconds % 3600 / 60 });
}

fn utc_offset_write(writer: *Writer, offset_s: ?i32) Error!void {
    const value = offset_s orelse return writer.writeAll(" (UTC)");
    const magnitude: u32 = @abs(value);
    // activity.zig drops offsets beyond real zones.
    assert(magnitude <= 14 * 3600);
    try writer.print(" (UTC{c}{d}:{d:0>2})", .{
        @as(u8, if (value < 0) '-' else '+'),
        magnitude / 3600,
        magnitude % 3600 / 60,
    });
}

/// Seconds per km as `M:SS/km`.
fn pace_write(writer: *Writer, pace_s_per_km: f64) Error!void {
    assert(pace_s_per_km > 0 and std.math.isFinite(pace_s_per_km));
    const seconds: u64 = @intFromFloat(@round(pace_s_per_km));
    try writer.print("{d}:{d:0>2}/km ({d} s/km)", .{ seconds / 60, seconds % 60, seconds });
}

/// At most `columns` codepoints of `name`, so a long checkpoint name keeps the table aligned.
fn name_clip(name: []const u8, columns: u32) []const u8 {
    assert(columns > 0);
    // Not UTF-8: left whole, and `padding_write` pads it by bytes.
    var view = std.unicode.Utf8View.init(name) catch return name;
    var iterator = view.iterator();
    var count: u32 = 0;
    while (iterator.nextCodepointSlice()) |_| {
        count += 1;
        if (count == columns) return name[0..iterator.i];
    }
    assert(count < columns);
    return name;
}

/// `{s:<N}` pads by bytes; names with accents need the missing columns padded by hand.
fn padding_write(writer: *Writer, name: []const u8, columns: u32) Error!void {
    const clipped = name_clip(name, columns);
    const codepoints = std.unicode.utf8CountCodepoints(clipped) catch clipped.len;
    assert(codepoints <= clipped.len);
    const bytes_extra = clipped.len - codepoints;
    try writer.splatByteAll(' ', bytes_extra);
}

test "options_parse: two paths and plan settings" {
    const options = options_parse(&.{
        "debriefz", "--pace", "560", "--fatigue", "0.003", "plan.gpx", "run.fit",
    }).?;
    try std.testing.expectEqualStrings("plan.gpx", options.path_plan);
    try std.testing.expectEqualStrings("run.fit", options.path_activity);
    try std.testing.expectEqual(@as(f64, 560), options.settings.pace_base_s_per_km);
    try std.testing.expectEqual(@as(f64, 0.003), options.settings.fatigue_coefficient);
    try std.testing.expect(!options.json);
}

test "options_parse: --json and --life-base-stop" {
    const options = options_parse(&.{
        "debriefz", "plan.gpx", "--json", "--life-base-stop", "1800", "run.fit",
    }).?;
    try std.testing.expect(options.json);
    try std.testing.expectEqual(@as(u32, 1800), options.settings.life_base_stop_s);
    try std.testing.expectEqualStrings("run.fit", options.path_activity);
    const zero = options_parse(&.{ "debriefz", "--life-base-stop", "0", "a", "b" }).?;
    try std.testing.expectEqual(@as(u32, 0), zero.settings.life_base_stop_s);
}

test "options_parse: usage errors return null" {
    try std.testing.expect(options_parse(&.{"debriefz"}) == null);
    try std.testing.expect(options_parse(&.{ "debriefz", "a.gpx" }) == null);
    try std.testing.expect(options_parse(&.{ "debriefz", "a.gpx", "b.fit", "c" }) == null);
    try std.testing.expect(options_parse(&.{ "debriefz", "--pace", "0", "a", "b" }) == null);
    try std.testing.expect(options_parse(&.{ "debriefz", "--pace", "nan", "a", "b" }) == null);
    try std.testing.expect(options_parse(&.{ "debriefz", "--fatigue", "-1", "a", "b" }) == null);
    try std.testing.expect(options_parse(&.{ "debriefz", "--nope", "1", "a", "b" }) == null);
    try std.testing.expect(options_parse(&.{ "debriefz", "", "b" }) == null);
    const stop_negative = [_][:0]const u8{ "debriefz", "--life-base-stop", "-1", "a", "b" };
    try std.testing.expect(options_parse(&stop_negative) == null);
    try std.testing.expect(options_parse(&.{ "debriefz", "a", "b", "--pace" }) == null);
    const too_many = [_][:0]const u8{"--json"} ** arguments_max;
    try std.testing.expect(options_parse(&(.{"debriefz"} ++ too_many)) == null);
}

test "delta_write, clock_write, pace_write, utc_offset_write" {
    var out: std.Io.Writer.Allocating = .init(std.testing.allocator);
    defer out.deinit();
    try delta_write(&out.writer, -12 * 60);
    try out.writer.writeByte(' ');
    try delta_write(&out.writer, 3600 + 5 * 60);
    try out.writer.writeByte(' ');
    // 2026-08-21T03:03:47Z at UTC+2.
    try clock_write(&out.writer, 1_787_281_427, 7200);
    try out.writer.writeByte(' ');
    try pace_write(&out.writer, 573.4);
    try utc_offset_write(&out.writer, -12600);
    const expected = "-0h12 +1h05 05:03 9:33/km (573 s/km) (UTC-3:30)";
    try std.testing.expectEqualStrings(expected, out.written());
}

test "name_clip and padding_write: codepoints, not bytes" {
    try std.testing.expectEqualStrings("Lac d'Es", name_clip("Lac d'Estaing", 8));
    try std.testing.expectEqualStrings("Col é", name_clip("Col éé", 5));
    try std.testing.expectEqualStrings("short", name_clip("short", 28));
    var out: std.Io.Writer.Allocating = .init(std.testing.allocator);
    defer out.deinit();
    try padding_write(&out.writer, "Pierrefitte → X", 28);
    // "→" is 3 bytes for 1 column: 2 extra spaces.
    try std.testing.expectEqualStrings("  ", out.written());
}
