//! What a FIT activity holds for a plan-vs-actual comparison: the positioned samples (time,
//! position, device distance, altitude, heart rate) and the device's session totals. Built on
//! fitz; pure: bytes in, an owned Activity out.

const std = @import("std");
const assert = std.debug.assert;
const testing = std.testing;
const fitz = @import("fitz");

/// Field 253, the timestamp in every FIT message (protocol-level, so fitz's root doesn't
/// re-export it).
const timestamp_field_number: u8 = 253;

/// Global message numbers from the FIT profile.
const message_record: u16 = 20;
const message_session: u16 = 18;
const message_activity: u16 = 34;

/// Field definition number of `local_timestamp` in `activity`.
const activity_local_timestamp: u8 = 5;

/// UTC offsets run from -12:00 to +14:00.
const utc_offset_s_max = 14 * 3600;

/// Field definition numbers in `record`.
const record_position_lat: u8 = 0;
const record_position_long: u8 = 1;
const record_altitude: u8 = 2;
const record_heart_rate: u8 = 3;
const record_distance: u8 = 5;
const record_enhanced_altitude: u8 = 78;

/// Field definition numbers in `session`.
const session_start_time: u8 = 2;
const session_total_elapsed_time: u8 = 7;
const session_total_timer_time: u8 = 8;
const session_total_distance: u8 = 9;
const session_total_ascent: u8 = 22;
const session_total_descent: u8 = 23;

/// A month of 1 Hz recording. A FIT file is at most 4 GiB, but no activity worth comparing to
/// a plan comes close, so a larger count is a limit, not a guess.
pub const samples_max = 31 * 24 * 3600;

pub const ActivityError = error{
    /// A record's timestamp is earlier than the one before it.
    TimeBackwards,
    /// A record has a position outside [-90, 90] × [-180, 180].
    PositionInvalid,
    TooManySamples,
    /// Not one record with both a timestamp and a position.
    NoPositionedSamples,
};

pub const Sample = struct {
    epoch_s: i64,
    latitude: f64,
    longitude: f64,
    /// `enhanced_altitude`, else `altitude`.
    altitude_m: ?f64,
    /// The device's odometer: smoothed, and usually better than summing GPS steps.
    distance_m: ?f64,
    heart_rate_bpm: ?u8,
};

/// The device's own totals from the `session` message, when it has one.
pub const Session = struct {
    epoch_s_start: ?i64 = null,
    elapsed_s: ?f64 = null,
    timer_s: ?f64 = null,
    distance_m: ?f64 = null,
    ascent_m: ?f64 = null,
    descent_m: ?f64 = null,
};

pub const Activity = struct {
    /// Records with a timestamp and a position, in time order.
    samples: []Sample,
    /// Records skipped for lacking a position (GPS not fixed yet, indoor, dropouts).
    records_without_position: u32,
    session: Session,
    /// The device's local time minus UTC, from the `activity` message: how to show wall-clock
    /// times as the runner saw them. Null without one, or with an implausible one.
    utc_offset_s: ?i32,

    pub fn deinit(self: *Activity, allocator: std.mem.Allocator) void {
        allocator.free(self.samples);
        self.* = undefined;
    }

    pub fn duration_s(self: *const Activity) f64 {
        assert(self.samples.len > 0);
        const first = self.samples[0].epoch_s;
        const last = self.samples[self.samples.len - 1].epoch_s;
        assert(last >= first);
        return @floatFromInt(last - first);
    }
};

/// Parses the records and the session of a FIT activity. fitz checks every CRC first, so a
/// damaged file is an error, never a partial activity. The caller owns the result.
pub fn parse(allocator: std.mem.Allocator, bytes: []const u8) !Activity {
    var parser = try fitz.Parser.init(allocator, bytes);
    defer parser.deinit();

    var samples: std.ArrayList(Sample) = .empty;
    errdefer samples.deinit(allocator);
    var records_without_position: u32 = 0;
    var session: Session = .{};
    var utc_offset_s: ?i32 = null;
    var timestamp_last: ?i64 = null;

    while (try parser.next()) |record| {
        const data = switch (record) {
            .data => |data| data,
            .definition => continue,
        };
        if (data.global_message_number == message_session) {
            session = session_read(&data);
            continue;
        }
        if (data.global_message_number == message_activity) {
            utc_offset_s = utc_offset_read(&data);
            continue;
        }
        if (data.global_message_number != message_record) continue;
        const read = record_read(&data) orelse continue;
        if (timestamp_last) |last| {
            if (read.epoch_s < last) return ActivityError.TimeBackwards;
        }
        timestamp_last = read.epoch_s;
        const sample = try sample_from(&read) orelse {
            records_without_position += 1;
            continue;
        };
        if (samples.items.len == samples_max) return ActivityError.TooManySamples;
        try samples.append(allocator, sample);
    }
    if (samples.items.len == 0) return ActivityError.NoPositionedSamples;

    const activity: Activity = .{
        .samples = try samples.toOwnedSlice(allocator),
        .records_without_position = records_without_position,
        .session = session,
        .utc_offset_s = utc_offset_s,
    };
    assert(activity.samples.len > 0 and activity.samples.len <= samples_max);
    assert(activity.samples[0].epoch_s <= activity.samples[activity.samples.len - 1].epoch_s);
    return activity;
}

/// A record's fields as stored: every one optional, positions still in degrees unchecked.
const RecordRead = struct {
    epoch_s: i64,
    latitude: ?f64 = null,
    longitude: ?f64 = null,
    altitude_m: ?f64 = null,
    altitude_m_enhanced: ?f64 = null,
    distance_m: ?f64 = null,
    heart_rate_bpm: ?u8 = null,
};

/// Returns null for a record without an absolute timestamp: it can't be placed in time.
fn record_read(data: *const fitz.DataMessage) ?RecordRead {
    assert(data.global_message_number == message_record);
    const epoch_s = timestamp_read(data) orelse return null;
    var read: RecordRead = .{ .epoch_s = epoch_s };
    var fields = data.fields_iterator();
    while (fields.next()) |field| {
        const number = field.field_definition_number;
        const value = field.element(0) orelse continue;
        const profile = fitz.profile.data_field_profile(data, number) orelse continue;
        const scaled = profile.scaled(value) orelse continue;
        switch (number) {
            record_position_lat => read.latitude = fitz.profile.semicircles_degrees(scaled),
            record_position_long => read.longitude = fitz.profile.semicircles_degrees(scaled),
            record_altitude => read.altitude_m = scaled,
            record_enhanced_altitude => read.altitude_m_enhanced = scaled,
            record_distance => read.distance_m = scaled,
            record_heart_rate => read.heart_rate_bpm = heart_rate_from(scaled),
            else => {},
        }
    }
    assert(read.epoch_s > 0);
    return read;
}

/// Null for a record without a position: GPS had no fix. An error for a position outside the
/// globe, which no device writes on purpose.
fn sample_from(read: *const RecordRead) ActivityError!?Sample {
    const latitude = read.latitude orelse return null;
    const longitude = read.longitude orelse return null;
    if (@abs(latitude) > 90.0 or @abs(longitude) > 180.0) return ActivityError.PositionInvalid;
    const sample: Sample = .{
        .epoch_s = read.epoch_s,
        .latitude = latitude,
        .longitude = longitude,
        .altitude_m = read.altitude_m_enhanced orelse read.altitude_m,
        .distance_m = read.distance_m,
        .heart_rate_bpm = read.heart_rate_bpm,
    };
    assert(@abs(sample.latitude) <= 90.0);
    assert(@abs(sample.longitude) <= 180.0);
    return sample;
}

fn heart_rate_from(scaled: f64) ?u8 {
    // 0 is how some devices say "no contact"; above 254 is not a heart rate.
    if (!(scaled > 0 and scaled < 255)) return null;
    const heart_rate_bpm: u8 = @intFromFloat(scaled);
    assert(heart_rate_bpm > 0 and heart_rate_bpm < 255);
    return heart_rate_bpm;
}

/// The data message's field 253 as Unix seconds, or null when absent or relative to power-on.
fn timestamp_read(data: *const fitz.DataMessage) ?i64 {
    assert(data.global_message_number == message_record or
        data.global_message_number == message_activity);
    if (data.compressed_timestamp) |compressed| return date_time_epoch_s(compressed);
    var fields = data.fields_iterator();
    while (fields.next()) |field| {
        if (field.field_definition_number != timestamp_field_number) continue;
        const value = field.element(0) orelse return null;
        return switch (value) {
            .unsigned => |unsigned| date_time_epoch_s(std.math.cast(u32, unsigned) orelse
                return null),
            .signed, .float, .string, .bytes => null,
        };
    }
    return null;
}

fn date_time_epoch_s(date_time: u32) ?i64 {
    const unix_s = fitz.profile.date_time_unix_s(date_time) orelse return null;
    assert(date_time >= fitz.profile.date_time_absolute_min);
    assert(unix_s > fitz.profile.fit_epoch_unix_s);
    return @intCast(unix_s);
}

/// `local_timestamp` minus `timestamp`, rounded to the quarter hour every real zone is a
/// multiple of: the two are written a moment apart on some devices.
fn utc_offset_read(data: *const fitz.DataMessage) ?i32 {
    assert(data.global_message_number == message_activity);
    const utc_s = timestamp_read(data) orelse return null;
    var fields = data.fields_iterator();
    while (fields.next()) |field| {
        if (field.field_definition_number != activity_local_timestamp) continue;
        const value = field.element(0) orelse return null;
        const local = switch (value) {
            .unsigned => |unsigned| std.math.cast(u32, unsigned) orelse return null,
            .signed, .float, .string, .bytes => return null,
        };
        // local_date_time counts from the same epoch as date_time, in local time.
        const local_s: i64 = @intCast(fitz.profile.fit_epoch_unix_s + local);
        return utc_offset_from(local_s - utc_s);
    }
    return null;
}

fn utc_offset_from(difference_s: i64) ?i32 {
    const quarter_s = 15 * 60;
    const rounded = @divFloor(difference_s + quarter_s / 2, quarter_s) * quarter_s;
    assert(@mod(rounded, quarter_s) == 0);
    assert(@abs(rounded - difference_s) <= quarter_s / 2);
    if (@abs(rounded) > utc_offset_s_max) return null;
    return @intCast(rounded);
}

fn session_read(data: *const fitz.DataMessage) Session {
    assert(data.global_message_number == message_session);
    var session: Session = .{};
    var fields = data.fields_iterator();
    while (fields.next()) |field| {
        const number = field.field_definition_number;
        const value = field.element(0) orelse continue;
        if (number == session_start_time) {
            const date_time = switch (value) {
                .unsigned => |unsigned| std.math.cast(u32, unsigned),
                .signed, .float, .string, .bytes => null,
            };
            if (date_time) |raw| session.epoch_s_start = date_time_epoch_s(raw);
            continue;
        }
        const profile = fitz.profile.data_field_profile(data, number) orelse continue;
        const scaled = profile.scaled(value) orelse continue;
        switch (number) {
            session_total_elapsed_time => session.elapsed_s = scaled,
            session_total_timer_time => session.timer_s = scaled,
            session_total_distance => session.distance_m = scaled,
            session_total_ascent => session.ascent_m = scaled,
            session_total_descent => session.descent_m = scaled,
            else => {},
        }
    }
    // Only the start time is checked: the totals are the file's own values, and a file may
    // declare them with a signed base type, so a negative one is bad bytes, not a bug.
    if (session.epoch_s_start) |epoch_s| assert(epoch_s > fitz.profile.fit_epoch_unix_s);
    return session;
}

test "heart_rate_from: the valid range and its edges" {
    try testing.expectEqual(@as(?u8, null), heart_rate_from(0));
    try testing.expectEqual(@as(?u8, 1), heart_rate_from(1));
    try testing.expectEqual(@as(?u8, 254), heart_rate_from(254));
    try testing.expectEqual(@as(?u8, null), heart_rate_from(255));
    try testing.expectEqual(@as(?u8, null), heart_rate_from(-3));
    try testing.expectEqual(@as(?u8, null), heart_rate_from(std.math.nan(f64)));
}

test "sample_from: no position is skipped, an impossible one is an error" {
    const base: RecordRead = .{ .epoch_s = 1_700_000_000, .altitude_m = 100 };
    try testing.expectEqual(@as(?Sample, null), try sample_from(&base));

    var half = base;
    half.latitude = 42.8;
    try testing.expectEqual(@as(?Sample, null), try sample_from(&half));

    var valid = half;
    valid.longitude = 0.3;
    valid.altitude_m_enhanced = 101.5;
    const sample = (try sample_from(&valid)).?;
    // enhanced_altitude wins over altitude: it has the range and resolution.
    try testing.expectEqual(@as(?f64, 101.5), sample.altitude_m);

    var edge = valid;
    edge.latitude = -90.0;
    edge.longitude = 180.0;
    try testing.expect((try sample_from(&edge)) != null);

    var outside = valid;
    outside.latitude = 90.5;
    try testing.expectError(ActivityError.PositionInvalid, sample_from(&outside));
    outside = valid;
    outside.longitude = -180.1;
    try testing.expectError(ActivityError.PositionInvalid, sample_from(&outside));
}

test "date_time_epoch_s: absolute dates only" {
    const relative = fitz.profile.date_time_absolute_min - 1;
    try testing.expectEqual(@as(?i64, null), date_time_epoch_s(relative));
    // 2026-08-21T03:03:47Z is FIT date_time 1_156_215_827 (Unix 1_787_281_427).
    try testing.expectEqual(@as(?i64, 1_787_281_427), date_time_epoch_s(1_156_215_827));
}

test "utc_offset_from: quarter hours, within real zones" {
    try testing.expectEqual(@as(?i32, 7200), utc_offset_from(7200));
    try testing.expectEqual(@as(?i32, 7200), utc_offset_from(7203));
    try testing.expectEqual(@as(?i32, -12600), utc_offset_from(-12600));
    try testing.expectEqual(@as(?i32, 0), utc_offset_from(-4));
    try testing.expectEqual(@as(?i32, utc_offset_s_max), utc_offset_from(utc_offset_s_max));
    try testing.expectEqual(@as(?i32, null), utc_offset_from(utc_offset_s_max + 900));
}

test "parse: bytes that are not a FIT file are an error" {
    try testing.expectError(error.UnexpectedEof, parse(testing.allocator, "abc"));
}
