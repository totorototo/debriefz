//! debriefz: compares a race plan (a GPX route run through gpxz's pace model) with what was
//! run (a FIT activity read by fitz). Pure: bytes and slices in, owned structs out, no I/O.

const std = @import("std");

pub const activity = @import("activity.zig");
pub const match = @import("match.zig");
pub const timeline = @import("timeline.zig");
pub const actual = @import("actual.zig");
pub const compare = @import("compare.zig");
pub const calibrate = @import("calibrate.zig");

pub const Report = compare.Report;
pub const Match = match.Match;

pub const Activity = activity.Activity;
pub const Sample = activity.Sample;

test {
    std.testing.refAllDecls(@This());
}
