const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const gpxz = b.dependency("gpxz", .{ .target = target, .optimize = optimize });
    const fitz = b.dependency("fitz", .{ .target = target, .optimize = optimize });

    // The module other packages import with `b.dependency("debriefz", ...).module("debriefz")`.
    const debriefz_mod = b.addModule("debriefz", .{
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "gpxz", .module = gpxz.module("gpxz") },
            .{ .name = "fitz", .module = fitz.module("fitz") },
        },
    });

    const lib = b.addLibrary(.{
        .linkage = .static,
        .name = "debriefz",
        .root_module = debriefz_mod,
    });
    b.installArtifact(lib);

    const exe = b.addExecutable(.{
        .name = "debriefz",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "debriefz", .module = debriefz_mod },
                .{ .name = "gpxz", .module = gpxz.module("gpxz") },
            },
        }),
    });
    b.installArtifact(exe);

    const run_cmd = b.addRunArtifact(exe);
    run_cmd.step.dependOn(b.getInstallStep());
    run_cmd.addPassthruArgs();
    const run_step = b.step("run", "Run the CLI: zig build run -- <plan.gpx> <activity.fit>");
    run_step.dependOn(&run_cmd.step);

    const test_step = b.step("test", "Run unit tests");
    const lib_tests = b.addTest(.{ .root_module = debriefz_mod });
    test_step.dependOn(&b.addRunArtifact(lib_tests).step);
    const exe_tests = b.addTest(.{ .root_module = exe.root_module });
    test_step.dependOn(&b.addRunArtifact(exe_tests).step);

    // Tests against real files get their own module, so the fixtures are embedded only there
    // and never in the library module. @embedFile can't reach outside src/, so each file is
    // named here. See testdata/README.md for where they come from.
    const fixtures_module = b.createModule(.{
        .root_source_file = b.path("src/fixtures_test.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "debriefz", .module = debriefz_mod },
            .{ .name = "gpxz", .module = gpxz.module("gpxz") },
        },
    });
    fixtures_module.addAnonymousImport("grp-160-2026.gpx", .{
        .root_source_file = b.path("testdata/grp-160-2026.gpx"),
    });
    fixtures_module.addAnonymousImport("Activity.fit", .{
        .root_source_file = b.path("testdata/Activity.fit"),
    });
    fixtures_module.addAnonymousImport("grp-160-2026.fit", .{
        .root_source_file = b.path("testdata/grp-160-2026.fit"),
    });
    const fixtures_tests = b.addTest(.{ .root_module = fixtures_module });
    test_step.dependOn(&b.addRunArtifact(fixtures_tests).step);
}
