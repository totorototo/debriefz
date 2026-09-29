# debriefz

[![CI](https://github.com/totorototo/debriefz/actions/workflows/ci.yml/badge.svg)](https://github.com/totorototo/debriefz/actions/workflows/ci.yml)

Plan vs actual for trail races: a Zig library and CLI that compares the plan
[gpxz](https://github.com/totorototo/gpxz) builds from a GPX route with the activity
[fitz](https://github.com/totorototo/fitz) reads from a FIT file.

- Checkpoints: planned and actual arrival, delta, time spent there, cutoff margin.
- Sections: moving time planned vs actual (stops excluded), pace ratio, heart rate.
- Climbs: planned and actual time and VAM.
- Off-route stretches: where the runner left the planned trace, for how long and how far.
- Calibration: the base pace and fatigue coefficient that would have predicted the race,
  checked by rerunning gpxz's plan with them, and ready to paste into the next plan.

debriefz only connects the two: parsing and the pace model stay in gpxz and fitz. The library
is pure: it works on in-memory bytes and does no I/O.

Requires **Zig 0.16.0**.

## CLI

```sh
zig build run -- route.gpx activity.fit            # text report
zig build run -- --json route.gpx activity.fit     # the same, as JSON (plus splits, profile, track)
zig build run -- --pace 450 --fatigue 0.003 route.gpx activity.fit
```

The plan options are gpxz's, and must match the ones the plan was made with:

| Option | Default | Meaning |
| --- | --- | --- |
| `--json` | off | Print the report as JSON |
| `--pace <s/km>` | 500 (8:20/km) | The plan's flat-terrain base pace |
| `--fatigue <k>` | 0.002 | The plan's fatigue coefficient |
| `--life-base-stop <s>` | 3600 | The plan's stop at each LifeBase |

The GPX needs typed waypoints (`Start`, `TimeBarrier`, `LifeBase`, `Arrival`), since that's
what gpxz builds a plan from.

A bad input is rejected with an error naming the problem, never compared half-read: gpxz's
`ParseError` for the GPX, fitz's `FitError` for the FIT, then `PlanMissing` (fewer than two
typed waypoints), `NoPositionedSamples`, `TimeBackwards` or `ActivityNotOnRoute` (no sample
ever came near the route).

## How it works

**Matching.** Each GPS sample is placed on the planned trace as a distance along it. A
nearest-point search over the whole trace goes wrong on a trail race: loops start and end at
the same place, and out-and-backs run the same path twice. So the search is a window that
slides forward with the runner, from 50 m behind to 300 m ahead of their progress, growing
with the distance they moved since last on route; within it, a candidate also pays for
straying from where the device's odometer says the runner should be. A sample more than 75 m
from the trace is off route and doesn't move progress.

**Arrivals** are the first time progress reaches a checkpoint's distance, interpolated
between samples. **Time at a checkpoint** runs from the arrival to the last sample within
75 m of it, before the runner moves on: stopped time alone misses the slow walking around an
aid station. **Moving time** leaves out intervals slower than 0.2 m/s over an 11-sample window.

**Race time** counts from crossing the start, so a late start line doesn't read as slow
running. **Cutoff margins** use the wall clock, since a barrier closes at a fixed time.

**Calibration.** gpxz slows a runner by `exp(k · effort_km)`. If each section's actual time
is its planned one times `a · exp(b · effort_km)`, then a pace of `a ×` the planned one and a
fatigue coefficient of `k + b` predict the race. A weighted least squares fit of
`ln(actual / planned)` gives both. Sections run more than 5 % off the trace are left out:
their time says nothing about the pace on the planned route. The fit is then checked by
rerunning gpxz's plan with it and each checkpoint's actual stop.

## Using the library

Add it to your project:

```sh
zig fetch --save git+https://github.com/totorototo/debriefz
```

```zig
// build.zig
const debriefz = b.dependency("debriefz", .{ .target = target, .optimize = optimize });
exe.root_module.addImport("debriefz", debriefz.module("debriefz"));
```

Then parse both files, match the activity to the route and compare:

```zig
const gpxz = @import("gpxz");
const debriefz = @import("debriefz");

var data = try gpxz.parse(allocator, gpx_bytes, &settings);
defer data.deinit(allocator);
var activity = try debriefz.activity.parse(allocator, fit_bytes);
defer activity.deinit(allocator);
var match = try debriefz.match.match(allocator, &data.trace, activity.samples, &.{});
defer match.deinit(allocator);
var report = try debriefz.compare.compare(allocator, &data, &settings, &activity, &match, &.{});
defer report.deinit(allocator);
```

`settings` must be the gpxz settings the plan was made with. The `Report`'s field names are the
`--json` keys.

## Development

```sh
zig build test --summary all        # unit tests and real-file fixtures
```

CI also checks `zig fmt`, a 100-column line limit, and runs the tests in Debug and ReleaseSafe
on Linux, macOS and Windows. The code follows
[TigerBeetle's style](https://github.com/tigerbeetle/tigerbeetle/blob/main/docs/TIGER_STYLE.md):
an error means a bad file, and a failed `assert` means a bug in debriefz.

```
src/activity.zig        FIT bytes → Activity (positioned samples, session totals, UTC offset)
src/match.zig           map matching, off-route deviations, odometer, stopped intervals
src/timeline.zig        gpxz's plan at every trace point
src/actual.zig          queries on the matched activity (arrivals, stops, heart rate)
src/compare.zig         the Report: checkpoints, sections, climbs, splits, deviations
src/series.zig          the plan's profile on both clocks, and the track thinned for a map
src/calibrate.zig       fitted pace and fatigue, and gpxz's plan rerun with them
src/main.zig            the CLI
src/fixtures_test.zig   tests against the files in testdata/
```

**Dependencies** are pinned by commit and hash in `build.zig.zon`. To move to a newer one:

```sh
zig fetch --save=gpxz git+https://github.com/totorototo/gpxz#<commit>
zig build test --summary all
```

## Licenses

`testdata/Activity.fit` comes from
[python-fitparse](https://github.com/dtcooper/python-fitparse) (MIT), through fitz;
`testdata/grp-160-2026.gpx` comes from gpxz, and `testdata/grp-160-2026.fit` is the author's
own race. See `testdata/README.md`.
