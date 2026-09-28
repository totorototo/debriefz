# Test fixtures

Real files, written by tools other than debriefz. `src/fixtures_test.zig` embeds them, so
`zig build test` runs against them. Each is named in `build.zig`, because `@embedFile` can't
reach outside `src/` by path, and a dependency's `testdata/` isn't part of its package.

| File | From | What it checks |
|---|---|---|
| `grp-160-2026.gpx` | gpxz's `testdata/` (Grand Raid des Pyrénées 2026 Ultra Tour, trail-passion.net) | A synthetic runner, 10 % slower than the plan with 30-minute LifeBase stops, is measured and calibrated as such |
| `Activity.fit` | fitz's `testdata/`, from python-fitparse's test suite (MIT) | Samples, session totals and UTC offset match Garmin's FIT Python SDK |
| `grp-160-2026.fit` | The author's own GRP 2026 Ultra Tour, recorded on a Fenix 7 Pro, already public | Records, session totals and every checkpoint arrival (loops and out-and-backs included) match Garmin's FIT Python SDK |

Expected values come from outside debriefz: Garmin's FIT SDK for the activities, and the
synthetic runner's recipe for the race. For `grp-160-2026.fit`, an arrival is the first sample
within 60 m of the waypoint, searched once the runner is 500 m clear of the one before.

`grp-160-2026.fit` is a personal recording, committed because it is already public. Don't add
other activities you recorded yourself: they hold GPS tracks and health data. `.gitignore`
ignores `*.fit` and `*.gpx` everywhere except here.
