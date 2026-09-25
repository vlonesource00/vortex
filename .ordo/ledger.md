# VORTEX — project ledger

## Goal
Build VORTEX per the corrected brief (see `BUILD_PLAN.md` for frozen contracts).
Acceptance = brief §25 P0–P9. Phase: **V1/V2 free-air pace**.
Non-goals right now: combat (§5–§15), benchmark pin (parent-owned).

## Frozen constraints (do not violate)
- `src/sim/**`, `src/render/**`, `public/assets/**` are byte-identical to
  `benchmark/host/astra`. Physics, collision, tyres, aero, geometry, renderer,
  timing immutable. Never edit these.
- `VortexDriver.update(car, cars, dt, context)` at 120 Hz; writes only
  `{steer, throttle, brake, reverse}` to `car.controls`.
- Deterministic: no RNG in benchmark path.
- One optimizer for free-air + attack + defence; labels are telemetry only.
- Nearby car alone must never lower speed or invoke braking.

## BLOCKER: concurrent writer in the same folder
While I was working, a second session rewrote `VORTEX/src/ai/**` and
`VORTEX/tools/**` out from under me. Evidence: `atlas/envelope.js`,
`atlas/oracle.js`, `atlas/oracle-data.json`, `tools/calibrate-envelope.mjs`,
`tools/_probe-debug.mjs` appeared — files I never created — and
`track-atlas.js`, `vortex-driver.js`, `servo.js`, `trajectory-refiner.js`,
`brake-events.js`, `multicorner-planner.js`, `vehicle-envelope.js` were all
rewritten 00:43–00:46 with timestamps after my edits. That session is almost
certainly the GPT-6 Sol "Vortex implementation" window. Idle since 00:46:35.

Result: the tree is half-merged. `npm test` now fails 1 of 4
(`nearby non-intersecting traffic` asserts `brakeReason === 'clear'`, now
`'corner'`). Two of my edits survive in `vortex-driver.js`; my
`physical-profile.js` and `actuator-allocator.js` survive; my `oracle-loader.js`
calls `atlas.installOracle(stations)` but their `track-atlas.js` expects
`{n, q, v, kappa}` — that call is dead.

## Baseline (measured 2026-09-24, `campaign:solo`)
84.175 / 84.192 / 86.108 s. Clean: 0 off-track, 0 contacts, 0 safety.
Astra pinned reference 74.850 s. Brief P1 target < 73.000 s.

## Diagnosis (measured against canonical `Vehicle.step`) — still valid
The free-air speed target was governed by `RacingLine.speeds`, a flat
`PACE.corner = 12.5 m/s^2` at every speed, and `GlobalPace.brake() = 7.5`.
Measured physics is far above both:

| Quantity | Old model | Measured |
|---|---|---|
| straight braking 50->20 | 7.5 | 15.0 – 17.8 m/s^2 |
| cornering @ 20 m/s | 12.5 | 13.28 m/s^2 |
| cornering @ 50 m/s | 12.5 | 17.65 m/s^2 |
| accel @ 27 / 60 | – | 8.01 / 2.19 m/s^2 |

`RacingLine.estimatedTime()` says 75.27 s; `GlobalPace.lapTime` says 84.77 s;
the car drives 84.175 s. The 75 s figure is a fiction of the flat-12.5 model.

Second defect, confirmed by trace: a positive-feedback loop in free air.
|slip| > 0.12 rad cut `targetSpeed` to 68% of current speed and slammed full
brake; forward load transfer then collapsed rear grip, raising slip further.
Worst episode at s=2157–2194: `v` grew 6.3 -> 9.9 while target fell
36.4 -> 17.5 with `brake = 1` throughout. 45% of the lap ran >1.5 m/s under
its own target.

## My work (partial, mixed with the other session's)
- `tools/probe-vehicle.mjs` — measured envelope evidence.
- `atlas/physical-profile.js` — aero-aware lateral/brake/drive envelope +
  closed-loop profile. **Note: probe transient peaks are not sustainable grip;
  at muLat 1.245 the car went off (224 s off-track, 840 safety). Needs a
  margin sweep to find the legal knee.**
- `atlas/actuator-allocator.js` — acceleration-demand allocation with
  friction-ellipse reserve (replaces bang-bang speed-error).
- `vortex-driver.js` — station-based longitudinal
  `a = (v*^2 - v^2)/(2 ds)`, reactive slip/edge target cuts removed,
  `limiter` diagnostics added.
- `tools/optimize-track-atlas.mjs` — margin sweep + oracle emit (currently
  broken: `driver.atlas.lapTime` undefined because their TrackAtlas has no
  `lapTime`).

## Other session's work (do not discard — it is more ambitious)
`atlas/oracle.js` `TrackOracle`: optimises the lateral line q(s) itself by
gradient descent, not just the speed profile. `atlas/envelope.js`
`EnvelopeModel`: lookup-table tyre thermal/pressure model. `tools/
calibrate-envelope.mjs`: ground-truth probe. These supersede my
`physical-profile.js` if they measure out.

## Next action
**Decision needed from user before further edits** (concurrent-writer conflict).
After that: reconcile onto one envelope/oracle, sweep the pace margin to the
fastest legal setting, then brief §27 pace-loss decomposition.
