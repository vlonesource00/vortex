# VORTEX implementation plan

Source brief: corrected VORTEX attachment supplied 2026-09-24. This file is the
parent-owned implementation plan and acceptance record, not a runtime input.

## Baseline and ownership

- `VORTEX` began empty and had no Git repository. Its 52-file starter is an
  exact byte copy of `benchmark/host/astra` at benchmark `3471a6b2` (`v1.2`).
- `astra` is at `ce7630e8` (`v1.2`) with pre-existing uncommitted work. Do not
  edit it. `benchmark` is clean at `3471a6b2` (`v1.2`).
- The copied `src/sim/vehicle.js`, `tyre.js`, `track.js`, `harbor-ring.js`,
  `car-specs.js`, `session.js`, all `src/render/*`, and `public/assets/*` stay
  byte-identical to the benchmark host. They own physics, geometry, timing,
  collisions, graphics, and asset identity.
- VORTEX implementation owns `src/ai/vortex/**`,
  `src/vortex-session.js`, `src/vortex-debugger.js`, standalone wiring in
  `src/main.js`, `src/ui.js`, `index.html`, `package.json`, `tests/**`,
  `tools/**`, and `README.md`. The benchmark integration is parent-owned.

## Frozen contracts

1. `VortexDriver.update(car,cars,dt,context)` is called by the canonical
   `Session.step` at 120 Hz. It reads vehicle/track state and writes only
   `{steer,throttle,brake,reverse}` to `car.controls`. A deterministic
   `makeVortexObservation` adapter feeds the same driver code in standalone
   and benchmark. No simulator component changes or hidden car advantages.
2. Track station `s` wraps over `track.length`; positive lateral is the
   canonical `Track.at` normal. Vehicle footprint uses `car.spec.halfWidth`
   and `halfLength`; legal lap requires `halfCarInside` of the road boundary.
3. One optimization process chooses free-air, attack, defence, and overlap
   trajectories. Labels are telemetry only. Nearby cars alone never lower
   speed or invoke braking. Brake events use station coordinates. Overlap
   ownership has hysteresis and persists until body clearance.
4. Layer clocks are deterministic accumulators: 120 Hz actuation/safety,
   approximately 60 Hz short-horizon control search, 25-30 Hz combat planner,
   30-60 Hz opponent model, 10-20 Hz opportunity field. Keep bounded work per
   update. No random exploration in benchmark mode.
5. Historical free-air pace learning accepts only clean unconstrained
   microsectors: valid lap, no off-track, contact, overlap, traffic limit,
   recovery, or safety intervention. Combat observations remain separate.

## Ordered VORTEX edits

1. Add `src/ai/vortex/observation.js` and `vortex-driver.js`. Construct the
   component stack once per driver; expose the canonical `update` contract,
   deterministic reset, telemetry and benchmark debug fields. The copied
   `Session` remains untouched.
2. Add `src/ai/vortex/atlas/track-atlas.js`, `oracle-loader.js`,
   `lap-memory.js`, `residual-learner.js`. Seed from canonical `RacingLine`,
   represent multiple continuously deformable legal corridors, hold sampled
   station speed/brake/exit data, and bound clean-lap residual updates.
3. Add `estimation/vehicle-envelope.js`, `opponent-filter.js`,
   `opponent-profile.js`, `occupancy-predictor.js`. Estimate grip/brake/drive
   from canonical wheel/chassis telemetry with bounded updates. Track each
   opponent persistently and predict hold, cover, brake, and yield occupancy.
4. Add `interaction/engagement-graph.js`, `corridor-ownership.js`,
   `opportunity-field.js`, `attack-contract.js`. Build joint reachable
   interactions for all cars, preserve physical overlap corridors, value
   retained passes and exit speed, and apply commitment hysteresis.
5. Add `planning/corridor-generator.js`, `multicorner-planner.js`,
   `scenario-game.js`, `trajectory-refiner.js`, `brake-events.js`. Generate
   continuous station/time candidates across multiple corners; score the
   full field, legal track, opponent reactions, race progress, and exit value.
   Refine the best trajectory and create explicit corner/interaction events.
6. Add `control/dynamic-model.js`, `trajectory-optimizer.js`, `servo.js`,
   `actuator-allocator.js`, and `safety/safety-kernel.js`. Warm-start a bounded
   1.2-1.8 s coupled rollout search using canonical car parameters and live
   envelope; track selected trajectory at 120 Hz; allocate combined tyre
   demand; intervene only for imminent severe physical conflicts.
7. Add `telemetry/vortex-telemetry.js`, pace-loss microsector attribution,
   plus deterministic evidence for combat ratio, retained passes, defence,
   interventions, solve time, and controller fallback.
8. Add `src/vortex-session.js` as a thin `Session` subclass replacing only
   drivers after each canonical reset. Update `src/main.js` to default Harbor
   Ring and use this subclass. Add `src/vortex-debugger.js` to display real
   selected candidates/telemetry without editing canonical render modules.
   Rebrand `src/ui.js` and `index.html`; preserve manual driving controls.
9. Add `tools/build-vortex-oracle.mjs` and lap/combat diagnostics using the
   canonical `Vehicle.step` and `Session.step`, plus focused tests in `tests`.
   Supply npm scripts for build, unit tests, solo campaign, and race campaign.
   Document commands and evidence in `README.md`.

## Parent benchmark integration after VORTEX code review

1. Initialize VORTEX Git locally, commit the reviewed source, and pin the
   full SHA in `benchmark/benchmark/subjects.json` using a portable sibling
   Git URL. `prepare-subjects.mjs` must clone the exact commit into
   `subjects/vortex`; no direct import from the development checkout.
2. Add `benchmark/sandbox/bridges/vortex-bridge.js` and register VORTEX in
   `sandbox/bridges/index.js` and all-architecture mode. The bridge only
   calls `makeVortexObservation`, `VortexDriver`, and copies the four controls.
3. Add `benchmark/scripts/vortex-parity-smoke.mjs` comparing standalone and
   pinned bridge controls bit for bit over a non-leading 120 Hz stint. Add a
   canonical VORTEX/Astra/Gemini Supreme/NOVA campaign entry point without
   altering race physics or timing.

## Verification and acceptance

- Before any AI result, compare SHA-256 of all immutable VORTEX files with
  the benchmark host. Run standalone unit tests and production build.
- Measure three consecutive valid flying Harbor laps at 120 Hz: best below
  73.000 s, median below 73.4 s preferred, no off-track/spin/safety/fallback.
- Compare solo with non-intersecting nearby traffic; controls and time must
  stay equivalent. Exercise slow-car pass, rear threat, overlap, three-wide,
  counterattack, and clean-lap learning with deterministic scenario tests.
- Run `prepare`/`verify` for the new pinned subject, parity smoke, benchmark
  sandbox build, and a shared-host campaign. Report actual numbers and gaps;
  never claim a performance target from a model estimate alone.

Risks: the corrected design is a V0-V9 program; a copied UI debugger is
Astra-specific, the offline oracle may be compute-heavy, and a sub-73 lap is
unproved until the physical simulator records it. Preserve all pre-existing
changes in sibling repositories. Rollback is removal of VORTEX's benchmark
registration and subject pin; no physics files need reverting.
