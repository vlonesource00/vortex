# VORTEX Motorsport

VORTEX is a standalone Harbor Ring racing AI running on the copied canonical
Three.js host, track, and vehicle simulation. The simulator, collision model,
track geometry, renderer, and assets remain unchanged. The current driver uses
the canonical RacingLine as its seed; the offline oracle is diagnostic work,
not yet a proven time-optimal atlas.

## Run

```powershell
npm install
npm run dev
```

The standalone circuit is Harbor Ring. Manual driving remains available. Press
`P` during a race to toggle the AI demonstration and `B` to open the VORTEX
race engineer panel.

## Verify and measure

```powershell
npm test
npm run build
npm run campaign:solo
npm run campaign:race
npm run oracle:measure
npm run diagnose:pace
```

The campaign tool also accepts `--trace`, `--sample <seconds>`, `--from
<seconds>`, and `--to <seconds>` for focused telemetry. It reports physical
120 Hz session measurements; the line's estimated lap time is not a lap
result.

## Current evidence

Measured on 2026-09-25 in the frozen 120 Hz canonical simulator. Lap times are
session timing, never an oracle prediction.

- **Three consecutive valid solo flying laps: 74.908 / 74.658 / 76.817 s**, with
  zero off-track seconds, zero contacts and zero controller fallbacks across
  those laps. Best lap **74.658 s**.
- The three-lap target of P1 is met; `T_best < 73.000 s` is **not**. The gap to
  the design target is 1.66 s and to the engineering target 2.16 s.
- **Reliability is the open problem.** A fourth lap departs the track at around
  s=1200–1330 and never recovers. Grip identification shows tyre grip falling
  ~13% across a four-lap stint while the plan, until recently, did not follow
  it. Two fixes landed for this and lifted the clean-stint length from two laps
  to three, but the fourth lap still fails at every tuning point tried.
- `npm test`: 4 passing tests. `npm run build`: successful; Vite reports the
  generated JavaScript chunk is larger than 500 kB.
- `sandbox:vortex:parity` passes: 9 immutable host files byte-identical to
  `benchmark/host/astra`, and the bridge controls bit-identical to a standalone
  `VortexDriver`.
- In the four-car benchmark (24 grid orders × 3 laps against Astra, Gemini
  Supreme and NOVA) VORTEX is **second on pace** (mean best lap 76.316 s vs
  Gemini's 75.192) but **never wins**: P3 in 21 of 24 heats, 183 s off track and
  111 contacts. Racecraft is unvalidated and currently poor.

### Why the car is not yet at 73 s

Instrumented with `tools/analyze-lap-loss.mjs`:

- The oracle profile integrates to **73.03 s**; the car drives 3.3 s slower than
  its own plan (`integrationLoss`), i.e. it does not hold the profile it is
  given. Closing that is worth more than making the plan faster.
- The tyre uses only **39% of its lateral capacity on average** and stays at
  ~79% throttle while 5 m/s under target, so this is a control/plan-following
  loss, not a power limit.
- The plan demands a lateral excursion of 4–5 m through s=1200–1330 that the
  closed-loop car cannot make at 70–80% grip utilisation; the car tracks
  parallel to the plan there and departs when the tyres lose grip. A lateral-rate
  reachability cap now constrains the plan, which is what lifted the clean stint
  to three laps, at a cost of ~0.2 s.

### Not yet delivered against the spec

`TrajectoryOptimizer` is still a stub rather than the 60 Hz short-horizon
nonlinear optimizer of §16, so transient steering and yaw optimisation
(§28: 0.2–0.4 s) is untouched. `LapResidualLearner` is bounded and gated but has
not demonstrated measurable convergence (§4). The combat layers — engagement
graph, corridor ownership, opportunity field, attack contracts, defence engine
and multi-corner game — are implemented but unvalidated, and the benchmark shows
they do not yet work. Pace-loss decomposition is instrumented in
`tools/analyze-lap-loss.mjs` but not yet the per-station breakdown of §27.
