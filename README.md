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

Measured on 2026-09-24 against the shared Harbor host:

- Three consecutive legal solo laps: **84.192 / 84.175 / 86.108 s**. There
  were no off-track seconds, contacts, safety interventions, or controller
  fallbacks. The best lap is **9.325 s slower** than the shared-host Astra
  bridge reference of 74.850 s.
- One six-car race lap: **85.875 s**, legal, with zero severe contacts and zero
  off-track time. The field recorded 95 contacts; VORTEX recorded no retained
  passes. Combat quality is not validated.
- `npm test`: 4 passing tests. `npm run build`: successful; Vite reports the
  generated JavaScript chunk is larger than 500 kB.
- SHA-256 comparison against `benchmark/host/astra` found zero mismatches in
  48 immutable simulator, renderer, asset, and showcase files.

The bounded lap learner, opponent filters, interaction graph, corridor
ownership, opportunity scoring, multi-corner planner, dynamic shooting search,
servo, and safety kernel are implemented. The current evidence does not show
that they meet the VORTEX performance or racecraft goals. In particular, the
three-lap pace target, irrelevant-traffic equivalence, slow-car passing,
defence, side-by-side, three-wide, counterattack, measurable learner
convergence, and full benchmark integration/parity remain unvalidated. The
canonical RacingLine still anchors free-air pace, and pace-loss attribution is
currently scaffolding rather than measured microsector accounting.
