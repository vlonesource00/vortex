/**
 * Raw slipstream physics audit (brief sections 28-31). Controller-independent.
 *
 * Two identical GT cars. Follower behind leader at a fixed gap and lateral
 * offset, both under identical controls. Compare the follower against a
 * control car with no leader at all, and against the same follower with the
 * wake forced to zero. Nothing here changes physics; it proves whether the
 * existing wake model produces a meaningful physical gain.
 *
 *   node tools/slipstream-lab.mjs
 */
import { Track } from '../src/sim/track.js';
import { Vehicle, wakes } from '../src/sim/vehicle.js';

const track = new Track('harbor-ring');
const dt = 1 / 120;
const GAPS = [10, 20, 30, 40, 60];
const OFFSETS = [0, 0.5, 1.0, 2.0, 3.0];
const MARKS = [50, 100, 200, 400];

/**
 * Run a follower behind a leader (or alone) and record wake, drag, downforce,
 * acceleration and the distance/time marks.
 */
/**
 * Place a car rolling at speed in a sensible gear. `place()` resets the
 * gearbox to 1, which at 40 m/s puts the engine far past the torque peak and
 * makes the car decelerate under full throttle -- that artefact, not the wake
 * model, is what made the first version of this harness report near-zero gains.
 */
function placeRolling(car, s, lateral, speed) {
  car.place(track, s, lateral, speed);
  const SPEC = car.spec;
  let best = 1, bestErr = Infinity;
  for (let g = 1; g < SPEC.gears.length; g++) {
    const rpm = Math.abs(speed) / SPEC.radius * SPEC.gears[g] * SPEC.finalDrive * 60 / (2 * Math.PI);
    const err = Math.abs(rpm - 5800);
    if (err < bestErr) { bestErr = err; best = g; }
  }
  car.gear = best;
  return car;
}

function run({ gap, offset, wakeEnabled }) {
  const leader = new Vehicle(1, 'LEAD', '#888', 'gt');
  const follower = new Vehicle(0, 'FOLL', '#fff', 'gt');
  const S0 = 600, V0 = 40;
  placeRolling(leader, S0, 0, V0);
  placeRolling(follower, S0 - gap, offset, V0);

  const out = { wakeSamples: [], marks: {} };
  // Distance is INTEGRATED, not derived from the station difference: the
  // station wraps at the lap boundary and silently corrupted the marks.
  let travelled = 0, t = 0, axSum = 0, axN = 0;
  const MAXT = 30;
  for (const m of MARKS) out.marks[m] = null;

  for (let i = 0; i < MAXT / dt; i++) {
    // HOLD THE GAP FIXED. Letting the follower catch the leader measures
    // passing dynamics, not the aerodynamic effect: the tow exists for only a
    // moment and the mean wake collapses. Question A is "does the car
    // accelerate faster in the tow", which requires a steady state.
    if (wakeEnabled && gap < 9000) {
      const p = track.at(follower.s + gap, 0);
      leader.x = p.x; leader.z = p.z; leader.yaw = p.heading;
      leader.vx = p.tx * follower.speed; leader.vz = p.tz * follower.speed;
      leader.u = follower.speed; leader.speed = follower.speed;
    }
    const airflow = wakeEnabled ? wakes([leader, follower]) : [0, 0];
    leader.controls = { throttle: 1, brake: 0, steer: 0 };
    follower.controls = { throttle: 1, brake: 0, steer: 0 };
    leader.step(dt, track, airflow[0]);
    follower.step(dt, track, airflow[1]);
    t += dt;
    travelled += Math.max(0, follower.speed) * dt;

    if (i === 0) {
      out.drag = follower.aero.drag;
      out.downforce = follower.aero.downforce;
      out.wake0 = follower.aero.wake;
    }
    // Steady-state acceleration, not the first transient step.
    if (t > 0.5) { axSum += follower.ax; axN++; }
    if (i % 60 === 0) out.wakeSamples.push({ t: Number(t.toFixed(2)), wake: Number(follower.aero.wake.toFixed(3)) });

    for (const m of MARKS) {
      if (!out.marks[m] && travelled >= m) out.marks[m] = { time: Number(t.toFixed(3)), speed: Number(follower.speed.toFixed(3)) };
    }
    if (MARKS.every(m => out.marks[m])) break;
  }
  out.ax0 = axN ? axSum / axN : 0;
  out.finalSpeed = Number(follower.speed.toFixed(3));
  out.wakeMean = Number((out.wakeSamples.reduce((a, b) => a + b.wake, 0) / Math.max(1, out.wakeSamples.length)).toFixed(3));
  return out;
}

function control() {
  // A single car, no leader, full throttle. The reference for all gains.
  // Same starting station as the tests so the run is identical apart from wake.
  return run({ gap: 0, offset: 0, wakeEnabled: false, solo: true });
}

console.log('=== SLIPSTREAM PHYSICS AUDIT (controller-independent) ===');
console.log('existing model: wake = exp(-d/40) * clamp(1 - lateral/(1.4 + d*0.035), 0, 1),  2 < d < 75');
console.log('                drag * (1 - 0.24*wake)    downforce * (1 - 0.32*wake)\n');

const base = control();
console.log('CONTROL (no leader)');
console.log(`  drag=${base.drag.toFixed(2)}  downforce=${base.downforce.toFixed(2)}  ax@t0=${base.ax0.toFixed(3)}`);
console.log(`  speed@100m=${base.marks[100]?.speed}  time@100m=${base.marks[100]?.time}  time@400m=${base.marks[400]?.time}\n`);

console.log('gap  offset | wake0 wakeMean | drag_red%  df_red% |  ax0   ax gain | time@100 gain | time@400 gain | speed@200 gain');
console.log('-------------+---------------+-------------+-----------+---------------+---------------+----------------');

const results = [];
for (const gap of GAPS) {
  for (const offset of OFFSETS) {
    const on = run({ gap, offset, wakeEnabled: true });
    const off = run({ gap, offset, wakeEnabled: false });
    const dragRed = (1 - on.drag / base.drag) * 100;
    const dfRed = (1 - on.downforce / base.downforce) * 100;
    const axGain = on.ax0 - base.ax0;
    const t100gain = (base.marks[100]?.time ?? 0) - (on.marks[100]?.time ?? 0);
    const t400gain = (base.marks[400]?.time ?? 0) - (on.marks[400]?.time ?? 0);
    const v200gain = (on.marks[200]?.speed ?? 0) - (base.marks[200]?.speed ?? 0);
    results.push({ gap, offset, wake0: on.wake0, wakeMean: on.wakeMean, dragRed, dfRed, axGain, t100gain, t400gain, v200gain, on, off });

    console.log(
      String(gap).padStart(4) + String(offset).padStart(8) + ' | ' +
      (on.wake0 ?? 0).toFixed(3).padStart(5) + on.wakeMean.toFixed(3).padStart(9) + ' | ' +
      dragRed.toFixed(2).padStart(8) + dfRed.toFixed(2).padStart(7) + '  | ' +
      on.ax0.toFixed(3).padStart(6) + axGain.toFixed(3).padStart(8) + ' | ' +
      t100gain.toFixed(4).padStart(11) + ' | ' +
      t400gain.toFixed(4).padStart(11) + ' | ' +
      v200gain.toFixed(3).padStart(12),
    );
  }
}

// Question A vs Question B (section 32).
const centered = results.filter(r => r.offset === 0);
console.log('\n=== QUESTION A: does the car accelerate faster in the tow? ===');
for (const r of centered) {
  console.log(`  gap ${String(r.gap).padStart(2)} m: wake=${r.wake0.toFixed(3)}  drag -${r.dragRed.toFixed(2)}%  `
    + `ax gain ${r.axGain.toFixed(4)} m/s2  time@400 gain ${r.t400gain.toFixed(4)} s`);
}
const maxGain = Math.max(...results.map(r => r.t400gain));
const maxAx = Math.max(...results.map(r => r.axGain));
console.log(`\n  max ax gain over matrix      = ${maxAx.toFixed(4)} m/s2`);
console.log(`  max time@400 gain over matrix = ${maxGain.toFixed(4)} s`);
console.log('\n=== EXPECTED vs MEASURED (section 31) ===');
for (const gap of [10, 20, 40, 60]) {
  const r = results.find(x => x.gap === gap && x.offset === 0);
  console.log(`  gap ${String(gap).padStart(2)}: expected wake ${Math.exp(-gap / 40).toFixed(2)}, measured ${r.wake0.toFixed(3)}`
    + `  expected drag -${(Math.exp(-gap / 40) * 24).toFixed(1)}%, measured -${r.dragRed.toFixed(2)}%`);
}
