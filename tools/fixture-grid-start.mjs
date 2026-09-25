/**
 * Mandatory Grid-Start Fixture (Section 19).
 *
 * Reproduces the exact scenario visually observed by the user:
 * - VORTEX starts on the left (lateral = -1.75).
 * - NOVA starts on the right/inside (lateral = +1.75).
 * - Third rival behind/outside.
 *
 * Runs the first 8 seconds from standing start at 120 Hz.
 * High-resolution telemetry instruments:
 * - Q0 vs chosen trajectory
 * - Future q at +10, +20, +40, +60 m
 * - Swept envelope conflicts
 * - Steering, throttle, brake, contacts, safety interventions
 *
 * Proves whether VORTEX attempts to cross the rival's reachable space to reacquire Q0.
 */
import { Track } from '../src/sim/track.js';
import { wrap } from '../src/sim/math.js';
import { VortexSession } from '../src/vortex-session.js';
import { AdaptiveDriver as Driver } from '../src/sim/controller.js';

const track = new Track('harbor-ring');

export function runGridStartFixture({ duration = 8.0, verbose = true, vortexSlot = 0 } = {}) {
  const session = new VortexSession(track, { classId: 'gt' });
  session.mode = 'race';
  session.field = 3;
  session.laps = 1;
  session.autopilot = true;
  session.start({ freshTrack: true });
  session.phase = 'racing';
  session.countdown = 0;

  // Grid placement:
  // Slot 0: +1.75 (right/inside)
  // Slot 1: -1.75 (left/outside)
  // Slot 2: +1.75, 8m behind (right/inside)
  const vortexCar = session.cars[0];
  const rival1Car = session.cars[1];
  const rival2Car = session.cars[2];

  const gridS = track.gridS;
  if (vortexSlot === 0) {
    vortexCar.place(track, gridS, 1.75, 0);
    rival1Car.place(track, gridS, -1.75, 0);
    rival2Car.place(track, gridS - 8.0, 1.75, 0);
  } else {
    vortexCar.place(track, gridS, -1.75, 0);
    rival1Car.place(track, gridS + 2.0, 1.75, 0);
    rival2Car.place(track, gridS - 8.0, -1.75, 0);
  }

  // Set standard AI drivers for rivals
  session.drivers[1] = new Driver(1, session.lineFor(rival1Car), 0.98, 0.90);
  session.drivers[2] = new Driver(2, session.lineFor(rival2Car), 0.96, 0.85);

  const vortexDriver = session.drivers[0];
  const DT = 1 / 120;
  const steps = Math.round(duration / DT);

  const telemetryLog = [];
  let contactsWithRival1 = 0;
  let contactsWithRival2 = 0;
  let safetyCount = 0;
  let maxCrossTowardsRival = 0;
  let crossedIntoRivalLane = false;
  let lastReportT = -1;

  if (verbose) {
    console.log('=== VORTEX GRID-START FIXTURE (Section 19) ===');
    console.log(`Initial: VORTEX at s=${vortexCar.s.toFixed(1)}m, q=${vortexCar.lateral.toFixed(2)}m (LEFT)`);
    console.log(`Initial: RIVAL 1 at s=${rival1Car.s.toFixed(1)}m, q=${rival1Car.lateral.toFixed(2)}m (RIGHT/INSIDE)`);
    console.log(`Initial: RIVAL 2 at s=${rival2Car.s.toFixed(1)}m, q=${rival2Car.lateral.toFixed(2)}m (LEFT BEHIND)`);
    const q0AtStart = vortexDriver.atlas.lineOffset(vortexCar.s);
    console.log(`Track Free-Air Q0 at Start: ${q0AtStart.toFixed(2)}m\n`);
  }

  for (let step = 0; step < steps; step++) {
    const t = step * DT;
    session.step(DT, { steer: 0, throttle: 0, brake: 0 });

    if (vortexCar.impact > 0.02) {
      const d1 = Math.hypot(vortexCar.x - rival1Car.x, vortexCar.z - rival1Car.z);
      const d2 = Math.hypot(vortexCar.x - rival2Car.x, vortexCar.z - rival2Car.z);
      if (d1 < 4.8) contactsWithRival1++;
      if (d2 < 4.8) contactsWithRival2++;
    }
    if (vortexDriver.state === 'SAFETY') safetyCount++;

    const q0Now = vortexDriver.atlas.lineOffset(vortexCar.s);
    const chosenPlan = vortexDriver.plan;
    const planPoints = chosenPlan?.points || [];
    const qAt10 = chosenPlan ? vortexDriver.planner.at(vortexCar.s + 10).offset : vortexCar.lateral;
    const qAt20 = chosenPlan ? vortexDriver.planner.at(vortexCar.s + 20).offset : vortexCar.lateral;
    const qAt40 = chosenPlan ? vortexDriver.planner.at(vortexCar.s + 40).offset : vortexCar.lateral;
    const qAt60 = chosenPlan ? vortexDriver.planner.at(vortexCar.s + 60).offset : vortexCar.lateral;

    const relS1 = wrap(rival1Car.s - vortexCar.s + track.length / 2, track.length) - track.length / 2;
    // Check if VORTEX crossed across the center divider towards rival's lane DURING overlap
    const isOverlapping = relS1 >= -4.5;
    if (isOverlapping) {
      const crossed = vortexSlot === 0 ? (vortexCar.lateral < 0.2) : (vortexCar.lateral > -0.2);
      if (crossed) {
        crossedIntoRivalLane = true;
      }
      const crossShift = vortexSlot === 0 ? (1.75 - vortexCar.lateral) : (vortexCar.lateral - (-1.75));
      if (crossShift > maxCrossTowardsRival) {
        maxCrossTowardsRival = crossShift;
      }
    }

    if (t - lastReportT >= 0.25 || vortexCar.impact > 0.05) {
      lastReportT = t;
      const rec = {
        t: Number(t.toFixed(2)),
        s: Number(vortexCar.s.toFixed(1)),
        speed: Number(vortexCar.speed.toFixed(1)),
        q: Number(vortexCar.lateral.toFixed(2)),
        q0: Number(q0Now.toFixed(2)),
        planId: chosenPlan?.id || 'none',
        qAt10: Number(qAt10.toFixed(2)),
        qAt20: Number(qAt20.toFixed(2)),
        qAt40: Number(qAt40.toFixed(2)),
        steer: Number(vortexCar.controls.steer.toFixed(3)),
        throttle: Number(vortexCar.controls.throttle.toFixed(2)),
        brake: Number(vortexCar.controls.brake.toFixed(2)),
        r1_s: Number(rival1Car.s.toFixed(1)),
        r1_q: Number(rival1Car.lateral.toFixed(2)),
        r1_speed: Number(rival1Car.speed.toFixed(1)),
        gap1: Number(relS1.toFixed(1)),
        impact: Number(vortexCar.impact.toFixed(3)),
        state: vortexDriver.state
      };
      telemetryLog.push(rec);
      if (verbose) {
        console.log(`t=${rec.t.toFixed(2)}s | s=${rec.s} q=${rec.q} (Q0=${rec.q0}) | plan=${rec.planId} q@20m=${rec.qAt20} | steer=${rec.steer} thr=${rec.throttle} brk=${rec.brake} | R1: s=${rec.r1_s} q=${rec.r1_q} gap=${rec.gap1}m | state=${rec.state}${rec.impact > 0.02 ? ' IMPACT!' : ''}`);
      }
    }
  }

  const result = {
    contactsWithRival1,
    contactsWithRival2,
    safetyCount,
    maxCrossTowardsRival: Number(maxCrossTowardsRival.toFixed(2)),
    crossedIntoRivalLane,
    finalSpeed: Number(vortexCar.speed.toFixed(1)),
    finalS: Number(vortexCar.s.toFixed(1)),
    telemetryLog
  };

  if (verbose) {
    console.log('\n=== FIXTURE RESULTS ===');
    console.log(`Contacts with Rival 1 (NOVA): ${contactsWithRival1}`);
    console.log(`Contacts with Rival 2 (Astra): ${contactsWithRival2}`);
    console.log(`Safety Interventions: ${safetyCount}`);
    console.log(`Max Lateral Shift towards Rival: +${result.maxCrossTowardsRival}m`);
    console.log(`Crossed into Rival Lane: ${crossedIntoRivalLane ? 'YES (UNACCEPTABLE CONFLICT)' : 'NO (HELD OWNED CORRIDOR)'}`);
  }

  return result;
}

if (process.argv[1]?.endsWith('fixture-grid-start.mjs')) {
  console.log('--- TESTING SLOT 0 (VORTEX RIGHT/OUTSIDE, NOVA INSIDE) ---');
  const res0 = runGridStartFixture({ duration: 7.0, vortexSlot: 0, verbose: true });
  console.log('\n--- TESTING SLOT 1 (VORTEX LEFT/INSIDE, NOVA OUTSIDE) ---');
  const res1 = runGridStartFixture({ duration: 7.0, vortexSlot: 1, verbose: true });
  const allPass = res0.contactsWithRival1 === 0 && res0.contactsWithRival2 === 0 && !res0.crossedIntoRivalLane
    && res1.contactsWithRival1 === 0 && res1.contactsWithRival2 === 0 && !res1.crossedIntoRivalLane;
  console.log(`\nGRID START FIXTURE OVERALL: ${allPass ? 'ALL TESTS PASSED' : 'TEST FAILED'}`);
  process.exit(allPass ? 0 : 1);
}
