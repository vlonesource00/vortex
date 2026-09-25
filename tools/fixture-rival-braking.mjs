/**
 * Rival Braking Ahead Fixture (Section 10).
 *
 * Tests whether VORTEX, when approaching a lead rival that brakes early on a straight
 * or corner entry, simply brakes behind the rival or evaluates an open flank corridor
 * (ESCAPE LEFT / RIGHT) to preserve momentum and pass.
 *
 * Scenarios:
 * 1. Straight approach (s = 500m): rival brakes early at 50 m/s -> 30 m/s with open left and right corridors.
 * 2. Corner entry (s = 1250m before hairpin): rival brakes 40m earlier than standard marker.
 */
import { Track } from '../src/sim/track.js';
import { wrap, clamp } from '../src/sim/math.js';
import { VortexSession } from '../src/vortex-session.js';

const track = new Track('harbor-ring');
const HALF_L = 2.3;

export function runRivalBrakingFixture({ scenario = 'straight', verbose = true } = {}) {
  const session = new VortexSession(track, { classId: 'gt' });
  session.mode = 'race';
  session.field = 2;
  session.laps = 1;
  session.autopilot = true;
  session.start({ freshTrack: true });
  session.phase = 'racing';
  session.countdown = 0;

  const ego = session.cars[0];
  const rival = session.cars[1];

  const startS = scenario === 'straight' ? 550 : 1220;
  const initialSpeed = scenario === 'straight' ? 48 : 42;
  const initialGap = 22; // 22m behind

  // Ego and rival both start on centerline or slight right
  ego.place(track, startS, 0.5, initialSpeed);
  rival.place(track, startS + initialGap, 0.5, initialSpeed);

  // Scripted rival driver: holds speed then hard-brakes
  class EarlyBrakingRival {
    constructor() {
      this.brakeTriggered = false;
    }
    update(car, cars, dt) {
      // Rival brakes after 1.0 second or 45 meters
      if (!this.brakeTriggered && car.s > startS + initialGap + 35) {
        this.brakeTriggered = true;
      }
      if (this.brakeTriggered) {
        car.controls = { throttle: 0, brake: 0.75, steer: 0, reverse: false };
      } else {
        car.controls = { throttle: 0.6, brake: 0, steer: 0, reverse: false };
      }
    }
  }

  session.drivers[1] = new EarlyBrakingRival();
  const driver = session.drivers[0];
  const DT = 1 / 120;
  const duration = 6.0;
  const steps = Math.round(duration / DT);

  let passed = false;
  let minSpeedDuringBrake = Infinity;
  let didEscapeFlank = false;
  let brakedBehind = false;
  let totalBrakeTicks = 0;
  let contacts = 0;
  let lastImpact = 0;

  if (verbose) {
    console.log(`=== RIVAL BRAKING FIXTURE: ${scenario.toUpperCase()} ===`);
    console.log(`Initial: Ego s=${ego.s.toFixed(1)} v=${ego.speed.toFixed(1)} | Rival s=${rival.s.toFixed(1)} v=${rival.speed.toFixed(1)} gap=${initialGap}m\n`);
  }

  for (let s = 0; s < steps; s++) {
    const t = s * DT;
    session.step(DT, { steer: 0, throttle: 0, brake: 0 });

    const ds = wrap(rival.s - ego.s + track.length / 2, track.length) - track.length / 2;
    const latGap = Math.abs(rival.lateral - ego.lateral);

    if (ego.impact > 0.02 && lastImpact <= 0.02) contacts++;
    lastImpact = ego.impact;

    if (session.drivers[1].brakeTriggered) {
      minSpeedDuringBrake = Math.min(minSpeedDuringBrake, ego.speed);
      if (ego.controls.brake > 0.25) totalBrakeTicks++;
      // Did ego diverge laterally by > 1.8m to take an escape flank?
      if (latGap > 1.8) didEscapeFlank = true;
      if (latGap < 1.0 && ego.controls.brake > 0.4 && ds > 0 && ds < 15) {
        brakedBehind = true;
      }
    }

    if (!passed && ds < -(2 * HALF_L + 0.5)) {
      passed = true;
      if (verbose) console.log(`[PASS COMPLETED] at t=${t.toFixed(2)}s | Ego speed=${ego.speed.toFixed(1)}m/s | Rival speed=${rival.speed.toFixed(1)}m/s`);
    }

    if (s % 30 === 0 && verbose) {
      console.log(`t=${t.toFixed(2)}s | ego s=${ego.s.toFixed(1)} q=${ego.lateral.toFixed(2)} v=${ego.speed.toFixed(1)} | thr=${ego.controls.throttle.toFixed(2)} brk=${ego.controls.brake.toFixed(2)} | rival s=${rival.s.toFixed(1)} q=${rival.lateral.toFixed(2)} v=${rival.speed.toFixed(1)} (brkTrig=${session.drivers[1].brakeTriggered}) | gap=${ds.toFixed(1)}m | plan=${driver.plan?.id}`);
    }
  }

  const result = {
    scenario,
    passed,
    minSpeed: Number(minSpeedDuringBrake.toFixed(1)),
    didEscapeFlank,
    brakedBehind,
    brakeFraction: Number((totalBrakeTicks / steps).toFixed(2)),
    contacts,
    finalEgoSpeed: Number(ego.speed.toFixed(1))
  };

  if (verbose) {
    console.log('\n--- Result Summary ---');
    console.log(`Passed: ${passed ? 'YES' : 'NO'}`);
    console.log(`Did Escape Flank: ${didEscapeFlank ? 'YES' : 'NO'}`);
    console.log(`Braked Directly Behind: ${brakedBehind ? 'YES' : 'NO'}`);
    console.log(`Min Speed Maintained: ${result.minSpeed} m/s`);
    console.log(`Contacts: ${contacts}`);
  }

  return result;
}

if (process.argv[1]?.endsWith('fixture-rival-braking.mjs')) {
  runRivalBrakingFixture({ scenario: 'straight', verbose: true });
}
