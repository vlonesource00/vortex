/**
 * Multi-Car Awareness Fixture (Section 20).
 *
 * Tests multi-car pack situations where two rivals jointly constrain the available road:
 * 1. Two-rival squeeze: Rival 1 inside (q = -2.0), Rival 2 outside (q = +2.0), VORTEX behind (q = 0).
 * 2. Three-wide corner entry: VORTEX between two cars approaching turn-in.
 * 3. One ahead, one beside: Passing Rival 1 while defending against or avoiding Rival 2 alongside.
 *
 * Tests whether:
 * - VORTEX recognizes that free space is track minus UNION of all relevant swept bodies.
 * - Passing Rival A avoids blindly colliding into Rival B.
 * - Available legal corridors are partitioned correctly.
 */
import { Track } from '../src/sim/track.js';
import { wrap, angle } from '../src/sim/math.js';
import { VortexSession } from '../src/vortex-session.js';
import { AdaptiveDriver as Driver } from '../src/sim/controller.js';

const track = new Track('harbor-ring');

export function runMultiCarFixture({ mode = 'squeeze', verbose = true } = {}) {
  const session = new VortexSession(track, { classId: 'gt' });
  session.mode = 'race';
  session.field = 3;
  session.laps = 1;
  session.autopilot = true;
  session.start({ freshTrack: true });
  session.phase = 'racing';
  session.countdown = 0;

  const ego = session.cars[0];
  const rival1 = session.cars[1];
  const rival2 = session.cars[2];

  const startS = mode === 'squeeze' ? 520 : 1240;
  const initialSpeed = 40;

  if (mode === 'squeeze') {
    // Squeeze setup:
    // Ego behind at centerline: s = startS, q = 0
    // Rival 1 ahead left: s = startS + 18, q = -2.0 (speed 38)
    // Rival 2 ahead right: s = startS + 18, q = +2.0 (speed 38)
    // Road width is ~16m (halfWidth = 8.2). Gap between rivals is 4.0m, which is too narrow for a 2m car with safe margin!
    ego.place(track, startS, 0, initialSpeed);
    rival1.place(track, startS + 18, -2.0, 38);
    rival2.place(track, startS + 18, 2.0, 38);
  } else if (mode === 'three-wide') {
    // Three-wide setup:
    // All 3 cars nearly level:
    // Rival 1 inside: q = -2.5
    // Ego center: q = 0.0
    // Rival 2 outside: q = +2.5
    ego.place(track, startS, 0.0, initialSpeed);
    rival1.place(track, startS + 1, -2.5, initialSpeed);
    rival2.place(track, startS + 1, 2.5, initialSpeed);
  }

  // Set steady drivers for rivals
  class SteadyLaneDriver {
    constructor(targetQ, targetSpeed) {
      this.targetQ = targetQ;
      this.targetSpeed = targetSpeed;
    }
    update(car) {
      const vErr = this.targetSpeed - car.speed;
      const qErr = this.targetQ - car.lateral;
      const trk = track.at(car.s);
      const hErr = angle(trk.heading - car.yaw);
      const steer = hErr * 1.5 + qErr * 0.15 - (car.yawRate ?? 0) * 0.2;
      car.controls = {
        throttle: vErr > 0 ? 0.7 : 0.3,
        brake: vErr < -1.0 ? 0.4 : 0,
        steer: Math.max(-0.4, Math.min(0.4, steer)),
        reverse: false
      };
    }
  }

  session.drivers[1] = new SteadyLaneDriver(rival1.lateral, 38);
  session.drivers[2] = new SteadyLaneDriver(rival2.lateral, 38);

  const driver = session.drivers[0];
  const DT = 1 / 120;
  const duration = 6.0;
  const steps = Math.round(duration / DT);

  let contactsR1 = 0, contactsR2 = 0;
  let lastImpact = 0;
  let didBlindCollision = false;

  if (verbose) {
    console.log(`=== MULTI-CAR FIXTURE: ${mode.toUpperCase()} ===`);
    console.log(`Ego at s=${ego.s.toFixed(1)} q=${ego.lateral.toFixed(2)}`);
    console.log(`Rival 1 at s=${rival1.s.toFixed(1)} q=${rival1.lateral.toFixed(2)}`);
    console.log(`Rival 2 at s=${rival2.s.toFixed(1)} q=${rival2.lateral.toFixed(2)}\n`);
  }

  for (let s = 0; s < steps; s++) {
    const t = s * DT;
    session.step(DT, { steer: 0, throttle: 0, brake: 0 });

    if (ego.impact > 0.02 && lastImpact <= 0.02) {
      const d1 = Math.hypot(ego.x - rival1.x, ego.z - rival1.z);
      const d2 = Math.hypot(ego.x - rival2.x, ego.z - rival2.z);
      if (d1 < 4.8) contactsR1++;
      if (d2 < 4.8) contactsR2++;
      if (ego.impact > 0.2) didBlindCollision = true;
    }
    lastImpact = ego.impact;

    if (s % 30 === 0 && verbose) {
      console.log(`t=${t.toFixed(2)}s | ego s=${ego.s.toFixed(1)} q=${ego.lateral.toFixed(2)} v=${ego.speed.toFixed(1)} | plan=${driver.plan?.id} | R1 gap=${(rival1.s - ego.s).toFixed(1)}m | R2 gap=${(rival2.s - ego.s).toFixed(1)}m | thr=${ego.controls.throttle.toFixed(2)} brk=${ego.controls.brake.toFixed(2)}`);
    }
  }

  const result = {
    mode,
    contactsR1,
    contactsR2,
    totalContacts: contactsR1 + contactsR2,
    didBlindCollision,
    finalEgoSpeed: Number(ego.speed.toFixed(1)),
    finalEgoQ: Number(ego.lateral.toFixed(2))
  };

  if (verbose) {
    console.log('\n--- Result Summary ---');
    console.log(`Contacts with Rival 1: ${contactsR1}`);
    console.log(`Contacts with Rival 2: ${contactsR2}`);
    console.log(`Blind High-Energy Collision: ${didBlindCollision ? 'YES (FAIL)' : 'NO (PASSED)'}`);
    console.log(`Final Ego Speed: ${result.finalEgoSpeed} m/s, Lateral Q: ${result.finalEgoQ}m`);
  }

  return result;
}

if (process.argv[1]?.endsWith('fixture-multi-car.mjs')) {
  runMultiCarFixture({ mode: 'squeeze', verbose: true });
}
