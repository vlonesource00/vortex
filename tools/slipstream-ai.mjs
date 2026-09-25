/**
 * Slipstream AI Utilization Diagnostic (Section 17).
 *
 * Runs VORTEX behind a lead car in two identical runs:
 * 1. wake ON (standard turbulent wake)
 * 2. wake OFF (wake forced to zero)
 *
 * Measures:
 * - wake
 * - atlas free-air profile speed
 * - planSpeed(s)
 * - speedTarget
 * - actual ego speed
 * - throttle / brake
 * - longitudinal demand
 * - closing rate
 *
 * Specifically verifies whether VORTEX's controller caps target speed to free-air atlas speed,
 * causing uncommanded coasting/braking while in the tow.
 */
import { Track } from '../src/sim/track.js';
import { wrap } from '../src/sim/math.js';
import { VortexSession } from '../src/vortex-session.js';
import { AdaptiveDriver } from '../src/sim/controller.js';

const track = new Track('harbor-ring');

export function runSlipstreamTest({ wakeEnabled = true, verbose = true } = {}) {
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

  // Start on long Harbor straight: s = 500m, speed = 40 m/s
  const startS = 500;
  const initialSpeed = 42;
  const initialGap = 25; // 25m behind

  ego.place(track, startS, 0, initialSpeed);
  rival.place(track, startS + initialGap, 0, initialSpeed);

  session.drivers[1] = new AdaptiveDriver(1, session.lineFor(rival), 0.95, 0.85);

  const driver = session.drivers[0];
  const DT = 1 / 120;
  const origStep = ego.step;
  if (!wakeEnabled) {
    ego.step = function(dt, trk, wk) {
      return origStep.call(this, dt, trk, 0);
    };
  }
  const duration = 6.0;
  const steps = Math.round(duration / DT);

  const logs = [];
  let towThrottledDown = 0;

  for (let s = 0; s < steps; s++) {
    const t = s * DT;
    if (!wakeEnabled) {
      ego.aero.wake = 0;
      rival.aero.wake = 0;
    }
    session.step(DT, { steer: 0, throttle: 0, brake: 0 });
    if (!wakeEnabled) {
      ego.aero.wake = 0;
    }

    const freeAtlasSpeed = driver.atlas.profileSpeed(ego.s);
    const plannedSpeed = driver.planSpeed(ego.s);
    const targetSpeed = driver.targetSpeed || driver.speedTarget(ego);
    const closing = ego.speed - rival.speed;
    const gap = wrap(rival.s - ego.s + track.length / 2, track.length) - track.length / 2;

    // Check if car is in tow with speed near/exceeding atlas speed, but throttles down
    if (ego.aero.wake > 0.2 && ego.speed >= freeAtlasSpeed - 0.5 && ego.controls.throttle < 0.5 && ego.controls.brake < 0.1) {
      towThrottledDown++;
    }

    if (s % 20 === 0) {
      logs.push({
        t: Number(t.toFixed(2)),
        s: Number(ego.s.toFixed(1)),
        wake: Number(ego.aero.wake.toFixed(3)),
        vEgo: Number(ego.speed.toFixed(2)),
        vRival: Number(rival.speed.toFixed(2)),
        freeAtlasSpeed: Number(freeAtlasSpeed.toFixed(2)),
        plannedSpeed: Number(plannedSpeed.toFixed(2)),
        targetSpeed: Number(targetSpeed.toFixed(2)),
        throttle: Number(ego.controls.throttle.toFixed(2)),
        brake: Number(ego.controls.brake.toFixed(2)),
        gap: Number(gap.toFixed(1)),
        closing: Number(closing.toFixed(2))
      });
    }
  }

  const finalRecord = logs[logs.length - 1];
  if (verbose) {
    console.log(`\n=== SLIPSTREAM AI TEST (wake=${wakeEnabled ? 'ON' : 'OFF'}) ===`);
    console.table(logs);
    console.log(`Final Ego Speed: ${finalRecord.vEgo} m/s | Final Gap: ${finalRecord.gap}m | Tow throttled down events: ${towThrottledDown}`);
  }

  return {
    wakeEnabled,
    finalSpeed: finalRecord.vEgo,
    finalGap: finalRecord.gap,
    towThrottledDown,
    logs
  };
}

if (process.argv[1]?.endsWith('slipstream-ai.mjs')) {
  console.log('Testing Wake OFF:');
  const off = runSlipstreamTest({ wakeEnabled: false, verbose: true });
  console.log('\nTesting Wake ON:');
  const on = runSlipstreamTest({ wakeEnabled: true, verbose: true });

  console.log('\n=== WAKE COMPARISON ===');
  console.log(`Wake OFF Final Speed: ${off.finalSpeed} m/s, Gap: ${off.finalGap}m`);
  console.log(`Wake ON  Final Speed: ${on.finalSpeed} m/s, Gap: ${on.finalGap}m`);
  console.log(`Speed Gain: +${(on.finalSpeed - off.finalSpeed).toFixed(2)} m/s, Gap Closed: ${(off.finalGap - on.finalGap).toFixed(2)} m`);
  console.log(`Tow Throttling-Down Count: ${on.towThrottledDown}`);
}
