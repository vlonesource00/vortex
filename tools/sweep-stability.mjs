/**
 * Stability-threshold sweep at zero thermal charge.
 *
 * With the thermal term out of the way the predictive allocator reproduces the
 * protected fresh baseline exactly, so the only remaining question is how far
 * the sideslip threshold has to come in before the final stint is survivable,
 * and what that costs on a tyre that is still healthy.
 *
 *   node tools/sweep-stability.mjs
 */
import { Track } from '../src/sim/track.js';
import { VortexSession } from '../src/vortex-session.js';
import { VortexDriver } from '../src/ai/vortex/vortex-driver.js';
import { ActuatorAllocator } from '../src/ai/vortex/control/actuator-allocator.js';

const LAPS = 4;

function run(betaClear, betaSpin, spinCostSeconds) {
  const track = new Track('harbor-ring');
  const session = new VortexSession(track, { classId: 'gt' });
  session.mode = 'practice'; session.field = 1; session.laps = LAPS; session.autopilot = true;
  session.start({ freshTrack: true });
  const car = session.cars[0];
  const driver = new VortexDriver(0, session.lineFor(car), { aggression: session.aggression });
  driver.allocator = new ActuatorAllocator({
    mode: 'predictive', thermalScale: 0, betaClear, betaSpin, spinCostSeconds,
  });
  session.drivers[0] = driver;

  const zero = { steer: 0, throttle: 0, brake: 0 };
  const laps = [];
  let prevLap = car.race.lap;
  let granted = 0;
  for (let step = 0; step < 120 * 60 * 10; step++) {
    session.step(1 / 120, zero);
    if (session.phase !== 'racing') continue;
    if (car.race.lap !== prevLap) {
      if (car.race.lastLap !== null) laps.push(Number(car.race.lastLap.toFixed(3)));
      prevLap = car.race.lap;
      if (car.race.lap > LAPS) break;
    }
    const lim = driver.limiter;
    if (lim && lim.throttlePhysical !== undefined && lim.throttle > lim.throttlePhysical + 0.005) granted++;
  }
  return { laps, granted, offtrack: Number(session.player.race.offtrack.toFixed(2)) };
}

console.log('clear  spin   cost   L1        L2        L3        L4        best     granted  off');
for (const [clear, spin, cost] of [
  [0.33, 1.20, 10],
  [0.25, 1.00, 10],
  [0.18, 0.90, 10],
  [0.33, 1.20, 30],
  [0.25, 1.00, 30],
  [0.18, 0.90, 40],
  [0.12, 0.80, 60],
]) {
  const r = run(clear, spin, cost);
  console.log(
    String(clear).padStart(5) + String(spin).padStart(7) + String(cost).padStart(6) +
    r.laps.map(t => String(t).padStart(9)).join('') +
    String(Math.min(...r.laps)).padStart(9) +
    String(r.granted).padStart(9) +
    String(r.offtrack).padStart(7)
  );
}
