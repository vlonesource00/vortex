/**
 * Thermal-charge calibration sweep.
 *
 * The thermal term is physically derived but its persistence is not: the
 * steady-state grip loss is what the tyre would pay if the demand were kept up
 * indefinitely, and how much of that a decision really commits depends on how
 * often the demand is repeated. Sweeping the multiplier locates whether any
 * persistence assumption buys both fresh pace and a survivable final stint.
 *
 *   node tools/sweep-thermal.mjs
 */
import { Track } from '../src/sim/track.js';
import { VortexSession } from '../src/vortex-session.js';
import { VortexDriver } from '../src/ai/vortex/vortex-driver.js';
import { ActuatorAllocator } from '../src/ai/vortex/control/actuator-allocator.js';

const LAPS = 4;

function run(thermalScale, useStability = true) {
  const track = new Track('harbor-ring');
  const session = new VortexSession(track, { classId: 'gt' });
  session.mode = 'practice'; session.field = 1; session.laps = LAPS; session.autopilot = true;
  session.start({ freshTrack: true });
  const car = session.cars[0];
  const driver = new VortexDriver(0, session.lineFor(car), { aggression: session.aggression });
  driver.allocator = new ActuatorAllocator({ mode: 'predictive', thermalScale, useStability });
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
  const cores = car.wheels.reduce((a, w) => a + w.tyre.core, 0) / 4;
  return { laps, granted, offtrack: Number(session.player.race.offtrack.toFixed(2)), core: Number(cores.toFixed(1)) };
}

console.log('scale  L1        L2        L3        L4        best     granted  off    core@end');
for (const s of [0, 1, 4, 16, 64, 256]) {
  const r = run(s);
  const valid = r.laps.slice();
  console.log(
    String(s).padStart(5) +
    r.laps.map(t => String(t).padStart(9)).join('') +
    String(Math.min(...valid)).padStart(9) +
    String(r.granted).padStart(9) +
    String(r.offtrack).padStart(7) +
    String(r.core).padStart(9)
  );
}
