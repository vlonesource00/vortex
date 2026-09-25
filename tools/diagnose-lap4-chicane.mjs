import { Track } from '../src/sim/track.js';
import { VortexSession } from '../src/vortex-session.js';
import { VortexDriver } from '../src/ai/vortex/vortex-driver.js';

const track = new Track('harbor-ring');
const session = new VortexSession(track, { classId: 'gt' });
session.mode = 'practice'; session.field = 1; session.laps = 4; session.autopilot = true;
session.start({ freshTrack: true });
const car = session.cars[0];
const driver = new VortexDriver(0, session.lineFor(car), { aggression: session.aggression });
session.drivers[0] = driver;

const zero = { steer: 0, throttle: 0, brake: 0 };
let step = 0;
let prevLap = car.race.lap;

while (step < 120 * 400) {
  session.step(1 / 120, zero); step++;
  if (car.race.lap !== prevLap) {
    console.log(`Lap ${prevLap} time: ${car.race.lastLap.toFixed(3)}s`);
    prevLap = car.race.lap;
    if (car.race.lap > 4) break;
  }

  if (car.race.lap === 4 && car.s >= 2580 && car.s <= 2650) {
    const here = driver.planner.at(car.s);
    const env = driver.envelope.at(car, car.speed, here.curvature, car.lateral);
    console.log(`s=${car.s.toFixed(1)}m | v=${(car.speed*3.6).toFixed(1)}km/h (tgt=${(driver.targetSpeed*3.6).toFixed(1)}, prof=${(driver.atlas.profileSpeed(car.s)*3.6).toFixed(1)}) | q=${car.lateral.toFixed(2)} (tgtQ=${here.offset?.toFixed(2)}) | brk=${car.controls.brake.toFixed(2)} thr=${car.controls.throttle.toFixed(2)} str=${car.controls.steer.toFixed(3)} | muScale=${driver.envelope.muScale.toFixed(3)} state=${driver.state}`);
  }
}
