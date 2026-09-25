import { Track } from '../src/sim/track.js';
import { VortexSession } from '../src/vortex-session.js';
import { VortexDriver } from '../src/ai/vortex/vortex-driver.js';

const track = new Track('harbor-ring');
const session = new VortexSession(track, { classId: 'gt' });
session.mode = 'practice';
session.field = 1;
session.laps = 3;
session.autopilot = true;
session.start({ freshTrack: true });

const car = session.cars[0];
const driver = new VortexDriver(0, session.lineFor(car), { aggression: session.aggression });
session.drivers[0] = driver;

const zero = { steer: 0, throttle: 0, brake: 0 };
let step = 0;
let lap2Started = false;

while (step < 120 * 400) {
  session.step(1 / 120, zero);
  step++;
  if (session.phase !== 'racing') continue;

  if (car.race.lap === 2 && !lap2Started) {
    lap2Started = true;
    console.log('Lap 2 started at step', step, 't=', session.time);
  }

  if (car.race.lap === 2 && car.s >= 280 && car.s <= 370) {
    // Print diagnostic every 10 steps (every ~83 ms)
    if (step % 8 === 0) {
      const here = driver.planner.at(car.s);
      const plan = driver.planner.plan;
      const candidates = driver.planner.candidates || [];

      // Check which distance in speedTarget sets the limit
      let minReach = Infinity;
      let minDistance = -1;
      let minSpeedAtDist = 0;
      let minAuthAtDist = 0;

      for (const distance of [0, 2, 4, 7, 12, 20, 32, 50, 75, 110, 160]) {
        const speed = driver.planSpeed(car.s + distance);
        const auth = driver.brakeAuthority(car, car.s + distance, speed);
        const reach = distance <= 0
          ? speed
          : Math.sqrt(Math.max(0, speed * speed) + 2 * auth * Math.max(0, distance - 1.5));
        if (reach < minReach) {
          minReach = reach;
          minDistance = distance;
          minSpeedAtDist = speed;
          minAuthAtDist = auth;
        }
      }

      console.log(`\n[s=${car.s.toFixed(1)}m, v=${(car.speed*3.6).toFixed(1)}km/h, q=${car.lateral.toFixed(2)}]`);
      console.log(`  Plan ID: ${plan?.id}, points: ${plan?.points?.length}, targetSpeed: ${(driver.targetSpeed*3.6).toFixed(1)} km/h`);
      console.log(`  SpeedTarget clamp: dist=${minDistance}m, planSpeed@dist=${(minSpeedAtDist*3.6).toFixed(1)} km/h, auth=${minAuthAtDist.toFixed(2)}, reach=${(minReach*3.6).toFixed(1)} km/h`);
      console.log(`  here.curvature: ${here.curvature?.toFixed(5)}, here.offset: ${here.offset?.toFixed(2)}`);
      console.log(`  throttle: ${car.controls.throttle.toFixed(2)}, brake: ${car.controls.brake.toFixed(2)}, steer: ${car.controls.steer.toFixed(4)}`);
      console.log(`  Candidates (${candidates.length}):`);
      for (const c of candidates) {
        console.log(`    cand ${c.id}: score=${c.score?.toFixed(2)} p0.offset=${c.points?.[0]?.offset?.toFixed(2)} p0.curv=${c.points?.[0]?.curvature?.toFixed(5)} p0.spdLim=${(c.points?.[0]?.speedLimit*3.6)?.toFixed(1)} p10.spdLim=${(c.points?.[10]?.speedLimit*3.6)?.toFixed(1)}`);
      }
    }
  }

  if (car.race.lap === 2 && car.s > 380) break;
}
