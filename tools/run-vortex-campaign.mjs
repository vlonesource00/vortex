import { VortexSession } from '../src/vortex-session.js';
import { Track } from '../src/sim/track.js';

const arg = (name, fallback) => { const i = process.argv.indexOf(name); return i >= 0 ? Number(process.argv[i + 1]) : fallback; };
const field = Math.max(1, Math.min(8, arg('--field', 1))), laps = Math.max(1, Math.min(8, arg('--laps', 3)));
const optionalNumber = name => { const i = process.argv.indexOf(name); return i >= 0 ? Number(process.argv[i + 1]) : null; };
const session = new VortexSession(new Track('harbor-ring'), { classId: 'gt' });
session.mode = field === 1 ? 'practice' : 'race'; session.field = field; session.laps = laps; session.autopilot = true;
session.start({ freshTrack: true });
let lastLap = session.player.race.lap, lapValid = true, steps = 0;
const sampleInterval = Math.max(.05, Number(process.argv[process.argv.indexOf('--sample') + 1]) || 5);
const sampleFrom = optionalNumber('--from') ?? -Infinity;
const sampleTo = optionalNumber('--to') ?? Infinity;
let nextSample = sampleFrom > 0 ? sampleFrom : sampleInterval;
const recorded = [], samples = [];
const zero = { steer: 0, throttle: 0, brake: 0 };
const start = performance.now();
while (steps < 120 * 60 * 7 && session.phase !== 'finished') {
  const priorLap = session.player.race.lap, priorValid = session.player.race.valid;
  session.step(1 / 120, zero); steps++;
  if (session.phase === 'racing' && session.time >= nextSample) {
    const car = session.player, driver = session.drivers[0];
    if (session.time >= sampleFrom && session.time <= sampleTo) {
      const reference = driver.line.at(car.s + Math.max(8, car.speed * .42));
      const planned = driver.planner.at(car.s + Math.max(8, car.speed * .42));
      samples.push({ time: Number(session.time.toFixed(2)), s: Number(car.s.toFixed(1)), speed: Number(car.speed.toFixed(2)),
        u: Number(car.u.toFixed(2)), v: Number(car.v.toFixed(2)), yawRate: Number(car.yawRate.toFixed(3)), ay: Number(car.ay.toFixed(2)),
        target: Number(driver.targetSpeed.toFixed(2)), lateral: Number(car.lateral.toFixed(2)), steer: Number(car.controls.steer.toFixed(3)),
        referenceSpeed: Number(reference.speed.toFixed(2)), referenceOffset: Number(reference.offset.toFixed(2)),
        plannedCurvature: Number(planned.curvature.toFixed(4)), plannedSpeed: Number(planned.speed.toFixed(2)), plannedLimit: Number(planned.speedLimit.toFixed(2)),
        throttle: Number(car.controls.throttle.toFixed(2)), brake: Number(car.controls.brake.toFixed(2)), safety: driver.safety.reason, plan: driver.plan?.id });
    }
    nextSample += sampleInterval;
  }
  if (session.player.race.lap !== lastLap) {
    lastLap = session.player.race.lap;
    if (session.player.race.lastLap !== null) recorded.push({ lap: lastLap - 1, time: session.player.race.lastLap,
      valid: lapValid && priorValid, offtrack: session.player.race.offtrack, contacts: session.contacts });
    lapValid = session.player.race.valid;
  } else if (!session.player.race.valid) lapValid = false;
  if (field === 1 && recorded.filter(lap => lap.valid).length >= laps) break;
}
const valid = recorded.filter(lap => lap.valid).map(lap => lap.time).sort((a, b) => a - b);
const drivers = session.drivers.slice(0, field), safety = drivers.reduce((sum, driver) => sum + driver.telemetry.safetyInterventions, 0);
const fallbacks = drivers.reduce((sum, driver) => sum + driver.telemetry.controllerFallbacks, 0);
const output = { track: 'harbor-ring', physicsHz: 120, field, requestedFlyingLaps: laps, frames: steps,
  simulatedSeconds: session.time.toFixed(3), wallSeconds: ((performance.now() - start) / 1000).toFixed(2),
  validLapTimes: valid.map(value => Number(value.toFixed(3))), best: valid.length ? Number(valid[0].toFixed(3)) : null,
  median: valid.length ? Number(valid[Math.floor(valid.length / 2)].toFixed(3)) : null,
  recordedLaps: recorded, offtrackSeconds: Number(session.player.race.offtrack.toFixed(3)),
  contacts: session.contacts, severeContacts: session.collisionStats.severeContacts, safetyInterventions: safety,
  controllerFallbacks: fallbacks, diagnostics: session.drivers[0].diagnostics };
if (process.argv.includes('--trace')) output.samples = samples;
console.log(JSON.stringify(output, null, 2));
if (!valid.length || valid.length < Math.min(2, laps)) process.exitCode = 1;
