/**
 * Allocator ablation.
 *
 * Which term actually protects the degraded-stint lap is the question this
 * answers, and it cannot be answered by looking at the winning configuration:
 * a controller that keeps lap 4 stable may be doing it with the stability term,
 * with the thermal term, or by simply being slower. So every variant is run
 * over the same four-lap stint and reported side by side.
 *
 *   A  physical        bounded map, no prediction        (53eb219 behaviour)
 *   B  cheap           full-ask map, no prediction       (historical behaviour)
 *   C  predictive      both terms live
 *   D  predictive, thermal cost disabled
 *   E  predictive, stability cost disabled
 *
 *   node tools/ab-allocator.mjs
 */
import { Track } from '../src/sim/track.js';
import { VortexSession } from '../src/vortex-session.js';
import { VortexDriver } from '../src/ai/vortex/vortex-driver.js';
import { ActuatorAllocator } from '../src/ai/vortex/control/actuator-allocator.js';

const LAPS = 4;

function run(label, allocatorOptions) {
  const track = new Track('harbor-ring');
  const session = new VortexSession(track, { classId: 'gt' });
  session.mode = 'practice';
  session.field = 1;
  session.laps = LAPS;
  session.autopilot = true;
  session.start({ freshTrack: true });

  const car = session.cars[0];
  const driver = new VortexDriver(0, session.lineFor(car), { aggression: session.aggression });
  driver.allocator = new ActuatorAllocator(allocatorOptions);
  session.drivers[0] = driver;

  const zero = { steer: 0, throttle: 0, brake: 0 };
  const laps = [];
  let prevLap = car.race.lap;
  let granted = 0, decisions = 0, slipPowerSum = 0, slipN = 0;
  let kappaSum = 0, kappaN = 0, tcN = 0, tcActive = 0, thrSum = 0, thrSat = 0, n = 0;
  const perLap = [];

  const started = Date.now();
  for (let step = 0; step < 120 * 60 * 10; step++) {
    session.step(1 / 120, zero);
    if (session.phase !== 'racing') continue;

    if (car.race.lap !== prevLap) {
      if (car.race.lastLap !== null) {
        laps.push(car.race.lastLap);
        perLap.push({
          lap: prevLap,
          time: Number(car.race.lastLap.toFixed(3)),
          core: Number((car.wheels.reduce((a, w) => a + w.tyre.core, 0) / 4).toFixed(1)),
          surface: Number((car.wheels.reduce((a, w) => a + w.tyre.surface, 0) / 4).toFixed(1)),
          pressure: Number((car.wheels.reduce((a, w) => a + w.tyre.pressure, 0) / 4).toFixed(3)),
          wear: Number((car.wheels.reduce((a, w) => a + w.tyre.wear, 0) / 4).toFixed(5)),
          slipPowerMean: Number((slipPowerSum / Math.max(1, slipN)).toFixed(0)),
          kappaMean: Number((kappaSum / Math.max(1, kappaN)).toFixed(4)),
          tcFrac: Number((tcActive / Math.max(1, tcN)).toFixed(3)),
          thrMean: Number((thrSum / Math.max(1, n)).toFixed(3)),
          thrSatFrac: Number((thrSat / Math.max(1, n)).toFixed(3)),
        });
        slipPowerSum = 0; slipN = 0; kappaSum = 0; kappaN = 0; tcN = 0; tcActive = 0;
        thrSum = 0; thrSat = 0; n = 0;
      }
      prevLap = car.race.lap;
      if (car.race.lap > LAPS) break;
    }

    const lim = driver.limiter;
    if (lim) {
      if (lim.throttlePhysical !== undefined && lim.throttle > lim.throttlePhysical + 0.005) granted++;
      if (lim.authorityDecision) decisions++;
    }
    const sp = (car.wheels[2].tyre.slipPower + car.wheels[3].tyre.slipPower) / 2;
    slipPowerSum += sp; slipN++;
    const k = Math.max(Math.abs(car.wheels[2].tyre.kappa), Math.abs(car.wheels[3].tyre.kappa));
    kappaSum += k; kappaN++;
    tcN++; if (car.tcActive) tcActive++;
    thrSum += car.controls.throttle;
    if (car.controls.throttle > 0.985) thrSat++;
    n++;
  }

  const valid = laps.filter(t => Number.isFinite(t));
  return {
    label,
    laps: laps.map(t => Number(t.toFixed(3))),
    best: valid.length ? Number(Math.min(...valid).toFixed(3)) : null,
    median: valid.length ? Number(valid.sort((a, b) => a - b)[Math.floor(valid.length / 2)].toFixed(3)) : null,
    granted,
    decisions,
    offtrack: Number(session.player.race.offtrack.toFixed(2)),
    contacts: session.contacts,
    severe: session.collisionStats.severeContacts,
    perLap,
    wallSeconds: Number(((Date.now() - started) / 1000).toFixed(1)),
  };
}

const CASES = [
  { label: 'A physical', opts: { mode: 'physical' } },
  { label: 'B cheap', opts: { mode: 'cheap' } },
  { label: 'C predictive', opts: { mode: 'predictive' } },
  { label: 'D predictive no-thermal', opts: { mode: 'predictive', useThermal: false } },
  { label: 'E predictive no-stability', opts: { mode: 'predictive', useStability: false } },
];

const results = [];
for (const c of CASES) {
  const r = run(c.label, c.opts);
  results.push(r);
  console.log(`${c.label.padEnd(28)} ${r.laps.map(t => String(t).padStart(7)).join('  ')}  best ${String(r.best).padStart(7)}  granted ${String(r.granted).padStart(6)}  off ${String(r.offtrack).padStart(5)}  ctc ${r.contacts}/${r.severe}  ${r.wallSeconds}s`);
}

console.log('\n=== PER-LAP THERMAL / SLIP STATE ===');
console.log('case                          lap    time   core  surf  press   wear  slipPwr  kappa  tc   thr  sat');
for (const r of results) {
  for (const p of r.perLap) {
    console.log(
      r.label.padEnd(28) + String(p.lap).padStart(4) + String(p.time).padStart(9) +
      String(p.core).padStart(7) + String(p.surface).padStart(6) + String(p.pressure).padStart(7) +
      String(p.wear).padStart(8) + String(p.slipPowerMean).padStart(9) + String(p.kappaMean).padStart(7) +
      String(p.tcFrac).padStart(6) + String(p.thrMean).padStart(6) + String(p.thrSatFrac).padStart(6)
    );
  }
}
