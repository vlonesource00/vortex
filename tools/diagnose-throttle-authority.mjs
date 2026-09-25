/**
 * Per-lap tyre state and throttle-authority diagnostic.
 *
 * Separates the fresh-tyre throttle tax from the degraded-tyre throttle need:
 * the same throttle mapping cannot be optimal on lap 1 and lap 4, so the
 * conditioning variable has to be measured before it is written.
 *
 *   node tools/diagnose-throttle-authority.mjs
 *   node tools/diagnose-throttle-authority.mjs --laps 4
 */
import { Track } from '../src/sim/track.js';
import { VortexSession } from '../src/vortex-session.js';
import { VortexDriver } from '../src/ai/vortex/vortex-driver.js';

const argv = process.argv.slice(2);
const number = (name, fallback) => {
  const i = argv.indexOf(name);
  return i >= 0 && Number.isFinite(Number(argv[i + 1])) ? Number(argv[i + 1]) : fallback;
};
const laps = number('--laps', 4);

const track = new Track('harbor-ring');
const session = new VortexSession(track, { classId: 'gt' });
session.mode = 'practice';
session.field = 1;
session.laps = laps;
session.autopilot = true;
session.start({ freshTrack: true });

const car = session.cars[0];
const driver = new VortexDriver(0, session.lineFor(car), { aggression: session.aggression });
session.drivers[0] = driver;

const zero = { steer: 0, throttle: 0, brake: 0 };
let step = 0;
let prevLap = car.race.lap;
const perLap = new Map();
const note = (key, value) => {
  const lap = car.race.lap;
  if (!perLap.has(lap)) {
    perLap.set(lap, {
      lap,
      samples: 0,
      mu: 0,
      planGrip: 0,
      driveScale: 0,
      brakeScale: 0,
      throttle: 0,
      throttleSaturated: 0,
      lateralDemand: 0,
      reserve: 0,
      overAsk: 0,
      tcActive: 0,
      gripUtil: 0,
      wear: 0,
      wearMax: 0,
      core: 0,
      surface: 0,
      slipPower: 0,
      slipPeak: 0,
    });
  }
  const row = perLap.get(lap);
  row[key] += value;
};
const tick = () => { const row = perLap.get(car.race.lap); if (row) row.samples++; };

while (step < 120 * 60 * 8) {
  session.step(1 / 120, zero);
  step++;
  if (session.phase !== 'racing') continue;

  if (car.race.lap !== prevLap) {
    if (car.race.lastLap !== null) {
      console.log(`Lap ${prevLap}: ${car.race.lastLap.toFixed(3)}s`);
    }
    prevLap = car.race.lap;
    if (car.race.lap > laps) break;
  }

  const env = driver.envelope;
  const limiter = driver.limiter;
  if (!limiter) continue;

  note('mu', env.muScale);
  note('planGrip', env.planGrip);
  note('driveScale', env.driveScale);
  note('brakeScale', env.brakeScale);
  note('throttle', car.controls.throttle);
  if (car.controls.throttle > 0.985) note('throttleSaturated', 1);
  note('lateralDemand', limiter.lateralDemand ?? 0);
  note('reserve', limiter.reserve ?? 0);
  note('tcActive', car.tcActive ? 1 : 0);
  note('gripUtil', limiter.gripUtil ?? 0);
  // Tyre energy budget: slip power is what the over-ask actually costs, and
  // wear is the interest rate on it. A per-lap mean hides the peak, so both
  // are accumulated and the max is tracked separately.
  const wheels = car.wheels ?? [];
  if (wheels.length === 4) {
    const wear = wheels.reduce((sum, w) => sum + (w.tyre?.wear ?? 0), 0) / 4;
    const core = wheels.reduce((sum, w) => sum + (w.tyre?.core ?? 0), 0) / 4;
    const surface = wheels.reduce((sum, w) => sum + (w.tyre?.surface ?? 0), 0) / 4;
    const slipPower = wheels.reduce((sum, w) => sum + (w.tyre?.slipPower ?? 0), 0) / 4;
    note('wear', wear);
    note('core', core);
    note('surface', surface);
    note('slipPower', slipPower);
    const row = perLap.get(car.race.lap);
    if (row) {
      row.wearMax = Math.max(row.wearMax ?? 0, wear);
      row.slipPeak = Math.max(row.slipPeak ?? 0, slipPower);
    }
  }
  // Over-ask proxy: throttle commanded beyond what the reserve-scaled
  // drive limit can deliver as a fraction of full torque.
  const drive = limiter.envDrive ?? 0;
  const drag = limiter.envDrag ?? 0;
  const reserve = limiter.reserve ?? 1;
  const phys = drive * reserve + drag;
  const asked = limiter.required ?? 0;
  if (phys > 0.2 && asked > 0) note('overAsk', Math.max(0, asked / Math.max(0.4, drive + drag) - phys / Math.max(0.4, drive + drag)));
  tick();
}

console.log('\n=== PER-LAP TYRE / THROTTLE STATE ===');
const rows = [...perLap.values()].sort((a, b) => a.lap - b.lap);
for (const r of rows) {
  const n = Math.max(1, r.samples);
  console.log(
    `lap ${r.lap}: mu=${(r.mu / n).toFixed(3)} planGrip=${(r.planGrip / n).toFixed(3)} ` +
    `driveScale=${(r.driveScale / n).toFixed(3)} brakeScale=${(r.brakeScale / n).toFixed(3)} | ` +
    `thr=${(r.throttle / n).toFixed(3)} sat=${(r.throttleSaturated / n).toFixed(3)} ` +
    `tc=${(r.tcActive / n).toFixed(3)} | latDem=${(r.lateralDemand / n).toFixed(3)} ` +
    `reserve=${(r.reserve / n).toFixed(3)} gripUtil=${(r.gripUtil / n).toFixed(3)} overAsk=${(r.overAsk / n).toFixed(3)}`
  );
  const wn = Math.max(1, r.samples);
  console.log(
    `      tyre: wear=${(r.wear / wn).toFixed(5)} (peak ${(r.wearMax ?? 0).toFixed(5)}) ` +
    `core=${(r.core / wn).toFixed(1)}C surface=${(r.surface / wn).toFixed(1)}C ` +
    `slipPwr=${(r.slipPower / wn).toFixed(0)}W (peak ${(r.slipPeak ?? 0).toFixed(0)}W)`
  );
}

console.log('\nmuScale is damped, so the per-lap mean is the right conditioning statistic.');
