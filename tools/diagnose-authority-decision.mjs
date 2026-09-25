/**
 * Authority-decision diagnostic.
 *
 * Every throttle reduction and every extra-authority grant has to be
 * explainable, so this dumps the full candidate evaluation the allocator made
 * at each control step, in the regions where the pace actually lives:
 *
 *   Fixture A  fresh exit gain   s 1000-1150, 1600-1800, 1900-2150
 *   Fixture B  final corner      s 2510-2660 (lap 4)
 *   Fixture C  combined-slip sweep (synthetic states, see --sweep)
 *
 *   node tools/diagnose-authority-decision.mjs
 *   node tools/diagnose-authority-decision.mjs --lap 2 --from 1000 --to 1150
 *   node tools/diagnose-authority-decision.mjs --sweep
 */
import { Track } from '../src/sim/track.js';
import { VortexSession } from '../src/vortex-session.js';
import { VortexDriver } from '../src/ai/vortex/vortex-driver.js';
import { TyrePredictor, gripSensitivityToCore } from '../src/ai/vortex/control/tyre-predictor.js';
import { clamp } from '../src/sim/math.js';

const argv = process.argv.slice(2);
const flag = n => argv.includes(n);
const number = (n, d) => { const i = argv.indexOf(n); return i >= 0 && Number.isFinite(Number(argv[i + 1])) ? Number(argv[i + 1]) : d; };

const FIXTURES = [
  { name: 'A1 exit 1000-1150', lap: 2, from: 1000, to: 1150 },
  { name: 'A2 exit 1600-1800', lap: 2, from: 1600, to: 1800 },
  { name: 'A3 exit 1900-2150', lap: 2, from: 1900, to: 2150 },
  { name: 'B  final corner 2510-2660', lap: 4, from: 2510, to: 2660 },
];

function runStint(laps = 4) {
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

  const log = [];
  driver.allocator.log = log;

  const zero = { steer: 0, throttle: 0, brake: 0 };
  const samples = [];
  let prevLap = car.race.lap;

  for (let step = 0; step < 120 * 60 * 8; step++) {
    session.step(1 / 120, zero);
    if (session.phase !== 'racing') continue;
    if (car.race.lap !== prevLap) {
      if (car.race.lastLap !== null) console.log(`lap ${prevLap}: ${car.race.lastLap.toFixed(3)}s`);
      prevLap = car.race.lap;
      if (car.race.lap > laps) break;
    }
    const lim = driver.limiter;
    if (!lim) continue;
    samples.push({
      lap: car.race.lap,
      s: car.s,
      u: car.u,
      throttle: car.controls.throttle,
      tPhys: lim.throttlePhysical,
      tCheap: lim.throttleFullAsk,
      beta: lim.beta,
      yawRate: car.yawRate,
      mu: driver.envelope.muScale,
      core: car.wheels.reduce((a, w) => a + w.tyre.core, 0) / 4,
      surface: car.wheels.reduce((a, w) => a + w.tyre.surface, 0) / 4,
      gripSens: gripSensitivityToCore(car.wheels[2].tyre, Math.max(1, car.wheels[2].load)),
      kappa: Math.max(Math.abs(car.wheels[2].tyre.kappa), Math.abs(car.wheels[3].tyre.kappa)),
      tc: car.tcActive ? 1 : 0,
      slipPower: (car.wheels[2].tyre.slipPower + car.wheels[3].tyre.slipPower) / 2,
      decision: lim.authorityDecision,
    });
  }
  return { samples, log, session };
}

function summarise(rows, label) {
  if (!rows.length) { console.log(`\n=== ${label}: no samples ===`); return; }
  const mean = k => rows.reduce((a, r) => a + r[k], 0) / rows.length;
  const pct = (k, p) => { const v = rows.map(r => r[k]).sort((a, b) => a - b); return v[Math.min(v.length - 1, Math.floor(v.length * p))]; };
  console.log(`\n=== ${label} (${rows.length} steps) ===`);
  console.log(
    `  throttle ${mean('throttle').toFixed(3)} (phys ${mean('tPhys').toFixed(3)} -> full-ask ${mean('tCheap').toFixed(3)})` +
    ` | granted ${rows.filter(r => r.throttle > r.tPhys + 0.005).length} steps`
  );
  console.log(
    `  core ${mean('core').toFixed(1)}C surface ${mean('surface').toFixed(1)}C` +
    ` | gripSens ${mean('gripSens').toExponential(2)}/C`
  );
  console.log(
    `  beta ${mean('beta').toFixed(3)} (p95 ${pct('beta', 0.95).toFixed(3)}, max ${Math.max(...rows.map(r => Math.abs(r.beta))).toFixed(3)})` +
    ` | kappa p95 ${pct('kappa', 0.95).toFixed(3)} | slipPower ${mean('slipPower').toFixed(0)}W` +
    ` | TC ${mean('tc').toFixed(2)}`
  );
  const withDecision = rows.filter(r => r.decision);
  if (withDecision.length) {
    const best = withDecision.map(r => r.decision);
    const evs = best.flatMap(d => d.evaluations);
    const m = (arr, k) => arr.length ? arr.reduce((a, x) => a + x[k], 0) / arr.length : 0;
    console.log(`  decisions ${best.length}, evaluations ${evs.length}`);
    console.log(
      `  mean over evaluations: benefit ${m(evs, 'benefit').toExponential(2)}s` +
      ` thermal ${m(evs, 'thermalCost').toExponential(2)}s` +
      ` stability ${m(evs, 'stability').toExponential(2)}s` +
      ` J ${m(evs, 'J').toExponential(2)}`
    );
    console.log(`  mean selected throttle ${m(best.map(d => ({ v: d.selected })), 'v').toFixed(3)}`);
    let widest = null, widestGap = -1;
    for (const d of best) {
      const g = d.tCheap - d.tPhys;
      if (g > widestGap) { widestGap = g; widest = d; }
    }
    if (widest) {
      console.log(`  widest gap sample: phys ${widest.tPhys.toFixed(3)} -> full-ask ${widest.tCheap.toFixed(3)}, selected ${widest.selected.toFixed(3)}`);
      console.log('    cand    ax     kappa  util   slipPwr    TC   beta     benefit    thermal   stab      J');
      for (const e of widest.evaluations) {
        console.log(
          `    ${e.throttle.toFixed(3)}  ${e.ax.toFixed(2)}  ${e.kappaMax.toFixed(3)}  ${e.utilMax.toFixed(3)}  ` +
          `${e.slipPowerMean.toFixed(0).padStart(6)}  ${e.tcMean.toFixed(2)}  ${e.beta.toFixed(3)}  ` +
          `${e.benefit.toExponential(2)}  ${e.thermalCost.toExponential(2)}  ${e.stability.toExponential(2)}  ${e.J.toExponential(2)}`
        );
      }
    }
  }
}

if (flag('--sweep')) {
  // Fixture C: synthetic combined-slip states. The allocator's decision must
  // vary smoothly with physical risk and never depend on where on the track we
  // are, so the sweep deliberately holds the track out of it entirely.
  console.log('=== FIXTURE C: generic combined-slip sweep ===');
  const track = new Track('harbor-ring');
  const session = new VortexSession(track, { classId: 'gt' });
  session.mode = 'practice'; session.field = 1; session.laps = 1; session.autopilot = true;
  session.start({ freshTrack: true });
  const car = session.cars[0];
  const driver = new VortexDriver(0, session.lineFor(car), {});
  session.drivers[0] = driver;
  const zero = { steer: 0, throttle: 0, brake: 0 };
  for (let i = 0; i < 200; i++) session.step(1 / 120, zero);

  const predictor = new TyrePredictor({ horizon: 0.2 });
  const rows = [];
  for (const speed of [12, 20, 30, 40]) {
    for (const curvature of [0.002, 0.01, 0.03, 0.06]) {
      for (const core of [72, 85, 95, 105]) {
        for (const kappa of [0, 0.08, 0.16, 0.28]) {
          // Clone a live wheel pair and force the state under test.
          const wheels = car.wheels.map(w => ({ ...w, tyre: { ...w.tyre } }));
          for (const w of [wheels[2], wheels[3]]) {
            w.tyre.core = core;
            w.tyre.surface = core + 18;
            w.tyre.pressure = (w.tyre.coldPressure + 1.01325) * ((core + 273.15) / 297.15) - 1.01325;
            w.tyre.kappa = kappa;
            w.tyre.wear = clamp((core - 80) / 400, 0, 1);
            w.load = 3300;
            w.omega = speed * (1 + kappa) / car.spec.radius;
          }
          const ego = {
            ...car,
            u: speed, v: 0.4, yawRate: curvature * speed, speed,
            fuel: 20, damage: 0,
            spec: car.spec, setup: car.setup,
            wheels,
          };
          const lateral = clamp(speed * speed * curvature / 13, 0, 0.985);
          const reserve = Math.sqrt(Math.max(0, 1 - lateral * lateral));
          const envelope = driver.envelope.at(ego, speed, curvature, 0);
          const drive = Math.max(0.35, envelope.drive * reserve);
          const drag = envelope.drag * reserve;
          const accel = drive * 0.8;
          const demand = accel + drag;
          const tPhys = clamp(demand / Math.max(0.4, envelope.drive + drag), 0, 1);
          const tCheap = clamp(demand / Math.max(0.4, drive + drag), 0, 1);
          const base = predictor.predict(ego, envelope, tPhys);
          const full = predictor.predict(ego, envelope, tCheap);
          const thermal = predictor.thermalCost(ego, full.slipPowerMean - base.slipPowerMean);
          rows.push({
            speed, curvature, core, kappa, gap: tCheap - tPhys,
            dAx: full.ax - base.ax,
            dSlip: full.slipPowerMean - base.slipPowerMean,
            kappaMax: full.kappaMax, utilMax: full.utilMax,
            dBeta: full.dBeta,
            dGrip: thermal.dGrip,
            gripSens: gripSensitivityToCore(wheels[2].tyre, 3300),
          });
        }
      }
    }
  }
  console.log('v    curv   core  kappa  gap    dAx    dSlipW  kappaMax util   dBeta    dGrip     gripSens');
  for (const r of rows) {
    console.log(
      `${String(r.speed).padStart(2)}  ${r.curvature.toFixed(3)}  ${String(r.core).padStart(3)}  ${r.kappa.toFixed(2)}  ` +
      `${r.gap.toFixed(3)}  ${r.dAx.toFixed(2).padStart(5)}  ${r.dSlip.toFixed(0).padStart(7)}  ` +
      `${r.kappaMax.toFixed(3)}  ${r.utilMax.toFixed(3)}  ${r.dBeta.toFixed(4)}  ${r.dGrip.toExponential(2)}  ${r.gripSens.toExponential(2)}`
    );
  }
  process.exit(0);
}

const lapFilter = flag('--lap') ? number('--lap', 2) : null;
const from = number('--from', null);
const to = number('--to', null);
const { samples } = runStint(4);

if (lapFilter !== null && from !== null) {
  summarise(samples.filter(r => r.lap === lapFilter && r.s >= from && r.s <= to), `lap ${lapFilter} s ${from}-${to}`);
} else {
  for (const f of FIXTURES) {
    summarise(samples.filter(r => r.lap === f.lap && r.s >= f.from && r.s <= f.to), `FIXTURE ${f.name}`);
  }
  summarise(samples.filter(r => r.lap <= 3), 'ALL FRESH (laps 1-3)');
  summarise(samples.filter(r => r.lap === 4), 'ALL LAP 4');
}
