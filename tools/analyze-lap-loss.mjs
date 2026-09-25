/**
 * Pace-loss attribution for VORTEX.
 *
 * Runs one flying lap on the frozen canonical simulator and decomposes the
 * difference between the driven lap and the optimised oracle profile into
 * per-microsector losses: line (lateral offset), minimum speed (corner entry),
 * exit (acceleration away from a corner), braking and control.
 *
 *   node tools/analyze-lap-loss.mjs
 *   node tools/analyze-lap-loss.mjs --laps 2 --sectors 12
 */
import { Track } from '../src/sim/track.js';
import { wrap, angle } from '../src/sim/math.js';
import { tyreGrip } from '../src/sim/tyre.js';
import { VortexSession } from '../src/vortex-session.js';

const argv = process.argv.slice(2);
const number = (name, fallback) => {
  const i = argv.indexOf(name);
  return i >= 0 && Number.isFinite(Number(argv[i + 1])) ? Number(argv[i + 1]) : fallback;
};

const track = new Track('harbor-ring');
const trackLength = track.length;
const session = new VortexSession(track, { classId: 'gt' });
session.mode = 'practice'; session.field = 1; session.laps = number('--laps', 2); session.autopilot = true;
session.start({ freshTrack: true });

const driver = session.drivers[0];
const atlas = driver.atlas;
const zero = { steer: 0, throttle: 0, brake: 0 };
const sectorCount = number('--sectors', 12);
const sectorLength = track.length / sectorCount;

const samples = [];
let lapStart = 0;
let previousLap = session.player.race.lap;
let lapValid = true;
const laps = [];

for (let step = 0; step < 120 * 700; step++) {
  const priorLap = session.player.race.lap;
  const priorValid = session.player.race.valid;
  session.step(1 / 120, zero);
  const car = session.player;
  if (session.phase !== 'racing') continue;
  if (car.race.lap !== previousLap) {
    laps.push({ lap: priorLap, time: car.race.lastLap, valid: lapValid && priorValid, from: lapStart, to: samples.length });
    previousLap = car.race.lap;
    lapValid = car.race.valid;
    lapStart = samples.length;
    if (laps.length >= number('--laps', 2)) break;
    continue;
  }
  if (!car.race.valid) lapValid = false;
  samples.push({
    s: car.s,
    v: car.speed,
    plan: atlas.profileSpeed(car.s),
    planLimit: driver.planSpeed(car.s),
    target: driver.targetSpeed,
    q: atlas.lineOffset(car.s),
    planOffset: driver.planner.at(car.s).offset,
    residualQ: atlas.memory?.residualAt(car.s)?.residualQ ?? 0,
    lateral: car.lateral,
    gripUtil: Math.abs(car.ay) / Math.max(1, atlas.envelope.lateralAt(car.speed)),
    throttle: car.controls.throttle,
    brake: car.controls.brake,
    state: driver.state,
    headingError: Math.abs(angle(car.heading - car.yaw)),
    lap: car.race.lap,
    fuel: car.fuel,
    tyreGrip: (car.wheels ?? []).reduce((sum, wheel) => sum + tyreGrip(wheel.tyre, Math.max(1500, wheel.load || 3300)), 0) / Math.max(1, (car.wheels ?? []).length),
    tyreTemp: (car.wheels ?? []).reduce((sum, wheel) => sum + (wheel.tyre?.temp ?? 0), 0) / Math.max(1, (car.wheels ?? []).length),
    required: driver.limiter?.required ?? 0,
    driveLimit: driver.limiter?.drive ?? 0,
    feedforward: driver.limiter?.feedforward ?? 0,
    limited: driver.limiter?.targetSpeed ?? 0,
    ay: car.ay,
    ax: car.ax,
  });
}

const flying = laps.filter(lap => lap.valid && lap.time > 20);
if (!flying.length) { console.error('no valid lap recorded'); process.exit(1); }

// Use the fastest valid lap: slower laps usually contain a recovery or a
// traffic compromise and would smear the attribution.
const lap = flying.reduce((best, candidate) => (candidate.time < best.time ? candidate : best), flying[0]);
const window = samples.slice(lap.from, lap.to);
const bucket = Array.from({ length: sectorCount }, () => ({
  distance: 0, time: 0, planTime: 0, vMin: Infinity, vMax: 0, planMin: Infinity,
  deficitSum: 0, samples: 0, qErrorSum: 0, brakeSum: 0, throttleSum: 0,
}));

let total = 0, planTotal = 0;
for (let i = 1; i < window.length; i++) {
  const a = window[i - 1], b = window[i];
  const ds = ((b.s - a.s + track.length) % track.length) || 0;
  if (ds > 5) continue;
  const index = Math.floor(b.s / sectorLength) % sectorCount;
  const cell = bucket[index];
  const dt = ds / Math.max(3, (a.v + b.v) * 0.5);
  const dtPlan = ds / Math.max(3, (a.plan + b.plan) * 0.5);
  cell.distance += ds; cell.time += dt; cell.planTime += dtPlan;
  cell.vMin = Math.min(cell.vMin, b.v); cell.planMin = Math.min(cell.planMin, b.plan);
  cell.deficitSum += Math.max(0, b.plan - b.v) * ds;
  cell.qErrorSum += Math.abs(b.lateral - b.q) * ds;
  cell.brakeSum += b.brake * ds; cell.throttleSum += b.throttle * ds;
  cell.samples++;
  total += dt; planTotal += dtPlan;
}

const sectors = bucket.map((cell, index) => ({
  sector: index + 1,
  sFrom: Number((index * sectorLength).toFixed(0)),
  sTo: Number(((index + 1) * sectorLength).toFixed(0)),
  loss: Number((cell.time - cell.planTime).toFixed(3)),
  driven: Number(cell.time.toFixed(3)),
  planned: Number(cell.planTime.toFixed(3)),
  vMin: Number((Number.isFinite(cell.vMin) ? cell.vMin : 0).toFixed(2)),
  planMin: Number((Number.isFinite(cell.planMin) ? cell.planMin : 0).toFixed(2)),
  meanDeficit: Number((cell.deficitSum / Math.max(1, cell.distance)).toFixed(2)),
  meanLineError: Number((cell.qErrorSum / Math.max(1, cell.distance)).toFixed(2)),
  brakeFraction: Number((cell.brakeSum / Math.max(1, cell.distance)).toFixed(3)),
  throttleFraction: Number((cell.throttleSum / Math.max(1, cell.distance)).toFixed(3)),
})).sort((a, b) => b.loss - a.loss);

const cleanSectors = sectors.filter(item => item.vMin > 1);

const kappaAt = (atlas, s) => {
  const point = atlas.sample(s, 0);
  return Number.isFinite(point?.curvature) ? point.curvature : 0;
};

// Is the plan's lateral geometry reachable? A quasi-steady oracle has no
// steering-rate or lateral-acceleration-rise constraint, so it can demand a
// lateral velocity the closed-loop car simply cannot produce. If the worst
// line errors coincide with the largest demanded lateral rates, the fix is
// geometric smoothing of the plan, not more servo gain.
const stations = [];
for (let i = 1; i + 1 < samples.length; i++) {
  const a = samples[i - 1], b = samples[i], c = samples[i + 1];
  const ds = wrap(c.s - a.s, trackLength);
  if (ds <= 0 || ds > 40) continue;
  const demandRate = ((c.q - a.q) / ds) * b.v;
  const actualRate = (c.lateral - a.lateral) / Math.max(1e-3, (c.v > 0 ? ds / Math.max(2, b.v) : 1 / 120));
  stations.push({
    s: Number(b.s.toFixed(1)),
    v: Number(b.v.toFixed(2)),
    error: Number(Math.abs(b.lateral - b.q).toFixed(2)),
    lateral: Number(b.lateral.toFixed(2)),
    q: Number(b.q.toFixed(2)),
    planOffset: Number(b.planOffset.toFixed(2)),
    residualQ: Number(b.residualQ.toFixed(2)),
    demandRate: Number(demandRate.toFixed(2)),
    actualRate: Number(actualRate.toFixed(2)),
    deficit: Number((Math.abs(demandRate) - Math.abs(actualRate)).toFixed(2)),
    ay: Number(b.ay.toFixed(2)),
    throttle: Number(b.throttle.toFixed(2)),
    brake: Number(b.brake.toFixed(2)),
    k: Number(kappaAt(atlas, b.s).toFixed(5)),
  });
}
const worstRate = stations.slice().sort((x, y) => y.demandRate - x.demandRate).slice(0, 10);
const worstError = stations.slice().sort((x, y) => y.error - x.error).slice(0, 10);

// Coarse error map: which stretch of the circuit does the driver actually lose
// the line on, and how far does it drift before it recovers.
const bucketSize = 100;
const bucketCount = Math.ceil(trackLength / bucketSize);
const buckets = new Array(bucketCount).fill(0).map(() => ({ n: 0, error: 0, lateral: 0, q: 0, deficit: 0 }));
for (const item of stations) {
  const index = Math.floor(wrap(item.s, trackLength) / bucketSize);
  const bucket = buckets[index];
  bucket.n++;
  bucket.error += item.error;
  bucket.lateral += item.lateral;
  bucket.q += item.q;
  bucket.deficit += item.deficit;
}
const errorMap = buckets.map((bucket, index) => ({
  s: index * bucketSize,
  meanError: bucket.n ? Number((bucket.error / bucket.n).toFixed(2)) : 0,
  meanLateral: bucket.n ? Number((bucket.lateral / bucket.n).toFixed(2)) : 0,
  meanQ: bucket.n ? Number((bucket.q / bucket.n).toFixed(2)) : 0,
  samples: bucket.n,
})).filter(item => item.samples > 0);

const report = {
  track: 'harbor-ring',
  measuredLap: Number(lap.time.toFixed(3)),
  oracleLap: atlas.oracle.lapTime,
  gap: Number((lap.time - atlas.oracle.lapTime).toFixed(3)),
  laps: flying.map(item => ({ lap: item.lap, time: Number(item.time.toFixed(3)), valid: item.valid })),
  profileIntegratedTime: Number(planTotal.toFixed(3)),
  drivenIntegratedTime: Number(total.toFixed(3)),
  integrationLoss: Number((total - planTotal).toFixed(3)),
  gripScale: atlas.envelope.gripScale,
  qMax: atlas.qMax,
  maxDemandRate: Math.max(...stations.map(item => item.demandRate)),
  meanAbsError: Number((stations.reduce((sum, item) => sum + item.error, 0) / stations.length).toFixed(3)),
  // If the planner's own plan speed sits below the oracle profile the car is
  // under-driving by construction and no amount of servo tuning will recover it.
  meanPlanCap: Number((samples.reduce((sum, item) => sum + (item.plan - item.planLimit), 0) / Math.max(1, samples.length)).toFixed(3)),
  meanTargetDeficit: Number((samples.reduce((sum, item) => sum + (item.target - item.v), 0) / Math.max(1, samples.length)).toFixed(3)),
  meanThrottle: Number((samples.reduce((sum, item) => sum + item.throttle, 0) / Math.max(1, samples.length)).toFixed(3)),
  // The two numbers that decide whether the residual speed deficit is physics
  // or control: if the tyre is saturated there is no throttle to give, and if
  // throttle is open while the tyre still has margin the longitudinal law is
  // simply not asking for the speed.
  meanGripUtil: Number((samples.reduce((sum, item) => sum + item.gripUtil, 0) / Math.max(1, samples.length)).toFixed(3)),
  behind: (() => {
    const set = samples.filter(item => item.target - item.v > 2);
    const avg = (key) => set.length ? Number((set.reduce((sum, item) => sum + item[key], 0) / set.length).toFixed(3)) : null;
    return {
      samples: set.length,
      fraction: Number((set.length / Math.max(1, samples.length)).toFixed(3)),
      meanThrottle: avg('throttle'),
      meanGripUtil: avg('gripUtil'),
      meanRequired: avg('required'),
      meanDriveLimit: avg('driveLimit'),
      meanFeedforward: avg('feedforward'),
      meanDeficit: set.length ? Number((set.reduce((sum, item) => sum + (item.target - item.v), 0) / set.length).toFixed(3)) : null,
    };
  })(),
  // Plant state per lap: if grip or fuel drifts, the fixed oracle profile
  // becomes unreachable and the car departs exactly as a stint wears on.
  perLap: (() => {
    const byLap = new Map();
    for (const item of samples) {
      if (!byLap.has(item.lap)) byLap.set(item.lap, []);
      byLap.get(item.lap).push(item);
    }
    return [...byLap.entries()].map(([lap, set]) => ({
      lap,
      samples: set.length,
      meanFuel: Number((set.reduce((sum, item) => sum + item.fuel, 0) / set.length).toFixed(2)),
      meanTyreGrip: Number((set.reduce((sum, item) => sum + item.tyreGrip, 0) / set.length).toFixed(3)),
      meanTyreTemp: Number((set.reduce((sum, item) => sum + item.tyreTemp, 0) / set.length).toFixed(2)),
      meanGripUtil: Number((set.reduce((sum, item) => sum + item.gripUtil, 0) / set.length).toFixed(3)),
      meanThrottle: Number((set.reduce((sum, item) => sum + item.throttle, 0) / set.length).toFixed(3)),
      meanAbsError: Number((set.reduce((sum, item) => sum + Math.abs(item.lateral - item.q), 0) / set.length).toFixed(2)),
      recoveryFraction: Number((set.filter(item => item.state !== 'FREE AIR').length / set.length).toFixed(3)),
    }));
  })(),
  // Where each lap first loses the car. If the station repeats, the plan is
  // asking for something the car cannot do at that corner; if it wanders, the
  // cause is a drifting plant or state.
  departureStations: (() => {
    const out = [];
    const seen = new Set();
    for (const item of samples) {
      if (item.state === 'FREE AIR') continue;
      const key = item.lap;
      if (seen.has(key)) continue;
      seen.add(key);
      out.push({
        lap: item.lap,
        s: Number(item.s.toFixed(1)),
        v: Number(item.v.toFixed(2)),
        lateral: Number(item.lateral.toFixed(2)),
        q: Number(item.q.toFixed(2)),
        plan: Number(item.plan.toFixed(2)),
        target: Number(item.limited.toFixed(2)),
        gripUtil: Number(item.gripUtil.toFixed(3)),
      });
    }
    return out;
  })(),
  stateFraction: (() => {
    const counts = {};
    for (const item of samples) counts[item.state] = (counts[item.state] ?? 0) + 1;
    return Object.fromEntries(Object.entries(counts).map(([key, value]) => [key, Number((value / samples.length).toFixed(3))]));
  })(),
  // Which recovery clause is actually firing: the deep-off trigger, the
  // heading trigger, the reverse timer, or the hysteresis.
  recovery: (() => {
    const set = samples.filter(item => item.state === 'PHYSICAL REJOIN' || item.state === 'REVERSE RECOVERY');
    if (!set.length) return { samples: 0 };
    const avg = (key) => Number((set.reduce((sum, item) => sum + item[key], 0) / set.length).toFixed(3));
    const max = (key) => Number(Math.max(...set.map(item => item[key])).toFixed(3));
    return {
      samples: set.length,
      fraction: Number((set.length / samples.length).toFixed(3)),
      meanLateral: avg('lateral'),
      maxAbsLateral: max('lateral'),
      meanAbsLateral: Number((set.reduce((sum, item) => sum + Math.abs(item.lateral), 0) / set.length).toFixed(3)),
      meanSpeed: avg('v'),
      meanHeadingError: avg('headingError'),
      maxHeadingError: max('headingError'),
      slow: set.filter(item => item.v < 16).length,
      deepOff: set.filter(item => Math.abs(item.lateral) > 8.9 && item.v < 16).length,
      headingTrigger: set.filter(item => item.headingError > 1.3).length,
    };
  })(),
  worstSectors: sectors.slice(0, 6),
  worstRate,
  worstError,
  errorMap,
  focus: stations.filter(item => item.s >= 850 && item.s <= 1420).filter((_, index) => index % 20 === 0),
  sectors,
};

console.log(JSON.stringify(report, null, 2));
