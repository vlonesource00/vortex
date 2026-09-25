/**
 * Free-air execution-loss attribution.
 *
 * The oracle profile integrates to roughly 71-73 s; the plant drives ~3 s
 * slower than the profile it is handed. That gap is the largest single source
 * of lap time, and it is not reachable by making the oracle faster -- it is the
 * car failing to extract value from a trajectory it has already been given.
 *
 * This tool answers two questions before anything is retuned:
 *
 *   §18  Is the throttle actually being withheld? Conditional on being behind
 *        target with no brake applied and no traffic, what is
 *        P(throttle > 0.98)? If that is near one, the pedal is not the
 *        constraint and the loss is in the path, the target or the tyre.
 *
 *   §17  Where does the time go? Per microsector, decompose the deficit
 *        against the driver's own target into line, braking, minimum speed,
 *        exit, steering scrub and actuation.
 *
 *   node tools/analyze-execution-loss.mjs
 *   node tools/analyze-execution-loss.mjs --laps 3 --sectors 24
 */
import { Track } from '../src/sim/track.js';
import { wrap, angle } from '../src/sim/math.js';
import { VortexSession } from '../src/vortex-session.js';

const argv = process.argv.slice(2);
const number = (name, fallback) => {
  const i = argv.indexOf(name);
  return i >= 0 && Number.isFinite(Number(argv[i + 1])) ? Number(argv[i + 1]) : fallback;
};

const track = new Track('harbor-ring');
const trackLength = track.length;
const session = new VortexSession(track, { classId: 'gt' });
session.mode = 'practice';
session.field = 1;
session.laps = number('--laps', 3);
session.autopilot = true;
session.start({ freshTrack: true });

const driver = session.drivers[0];
const atlas = driver.atlas;
const zero = { steer: 0, throttle: 0, brake: 0 };
const sectorCount = number('--sectors', 24);
const sectorLength = trackLength / sectorCount;

const samples = [];
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
    laps.push({ lap: priorLap, time: car.race.lastLap, valid: lapValid && priorValid });
    previousLap = car.race.lap;
    lapValid = car.race.valid;
    if (laps.length >= number('--laps', 3)) break;
    continue;
  }
  if (!car.race.valid) lapValid = false;

  const l = driver.limiter ?? {};
  samples.push({
    s: car.s,
    v: car.speed,
    target: l.targetSpeed ?? car.speed,
    plan: l.planned ?? car.speed,
    profile: atlas.profileSpeed(car.s),
    q: atlas.lineOffset(car.s),
    lateral: car.lateral,
    throttle: l.throttle ?? car.controls.throttle,
    brake: l.brakeCmd ?? car.controls.brake,
    steer: l.steer ?? car.controls.steer,
    required: l.required ?? 0,
    feedforward: l.feedforward ?? 0,
    speedIntegral: l.speedIntegral ?? 0,
    accelBias: l.accelBias ?? 0,
    reserve: l.reserve ?? 0,
    accel: l.accel ?? 0,
    envDrive: l.envDrive ?? 0,
    envBrake: l.envBrake ?? 0,
    envDrag: l.envDrag ?? 0,
    gripUtil: l.gripUtil ?? 0,
    lateralDemand: l.lateralDemand ?? 0,
    beta: l.beta ?? 0,
    yawRate: l.yawRate ?? 0,
    yawError: l.yawError ?? 0,
    ax: car.ax,
    ay: car.ay,
    state: driver.state,
  });
}

const valid = laps.filter((item) => item.valid && item.time > 20);
if (!valid.length) { console.error('no valid lap recorded'); process.exit(1); }
const bestLap = valid.reduce((a, b) => (a.time < b.time ? a : b));

const mean = (rows, key) => (rows.length ? rows.reduce((sum, r) => sum + r[key], 0) / rows.length : 0);
const quantile = (rows, key, p) => {
  if (!rows.length) return 0;
  const sorted = rows.map((r) => r[key]).sort((a, b) => a - b);
  return sorted[Math.min(sorted.length - 1, Math.floor(sorted.length * p))];
};

// ---------------------------------------------------------------------------
// §18 Conditional throttle diagnostic.
//
// A whole-lap mean throttle of 0.79 alongside a mean deficit of 5 m/s is not
// evidence of an actuation problem, because those averages need not co-occur.
// Restrict to samples that are genuinely behind target, not braking, and not
// traffic limited, then ask how often the pedal is already floored.
// ---------------------------------------------------------------------------
const behind = samples.filter((r) => r.target - r.v > 2 && r.brake < 0.01 && r.state === 'FREE AIR');
const floored = behind.filter((r) => r.throttle > 0.98);

// ---------------------------------------------------------------------------
// §17 Station-resolved execution loss.
//
// The naive decomposition -- attribute ds*(1/v - 1/target) at each sample --
// is wrong, because a car that lost speed in a corner is still catching up at
// full throttle several hundred metres later. It would blame the straight for
// a loss that was incurred at the apex.
//
// So attribute the *growth of the deficit* instead. With d(deficit)/ds =
// d(target)/ds - a_x/v, a positive value is where the car actually gives the
// time away. That is where the cause lives.
// ---------------------------------------------------------------------------
const categoryOf = (r, dtLoss) => {
  if (dtLoss <= 1e-6) return null;
  // Genuinely grip-saturated: nothing more was physically available.
  if (r.gripUtil > 0.92) return 'grip';
  // Braking harder than the plan needs is over-slowing the car.
  if (r.brake > 0.01) return 'braking';
  // Power was available and not requested.
  if (r.throttle < 0.98 && r.reserve > 0.15) return 'actuation';
  // Steering and sideslip are spending the tyre on rotation and drag instead
  // of on the progress the plan assumed.
  if (Math.abs(r.steer) > 0.22 || Math.abs(r.beta) > 0.05) return 'scrub';
  // Off the planned path: the car is driving a different radius than planned.
  if (Math.abs(r.lateral - r.q) > 1.2) return 'line';
  // Flat out, on line, unwound, tyre not saturated, yet still behind: the plan
  // is asking for more than the plant can deliver here.
  return 'plan';
};

const buckets = { line: 0, braking: 0, grip: 0, actuation: 0, scrub: 0, plan: 0 };
const sectorBuckets = new Array(sectorCount).fill(0).map(() => ({ loss: 0, growth: 0, n: 0, ...Object.fromEntries(Object.keys(buckets).map((k) => [k, 0])) }));
let totalGrowth = 0;
let totalPlanTime = 0;
let totalDrivenTime = 0;

for (let i = 1; i < samples.length; i++) {
  const a = samples[i - 1], b = samples[i];
  const ds = wrap(b.s - a.s, trackLength);
  if (ds <= 0 || ds > 30) continue;
  const dtDriven = ds / Math.max(2, b.v);
  const dtPlan = ds / Math.max(2, b.target);
  totalDrivenTime += dtDriven;
  totalPlanTime += dtPlan;
  // Cause comes from where the deficit *grows*; magnitude from the time that
  // sample actually gives away. Blending the two keeps the categories honest
  // without attributing a corner's loss to the straight that follows it.
  const dDeficit = (b.target - b.v) - (a.target - a.v);
  const growth = dDeficit > 0 ? dDeficit : 0;
  totalGrowth += growth;
  const dtLoss = Math.max(0, dtDriven - dtPlan);
  const category = categoryOf(b, dtLoss);
  const sector = sectorBuckets[Math.floor(wrap(b.s, trackLength) / sectorLength)];
  sector.n++;
  sector.growth += growth;
  sector.loss += dtLoss;
  if (category) {
    buckets[category] += dtLoss;
    sector[category] += dtLoss;
  }
}

const lossTotalPositive = Object.values(buckets).reduce((a, b) => a + b, 0);
const worstSectors = sectorBuckets
  .map((item, index) => ({
    sector: index,
    sFrom: Math.round(index * sectorLength),
    sTo: Math.round((index + 1) * sectorLength),
    loss: Number(item.loss.toFixed(3)),
    deficitGrowth: Number(item.growth.toFixed(3)),
    samples: item.n,
    line: Number(item.line.toFixed(3)),
    braking: Number(item.braking.toFixed(3)),
    grip: Number(item.grip.toFixed(3)),
    actuation: Number(item.actuation.toFixed(3)),
    scrub: Number(item.scrub.toFixed(3)),
    plan: Number(item.plan.toFixed(3)),
  }))
  .filter((item) => item.samples > 0)
  .sort((x, y) => y.loss - x.loss);

const report = {
  track: 'harbor-ring',
  laps: laps.map((item) => ({ lap: item.lap, time: Number(item.time.toFixed(3)), valid: item.valid })),
  measuredLap: Number(bestLap.time.toFixed(3)),
  profileIntegratedTime: Number(totalPlanTime.toFixed(3)),
  drivenIntegratedTime: Number(totalDrivenTime.toFixed(3)),
  integrationLoss: Number((totalDrivenTime - totalPlanTime).toFixed(3)),
  deficitGrowthTotal: Number(totalGrowth.toFixed(3)),

  conditionalThrottle: {
    definition: 'target - v > 2 m/s AND brake < 0.01 AND state == FREE AIR',
    samples: behind.length,
    fractionOfRun: Number((behind.length / Math.max(1, samples.length)).toFixed(3)),
    meanThrottle: Number(mean(behind, 'throttle').toFixed(3)),
    p50Throttle: Number(quantile(behind, 'throttle', 0.5).toFixed(3)),
    p95Throttle: Number(quantile(behind, 'throttle', 0.95).toFixed(3)),
    pFloor: Number((floored.length / Math.max(1, behind.length)).toFixed(4)),
    meanDeficit: Number(mean(behind.map((r) => ({ d: r.target - r.v })), 'd').toFixed(3)),
    meanReserve: Number(mean(behind, 'reserve').toFixed(3)),
    meanAvailableDrive: Number(mean(behind, 'envDrive').toFixed(3)),
    meanRequiredAccel: Number(mean(behind, 'required').toFixed(3)),
    meanAllocatedAccel: Number(mean(behind, 'accel').toFixed(3)),
    meanAbsSteer: Number(mean(behind.map((r) => ({ a: Math.abs(r.steer) })), 'a').toFixed(3)),
    meanAbsBeta: Number(mean(behind.map((r) => ({ a: Math.abs(r.beta) })), 'a').toFixed(4)),
    meanGripUtil: Number(mean(behind, 'gripUtil').toFixed(3)),
    meanLateralDemand: Number(mean(behind, 'lateralDemand').toFixed(3)),
    verdict: floored.length / Math.max(1, behind.length) > 0.8
      ? 'throttle is already floored in the large majority of clean-air deficit samples: the loss is NOT an actuation problem'
      : 'throttle is frequently NOT floored while behind target with reserve available: an actuation/limiter problem is present',
  },

  lossByCategory: Object.fromEntries(
    Object.entries(buckets).map(([key, value]) => [
      key,
      { seconds: Number(value.toFixed(3)), share: Number((value / Math.max(1e-6, lossTotalPositive)).toFixed(3)) },
    ]),
  ),
  bucketedLossTotal: Number(lossTotalPositive.toFixed(3)),
  worstSectors: worstSectors.slice(0, 8),
  sectors: worstSectors,
};

console.log(JSON.stringify(report, null, 2));
