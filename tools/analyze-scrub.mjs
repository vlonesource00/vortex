/**
 * Scrub-origin attribution.
 *
 * The execution-loss split put 1.147 s/lap into "scrub" -- sideslip and
 * steering spending the tyre on rotation instead of progress. That is a
 * symptom label, not a cause. This tool finds the cause.
 *
 * It answers, in order:
 *   §6   which servo term is generating the steering that produces the slip;
 *   §10  whether the curvature feed-forward is even the right shape;
 *   §11  whether the slip term is damping beta or feeding it;
 *   §12  whether the yaw-rate target is being tracked or overshot;
 *   §9   whether the car holds steering after the rotation is already done;
 *   §7   which beta is useful rotation and which is dissipative;
 *   §15  one primary classification per scrub microsector.
 *
 *   node tools/analyze-scrub.mjs
 *   node tools/analyze-scrub.mjs --laps 3
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
const zero = { steer: 0, throttle: 0, brake: 0 };

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
  const servo = l.servo ?? {};
  const planPoint = driver.planner.at(car.s);
  const near = driver.planner.at(car.s + 4);
  const far = driver.planner.at(car.s + 10);
  const dq = (near.offset - planPoint.offset) / 4;
  const d2q = (far.offset - 2 * near.offset + planPoint.offset) / 16;
  const wheels = car.wheels ?? [];

  samples.push({
    s: car.s,
    lap: car.race.lap,
    v: car.speed,
    target: l.targetSpeed ?? car.speed,
    lateral: car.lateral,
    q: planPoint.offset,
    qError: car.lateral - planPoint.offset,
    dq, d2q,
    trajCurvature: planPoint.curvature ?? 0,
    drivenCurvature: car.speed > 1 ? car.yawRate / car.speed : 0,
    steer: l.steer ?? car.controls.steer,
    steerRate: 0,
    throttle: l.throttle ?? car.controls.throttle,
    brake: l.brakeCmd ?? car.controls.brake,
    beta: l.beta ?? 0,
    yawRate: l.yawRate ?? 0,
    yawError: l.yawError ?? 0,
    ax: car.ax,
    ay: car.ay,
    gripUtil: l.gripUtil ?? 0,
    reserve: l.reserve ?? 0,
    envDrive: l.envDrive ?? 0,
    frontFy: wheels.length === 4 ? wheels[0].tyre.fy + wheels[1].tyre.fy : 0,
    rearFy: wheels.length === 4 ? wheels[2].tyre.fy + wheels[3].tyre.fy : 0,
    // Steering decomposition (§6) straight from the servo's own terms.
    tPursuit: servo.pursuit ?? 0,
    tSlip: servo.slip ?? 0,
    tTracking: servo.tracking ?? 0,
    tDamping: servo.damping ?? 0,
    tRotation: servo.rotation ?? 0,
    tFeedforward: servo.feedforward ?? 0,
    tYawError: servo.yawError ?? 0,
    tCorrection: servo.correction ?? 0,
    raw: servo.raw ?? 0,
  });
}

// Steering rate from consecutive samples.
for (let i = 1; i < samples.length; i++) {
  samples[i].steerRate = (samples[i].steer - samples[i - 1].steer) * 120;
}

const valid = laps.filter((item) => item.valid && item.time > 20);
if (!valid.length) { console.error('no valid lap recorded'); process.exit(1); }

const mean = (rows, key) => (rows.length ? rows.reduce((s, r) => s + (typeof key === 'function' ? key(r) : r[key]), 0) / rows.length : 0);
const absMean = (rows, key) => mean(rows.map((r) => ({ v: Math.abs(typeof key === 'function' ? key(r) : r[key]) })), 'v');
const peak = (rows, key) => (rows.length ? Math.max(...rows.map((r) => Math.abs(typeof key === 'function' ? key(r) : r[key]))) : 0);

// Local time loss against the driver's own target, used as the scrub magnitude.
for (let i = 1; i < samples.length; i++) {
  const a = samples[i - 1], b = samples[i];
  const ds = wrap(b.s - a.s, trackLength);
  b.ds = (ds > 0 && ds < 30) ? ds : 0;
  b.dtLoss = b.ds > 0 ? Math.max(0, b.ds / Math.max(2, b.v) - b.ds / Math.max(2, b.target)) : 0;
}
samples[0].ds = 0; samples[0].dtLoss = 0;

// ---------------------------------------------------------------------------
// §7 Scrub events. Not every nonzero beta is a defect: an event needs real
// sideslip AND measurable time being given away. USEFUL_ROTATION is a high-beta
// passage that is not losing time; DISSIPATIVE_SCRUB is one that is.
// ---------------------------------------------------------------------------
const BETA_THRESHOLD = 0.05;
const events = [];
let current = null;
for (const r of samples) {
  const hot = Math.abs(r.beta) > BETA_THRESHOLD || Math.abs(r.steerRate) > 3.5;
  if (hot) {
    if (!current) current = { start: r.s, rows: [] };
    current.rows.push(r);
    current.end = r.s;
  } else if (current) {
    events.push(current);
    current = null;
  }
}
if (current) events.push(current);

const scrubEvents = events
  .filter((e) => e.rows.length > 24)
  .map((e) => {
    const rows = e.rows;
    const loss = rows.reduce((s, r) => s + r.dtLoss, 0);
    const terms = [
      ['feedforward', absMean(rows, 'tFeedforward')],
      ['pursuit', absMean(rows, 'tPursuit')],
      ['tracking', absMean(rows, 'tTracking')],
      ['rotation', absMean(rows, 'tRotation')],
      ['slip', absMean(rows, 'tSlip')],
      ['yawError', absMean(rows, 'tYawError')],
      ['damping', absMean(rows, 'tDamping')],
      ['optimizer', absMean(rows, 'tCorrection')],
    ].sort((a, b) => b[1] - a[1]);
    return {
      start: Number(e.start.toFixed(1)),
      end: Number(e.end.toFixed(1)),
      duration: Number(((rows.length / 120)).toFixed(2)),
      samples: rows.length,
      timeLoss: Number(loss.toFixed(3)),
      kind: loss > 0.02 ? 'DISSIPATIVE_SCRUB' : 'USEFUL_ROTATION',
      entrySpeed: Number(rows[0].v.toFixed(2)),
      exitSpeed: Number(rows.at(-1).v.toFixed(2)),
      meanBeta: Number(absMean(rows, 'beta').toFixed(4)),
      peakBeta: Number(peak(rows, 'beta').toFixed(4)),
      meanSteer: Number(absMean(rows, 'steer').toFixed(4)),
      peakSteer: Number(peak(rows, 'steer').toFixed(4)),
      meanSteerRate: Number(absMean(rows, 'steerRate').toFixed(3)),
      meanYawError: Number(absMean(rows, 'yawError').toFixed(4)),
      meanQError: Number(absMean(rows, 'qError').toFixed(3)),
      dominantTerm: terms[0][0],
      dominantTermValue: Number(terms[0][1].toFixed(5)),
      terms: Object.fromEntries(terms.map(([k, v]) => [k, Number(v.toFixed(5))])),
    };
  })
  .sort((a, b) => b.timeLoss - a.timeLoss);

// ---------------------------------------------------------------------------
// §6 Steering decomposition. Compare the servo terms during scrub against the
// same terms when the car is clean, so a term that is simply always large does
// not masquerade as the culprit.
// ---------------------------------------------------------------------------
const scrubRows = samples.filter((r) => Math.abs(r.beta) > BETA_THRESHOLD);
const cleanRows = samples.filter((r) => Math.abs(r.beta) <= BETA_THRESHOLD && r.brake < 0.01);
const termNames = [
  ['feedforward', 'tFeedforward'], ['pursuit', 'tPursuit'], ['tracking', 'tTracking'],
  ['rotation', 'tRotation'], ['slip', 'tSlip'], ['yawError', 'tYawError'],
  ['damping', 'tDamping'], ['optimizer', 'tCorrection'],
];
const steeringTerms = termNames.map(([label, key]) => ({
  term: label,
  meanAbsWhenScrub: Number(absMean(scrubRows, key).toFixed(5)),
  meanAbsWhenClean: Number(absMean(cleanRows, key).toFixed(5)),
  // A term that is elevated during scrub relative to clean running is the
  // candidate mechanism; one that is identical in both is just background.
  elevation: Number((absMean(scrubRows, key) / Math.max(1e-6, absMean(cleanRows, key))).toFixed(2)),
})).sort((a, b) => b.elevation - a.elevation);

// ---------------------------------------------------------------------------
// §10 Feed-forward accuracy. The servo uses atan(L * k) * 0.55. Compare the
// applied feed-forward against the full static relationship and against the
// steering the car actually holds.
// ---------------------------------------------------------------------------
const feedForwardCheck = {
  appliedWeight: 0.55,
  meanAbsAppliedFeedforward: Number(absMean(samples, 'tFeedforward').toFixed(5)),
  meanAbsFullStatic: Number(mean(samples.map((r) => ({ v: Math.abs(Math.atan(2.9 * r.trajCurvature)) })), 'v').toFixed(5)),
  meanAbsActualSteer: Number(absMean(samples, 'steer').toFixed(5)),
  // Ratio of commanded curvature to the curvature the car actually produces:
  // below 1 means the car is under-rotating relative to the plan.
  curvatureTracking: Number(mean(samples.filter((r) => Math.abs(r.trajCurvature) > 0.002)
    .map((r) => ({ v: Math.abs(r.drivenCurvature / r.trajCurvature) })), 'v').toFixed(3)),
};

// ---------------------------------------------------------------------------
// §9 Unwind behaviour at the three regions of interest.
// ---------------------------------------------------------------------------
const REGIONS = [
  { id: 'T1-chicane', from: 789, to: 1014 },
  { id: 's1200-1330', from: 1200, to: 1330 },
  { id: 'final-corner', from: 2560, to: 2705 },
];

const regionReports = REGIONS.map((region) => {
  const rows = samples.filter((r) => r.s >= region.from && r.s <= region.to);
  if (!rows.length) return { id: region.id, empty: true };
  const minSpeedRow = rows.reduce((a, b) => (a.v < b.v ? a : b));
  const maxSteerRow = rows.reduce((a, b) => (Math.abs(a.steer) > Math.abs(b.steer) ? a : b));
  const maxBetaRow = rows.reduce((a, b) => (Math.abs(a.beta) > Math.abs(b.beta) ? a : b));
  const maxYawRow = rows.reduce((a, b) => (Math.abs(a.yawError) > Math.abs(b.yawError) ? a : b));
  // Unwind start: after the steering peak, the first station where |steer|
  // falls below 60% of the peak and keeps falling.
  const peakAbs = Math.abs(maxSteerRow.steer);
  const afterPeak = rows.filter((r) => r.s >= maxSteerRow.s);
  const unwind = afterPeak.find((r) => Math.abs(r.steer) < peakAbs * 0.6);
  // Throttle pickup / full throttle.
  const pickup = rows.find((r) => r.throttle > 0.12 && r.brake < 0.01);
  const full = rows.find((r) => r.throttle > 0.95 && r.brake < 0.01);
  const loss = rows.reduce((s, r) => s + r.dtLoss, 0);
  return {
    id: region.id,
    sFrom: region.from,
    sTo: region.to,
    timeLoss: Number(loss.toFixed(3)),
    entrySpeed: Number(rows[0].v.toFixed(2)),
    minSpeed: Number(minSpeedRow.v.toFixed(2)),
    minSpeedS: Number(minSpeedRow.s.toFixed(1)),
    exitSpeed: Number(rows.at(-1).v.toFixed(2)),
    peakSteerS: Number(maxSteerRow.s.toFixed(1)),
    peakSteer: Number(maxSteerRow.steer.toFixed(4)),
    unwindStartS: unwind ? Number(unwind.s.toFixed(1)) : null,
    unwindDelay: unwind ? Number((unwind.s - minSpeedRow.s).toFixed(1)) : null,
    peakBeta: Number(maxBetaRow.beta.toFixed(4)),
    peakBetaS: Number(maxBetaRow.s.toFixed(1)),
    peakYawError: Number(maxYawRow.yawError.toFixed(4)),
    peakYawErrorS: Number(maxYawRow.s.toFixed(1)),
    meanAbsSteer: Number(absMean(rows, 'steer').toFixed(4)),
    meanAbsBeta: Number(absMean(rows, 'beta').toFixed(4)),
    meanAbsYawError: Number(absMean(rows, 'yawError').toFixed(4)),
    meanAbsQError: Number(absMean(rows, 'qError').toFixed(3)),
    meanReserve: Number(mean(rows, 'reserve').toFixed(3)),
    throttlePickupS: pickup ? Number(pickup.s.toFixed(1)) : null,
    fullThrottleS: full ? Number(full.s.toFixed(1)) : null,
    fullThrottleDelay: full ? Number((full.s - minSpeedRow.s).toFixed(1)) : null,
  };
});

// Station timeline for the primary region (§13 asks for this explicitly).
const timelineFor = (from, to, step = 6) => {
  const rows = samples.filter((r) => r.s >= from && r.s <= to);
  const out = [];
  let last = -Infinity;
  for (const r of rows) {
    if (r.s - last < step) continue;
    last = r.s;
    out.push({
      s: Number(r.s.toFixed(0)),
      v: Number(r.v.toFixed(1)),
      tgt: Number(r.target.toFixed(1)),
      qErr: Number(r.qError.toFixed(2)),
      steer: Number(r.steer.toFixed(3)),
      steerRate: Number(r.steerRate.toFixed(1)),
      beta: Number(r.beta.toFixed(3)),
      yawErr: Number(r.yawError.toFixed(3)),
      brake: Number(r.brake.toFixed(2)),
      thr: Number(r.throttle.toFixed(2)),
      ff: Number(r.tFeedforward.toFixed(3)),
      trk: Number(r.tTracking.toFixed(3)),
      rot: Number(r.tRotation.toFixed(3)),
      slip: Number(r.tSlip.toFixed(3)),
    });
  }
  return out;
};

const report = {
  startingSha: '8093dd3',
  laps: laps.map((i) => ({ lap: i.lap, time: Number(i.time.toFixed(3)), valid: i.valid })),
  scrubEventCount: scrubEvents.length,
  dissipationTotal: Number(scrubEvents.filter((e) => e.kind === 'DISSIPATIVE_SCRUB')
    .reduce((s, e) => s + e.timeLoss, 0).toFixed(3)),

  steeringTerms,
  feedForwardCheck,
  regions: regionReports,
  worstScrubEvents: scrubEvents.slice(0, 10),
  timelines: {
    T1: timelineFor(789, 1014, 8),
    finalCorner: timelineFor(2560, 2705, 8),
  },
};

console.log(JSON.stringify(report, null, 2));
