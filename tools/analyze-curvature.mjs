/**
 * Curvature / geometry discrimination.
 *
 * The previous stage left one question open and it decides where the next fix
 * belongs:
 *
 *   Is the planned geometry wrong (PLAN_GEOMETRY), is the servo making the car
 *   trace the wrong geometry (SERVO_LATERAL), or is the chassis merely yawing
 *   and sliding more than the velocity path itself curves (TRANSIENT_DYNAMICS)?
 *
 * The single global ratio mean(|k_yaw / k_plan|) = 1.559 cannot answer that: it
 * is biased by small denominators, curvature sign reversals and sideslip, and
 * it compares chassis rotation rate against planned path curvature -- two
 * different physical quantities.
 *
 * So this tool computes three distinct curvatures:
 *
 *   k_plan         planned trajectory curvature at the aligned station
 *   k_yaw          chassis rotation rate / speed  (what the previous statistic used)
 *   k_velocityPath Menger curvature of the actual world-space path (what the
 *                  tyre actually has to produce)
 *
 * and then buckets the mismatch against lateral error. If excess curvature
 * survives when the car is on the planned line, the plan's curvature field is
 * wrong. If it vanishes when q error vanishes, the servo is cutting the path.
 * If k_yaw is high but k_velocityPath is not, the car is sliding, not turning
 * tighter.
 *
 *   node tools/analyze-curvature.mjs
 *   node tools/analyze-curvature.mjs --laps 4
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
session.laps = number('--laps', 4);
session.autopilot = true;
session.start({ freshTrack: true });

const driver = session.drivers[0];
const zero = { steer: 0, throttle: 0, brake: 0 };

const samples = [];
let previousLap = session.player.race.lap;
let lapValid = true;
const laps = [];

for (let step = 0; step < 120 * 800; step++) {
  const priorLap = session.player.race.lap;
  const priorValid = session.player.race.valid;
  session.step(1 / 120, zero);
  const car = session.player;
  if (session.phase !== 'racing') continue;
  if (car.race.lap !== previousLap) {
    laps.push({ lap: priorLap, time: car.race.lastLap, valid: lapValid && priorValid });
    previousLap = car.race.lap;
    lapValid = car.race.valid;
    if (laps.length >= number('--laps', 4)) break;
    continue;
  }
  if (!car.race.valid) lapValid = false;

  const l = driver.limiter ?? {};
  const servo = l.servo ?? {};
  const planPoint = driver.planner.at(car.s);
  const wheels = car.wheels ?? [];

  // §5 Geometry alignment: find the station on the planned trajectory that is
  // spatially nearest to the car. Comparing the plan at the car's centreline
  // station to the car's own curvature compares two different geometric
  // locations whenever the car is metres off the line.
  let sPlan = car.s;
  let bestD2 = Infinity;
  const planPts = driver.planner.plan?.points ?? [];
  for (const p of planPts) {
    const d2 = (p.x - car.x) ** 2 + (p.z - car.z) ** 2;
    if (d2 < bestD2) { bestD2 = d2; sPlan = p.s; }
  }

  samples.push({
    s: car.s,
    sPlan,
    alignDs: wrap(sPlan - car.s + trackLength / 2, trackLength) - trackLength / 2,
    x: car.x,
    z: car.z,
    lap: car.race.lap,
    v: car.speed,
    target: l.targetSpeed ?? car.speed,
    lateral: car.lateral,
    q: planPoint.offset,
    qError: car.lateral - planPoint.offset,
    kPlan: planPoint.curvature ?? 0,
    kYaw: car.speed > 2 ? car.yawRate / car.speed : 0,
    kVel: 0,
    steer: l.steer ?? car.controls.steer,
    beta: l.beta ?? 0,
    yawRate: l.yawRate ?? 0,
    yawError: l.yawError ?? 0,
    throttle: l.throttle ?? car.controls.throttle,
    brake: l.brakeCmd ?? car.controls.brake,
    gripUtil: l.gripUtil ?? 0,
    reserve: l.reserve ?? 0,
    ax: car.ax,
    ay: car.ay,
    frontFy: wheels.length === 4 ? wheels[0].tyre.fy + wheels[1].tyre.fy : 0,
    rearFy: wheels.length === 4 ? wheels[2].tyre.fy + wheels[3].tyre.fy : 0,
    tPursuit: servo.pursuit ?? 0,
    tSlip: servo.slip ?? 0,
    tTracking: servo.tracking ?? 0,
    tRotation: servo.rotation ?? 0,
    tFeedforward: servo.feedforward ?? 0,
    tYawError: servo.yawError ?? 0,
  });
}

// ---------------------------------------------------------------------------
// §6 Kinematic path curvature: the Menger (circumcircle) curvature of three
// actual world positions. This is the curvature the tyre must physically
// produce, independent of how much the chassis is yawing.
// ---------------------------------------------------------------------------
const STENCIL = 6;
for (let i = STENCIL; i < samples.length - STENCIL; i++) {
  const a = samples[i - STENCIL], b = samples[i], c = samples[i + STENCIL];
  const ab = Math.hypot(b.x - a.x, b.z - a.z);
  const bc = Math.hypot(c.x - b.x, c.z - b.z);
  const ca = Math.hypot(a.x - c.x, a.z - c.z);
  const cross = (b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x);
  const denom = ab * bc * ca;
  b.kVel = denom > 1e-6 ? (2 * cross) / denom : 0;
}

// Causal time loss, using deficit growth for cause and time for magnitude (§9).
for (let i = 1; i < samples.length; i++) {
  const a = samples[i - 1], b = samples[i];
  const ds = wrap(b.s - a.s, trackLength);
  b.ds = (ds > 0 && ds < 30) ? ds : 0;
  b.dtRegional = b.ds > 0 ? Math.max(0, b.ds / Math.max(2, b.v) - b.ds / Math.max(2, b.target)) : 0;
  const growth = (b.target - b.v) - (a.target - a.v);
  b.causalLoss = growth > 0 ? b.dtRegional : 0;
}
samples[0].ds = 0; samples[0].dtRegional = 0; samples[0].causalLoss = 0;

const valid = laps.filter((i) => i.valid && i.time > 20);
if (!valid.length) { console.error('no valid lap recorded'); process.exit(1); }

const mean = (rows, f) => (rows.length ? rows.reduce((s, r) => s + f(r), 0) / rows.length : 0);
const quant = (rows, f, p) => {
  if (!rows.length) return 0;
  const v = rows.map(f).sort((a, b) => a - b);
  return v[Math.min(v.length - 1, Math.floor(v.length * p))];
};
const absMean = (rows, f) => mean(rows, (r) => Math.abs(f(r)));

// Only samples where the plan has real curvature: tiny denominators dominate a
// naive mean ratio and inflate it badly (§4).
const curved = samples.filter((r) => Math.abs(r.kPlan) > 0.0015 && r.v > 3);

const ratio = (f) => curved.map((r) => Math.abs(f(r) / r.kPlan));

// §4 robust ratio statistics and denominator bins.
const ratioStats = (f) => {
  const rr = ratio(f);
  return {
    n: rr.length,
    p25: Number(quant(rr.map((v) => ({ v })), (r) => r.v, 0.25).toFixed(3)),
    p50: Number(quant(rr.map((v) => ({ v })), (r) => r.v, 0.5).toFixed(3)),
    p75: Number(quant(rr.map((v) => ({ v })), (r) => r.v, 0.75).toFixed(3)),
    p90: Number(quant(rr.map((v) => ({ v })), (r) => r.v, 0.9).toFixed(3)),
    p95: Number(quant(rr.map((v) => ({ v })), (r) => r.v, 0.95).toFixed(3)),
    mean: Number(mean(rr.map((v) => ({ v })), (r) => r.v).toFixed(3)),
  };
};

const BINS = [
  { id: '0.002-0.005', lo: 0.0015, hi: 0.005 },
  { id: '0.005-0.010', lo: 0.005, hi: 0.010 },
  { id: '0.010-0.020', lo: 0.010, hi: 0.020 },
  { id: '>0.020', lo: 0.020, hi: Infinity },
];
const denominatorBins = BINS.map((bin) => {
  const rows = curved.filter((r) => Math.abs(r.kPlan) >= bin.lo && Math.abs(r.kPlan) < bin.hi);
  return {
    bin: bin.id,
    n: rows.length,
    medianKVelRatio: Number(quant(rows, (r) => Math.abs(r.kVel / r.kPlan), 0.5).toFixed(3)),
    medianKYawRatio: Number(quant(rows, (r) => Math.abs(r.kYaw / r.kPlan), 0.5).toFixed(3)),
    meanBeta: Number(absMean(rows, (r) => r.beta).toFixed(4)),
    meanSpeed: Number(mean(rows, (r) => r.v).toFixed(1)),
  };
});

// Denominator-robust regression k_driven = a*k_plan + b (§4).
const regress = (f) => {
  const n = curved.length;
  const sx = mean(curved, (r) => r.kPlan), sy = mean(curved, (r) => f(r));
  let sxy = 0, sxx = 0, syy = 0;
  for (const r of curved) { const dx = r.kPlan - sx, dy = f(r) - sy; sxy += dx * dy; sxx += dx * dx; syy += dy * dy; }
  const a = sxx > 0 ? sxy / sxx : 0;
  const b = sy - a * sx;
  const r2 = syy > 0 ? (sxy * sxy) / (sxx * syy) : 0;
  return { slope: Number(a.toFixed(3)), intercept: Number(b.toFixed(5)), r2: Number(r2.toFixed(3)), n };
};

// §8 THE DISCRIMINATOR: mismatch against lateral error.
const Q_BUCKETS = [
  { id: '0.00-0.25', lo: 0, hi: 0.25 },
  { id: '0.25-0.50', lo: 0.25, hi: 0.5 },
  { id: '0.50-1.00', lo: 0.5, hi: 1.0 },
  { id: '1.00-1.50', lo: 1.0, hi: 1.5 },
  { id: '1.50-2.00', lo: 1.5, hi: 2.0 },
  { id: '>2.00', lo: 2.0, hi: Infinity },
];
const qErrorBuckets = Q_BUCKETS.map((b) => {
  const rows = curved.filter((r) => Math.abs(r.qError) >= b.lo && Math.abs(r.qError) < b.hi);
  return {
    bucket: b.id,
    n: rows.length,
    medianKVelRatio: Number(quant(rows, (r) => Math.abs(r.kVel / r.kPlan), 0.5).toFixed(3)),
    medianKYawRatio: Number(quant(rows, (r) => Math.abs(r.kYaw / r.kPlan), 0.5).toFixed(3)),
    meanBeta: Number(absMean(rows, (r) => r.beta).toFixed(4)),
    meanSpeed: Number(mean(rows, (r) => r.v).toFixed(1)),
    meanSteer: Number(absMean(rows, (r) => r.steer).toFixed(4)),
    scrubLoss: Number(rows.reduce((s, r) => s + r.causalLoss, 0).toFixed(3)),
  };
});

// §7 Per-corner audit. Corners are contiguous runs of meaningful planned
// curvature, merged when separated by less than a short straight.
const corners = [];
let run = null;
for (const r of samples) {
  const hot = Math.abs(r.kPlan) > 0.003;
  if (hot) {
    if (!run) run = { from: r.s, rows: [] };
    run.rows.push(r); run.to = r.s;
  } else if (run && r.s - run.to > 25) {
    corners.push(run); run = null;
  }
}
if (run) corners.push(run);

const cornerReports = corners
  .filter((c) => c.rows.length > 40)
  .map((c) => {
    const rows = c.rows;
    const sumAbs = (f) => rows.reduce((s, r) => s + Math.abs(f(r)), 0);
    const ratioSum = sumAbs((r) => r.kVel) / Math.max(1e-6, sumAbs((r) => r.kPlan));
    const ratioYawSum = sumAbs((r) => r.kYaw) / Math.max(1e-6, sumAbs((r) => r.kPlan));
    const minRow = rows.reduce((a, b) => (a.v < b.v ? a : b));
    const maxBetaRow = rows.reduce((a, b) => (Math.abs(a.beta) > Math.abs(b.beta) ? a : b));
    const qErr = absMean(rows, (r) => r.qError);
    // §7 classification. The discriminator is whether the driven path is
    // tighter when the car is already on the line.
    const onLine = rows.filter((r) => Math.abs(r.qError) < 0.5);
    const onLineRatio = onLine.length > 20
      ? quant(onLine, (r) => Math.abs(r.kVel / r.kPlan), 0.5) : null;
    let classification = 'UNKNOWN';
    if (onLineRatio !== null) {
      if (onLineRatio > 1.15) classification = 'PLAN_GEOMETRY';
      else if (qErr > 1.0 && ratioSum > 1.15) classification = 'SERVO_LATERAL';
      else if (absMean(rows, (r) => r.kYaw) / Math.max(1e-6, sumAbs((r) => r.kPlan) / rows.length) > 1.4
        && quant(rows, (r) => Math.abs(r.kVel / r.kPlan), 0.5) < 1.15) classification = 'TRANSIENT_DYNAMICS';
      else if (ratioSum < 1.12 && qErr < 0.8) classification = 'NECESSARY';
      else classification = 'SERVO_LATERAL';
    }
    return {
      sFrom: Number(c.from.toFixed(0)),
      sTo: Number(c.to.toFixed(0)),
      samples: rows.length,
      sumKPlan: Number(sumAbs((r) => r.kPlan).toFixed(4)),
      robustKVelRatio: Number(ratioSum.toFixed(3)),
      robustKYawRatio: Number(ratioYawSum.toFixed(3)),
      onLineKVelRatio: onLineRatio === null ? null : Number(onLineRatio.toFixed(3)),
      onLineSamples: onLine.length,
      meanQError: Number(qErr.toFixed(3)),
      peakQError: Number(Math.max(...rows.map((r) => Math.abs(r.qError))).toFixed(3)),
      entrySpeed: Number(rows[0].v.toFixed(2)),
      minSpeed: Number(minRow.v.toFixed(2)),
      exitSpeed: Number(rows.at(-1).v.toFixed(2)),
      meanBeta: Number(absMean(rows, (r) => r.beta).toFixed(4)),
      peakBeta: Number(Math.abs(maxBetaRow.beta).toFixed(4)),
      regionalDeficit: Number(rows.reduce((s, r) => s + r.dtRegional, 0).toFixed(3)),
      causalLoss: Number(rows.reduce((s, r) => s + r.causalLoss, 0).toFixed(3)),
      meanTracking: Number(absMean(rows, (r) => r.tTracking).toFixed(5)),
      meanPursuit: Number(absMean(rows, (r) => r.tPursuit).toFixed(5)),
      meanSlip: Number(absMean(rows, (r) => r.tSlip).toFixed(5)),
      meanFeedforward: Number(absMean(rows, (r) => r.tFeedforward).toFixed(5)),
      classification,
    };
  })
  .sort((a, b) => b.causalLoss - a.causalLoss);

// §10 Braking classification.
const brakeEvents = [];
let br = null;
for (const r of samples) {
  if (r.brake > 0.02) {
    if (!br) br = { rows: [] };
    br.rows.push(r);
  } else if (br) {
    brakeEvents.push(br);
    br = null;
  }
}
const brakeReport = brakeEvents
  .filter((e) => e.rows.length > 15)
  .map((e) => {
    const rows = e.rows;
    const entry = rows[0], peakRow = rows.reduce((a, b) => (a.brake > b.brake ? a : b));
    // Where the plan's speed first falls below the speed the car had when it
    // began braking. Braking meaningfully before that is EARLY_START.
    const startSpeed = entry.v;
    const later = samples.filter((r) => r.s > entry.s && r.s < entry.s + 260);
    const planCross = later.find((r) => r.target < startSpeed - 1);
    const earlyBy = planCross ? planCross.s - entry.s : 0;
    const release = rows.at(-1);
    const growth = rows.reduce((s, r) => s + r.causalLoss, 0);
    let cause = 'NECESSARY';
    if (earlyBy > 12) cause = 'EARLY_START';
    else if (release.throttle < 0.2 && release.v < release.target - 1.5) cause = 'LATE_RELEASE';
    else if (peakRow.brake > 0.9 && rows.reduce((s, r) => s + r.dtRegional, 0) > 0.06) cause = 'EXCESS_PEAK';
    else if (growth > 0.02) cause = 'TARGET_TOO_LOW';
    return {
      sFrom: Number(entry.s.toFixed(0)),
      sTo: Number(release.s.toFixed(0)),
      entrySpeed: Number(entry.v.toFixed(2)),
      peakBrake: Number(peakRow.brake.toFixed(2)),
      peakBrakeS: Number(peakRow.s.toFixed(0)),
      exitSpeed: Number(release.v.toFixed(2)),
      targetAtEntry: Number(entry.target.toFixed(2)),
      plannedCrossS: planCross ? Number(planCross.s.toFixed(0)) : null,
      earlyBy: Number(earlyBy.toFixed(1)),
      causalLoss: Number(growth.toFixed(3)),
      cause,
    };
  })
  .sort((a, b) => b.causalLoss - a.causalLoss);

const brakeByCause = {};
for (const e of brakeReport) brakeByCause[e.cause] = Number(((brakeByCause[e.cause] ?? 0) + e.causalLoss).toFixed(3));

// §18 tyre-state / lap sensitivity.
const perLap = [...new Set(samples.map((r) => r.lap))].sort((a, b) => a - b).map((lap) => {
  const rows = curved.filter((r) => r.lap === lap);
  return {
    lap,
    samples: rows.length,
    medianKVelRatio: Number(quant(rows, (r) => Math.abs(r.kVel / r.kPlan), 0.5).toFixed(3)),
    meanQError: Number(absMean(rows, (r) => r.qError).toFixed(3)),
    meanBeta: Number(absMean(rows, (r) => r.beta).toFixed(4)),
    causalLoss: Number(samples.filter((r) => r.lap === lap).reduce((s, r) => s + r.causalLoss, 0).toFixed(3)),
  };
});

// §17 regions. §9: report regional deficit AND causal loss separately.
const REGIONS = [
  { id: 'T1-chicane', from: 789, to: 1014 },
  { id: 's1200-1330', from: 1200, to: 1330 },
  { id: 'final-corner', from: 2540, to: 2705 },
];
const regions = REGIONS.map((reg) => {
  // §17B: only a clean traversal. A recovery-contaminated sample must not
  // produce final-corner conclusions.
  const rows = samples.filter((r) => r.s >= reg.from && r.s <= reg.to && Math.abs(r.lateral) < track.halfWidth + 0.2);
  if (!rows.length) return { id: reg.id, empty: true };
  return {
    id: reg.id,
    sFrom: reg.from,
    sTo: reg.to,
    samples: rows.length,
    regionalDeficit: Number(rows.reduce((s, r) => s + r.dtRegional, 0).toFixed(3)),
    causalLoss: Number(rows.reduce((s, r) => s + r.causalLoss, 0).toFixed(3)),
    medianKVelRatio: Number(quant(rows.filter((r) => Math.abs(r.kPlan) > 0.0015), (r) => Math.abs(r.kVel / r.kPlan), 0.5).toFixed(3)),
    medianKYawRatio: Number(quant(rows.filter((r) => Math.abs(r.kPlan) > 0.0015), (r) => Math.abs(r.kYaw / r.kPlan), 0.5).toFixed(3)),
    meanQError: Number(absMean(rows, (r) => r.qError).toFixed(3)),
    meanBeta: Number(absMean(rows, (r) => r.beta).toFixed(4)),
    meanAlignDs: Number(mean(rows, (r) => r.alignDs).toFixed(2)),
  };
});

const report = {
  startingSha: '125a052',
  laps: laps.map((i) => ({ lap: i.lap, time: Number(i.time.toFixed(3)), valid: i.valid })),
  curvedSamples: curved.length,

  robustCurvature: {
    kVelOverPlan: ratioStats((r) => r.kVel),
    kYawOverPlan: ratioStats((r) => r.kYaw),
    regressionKVel: regress((r) => r.kVel),
    regressionKYaw: regress((r) => r.kYaw),
    denominatorBins,
    sumRatioKVel: Number((curved.reduce((s, r) => s + Math.abs(r.kVel), 0)
      / Math.max(1e-6, curved.reduce((s, r) => s + Math.abs(r.kPlan), 0))).toFixed(3)),
    sumRatioKYaw: Number((curved.reduce((s, r) => s + Math.abs(r.kYaw), 0)
      / Math.max(1e-6, curved.reduce((s, r) => s + Math.abs(r.kPlan), 0))).toFixed(3)),
    alignment: {
      meanAbsAlignDs: Number(absMean(curved, (r) => r.alignDs).toFixed(2)),
      p95AbsAlignDs: Number(quant(curved.map((r) => ({ v: Math.abs(r.alignDs) })), (r) => r.v, 0.95).toFixed(2)),
    },
  },

  qErrorBuckets,
  corners: cornerReports.slice(0, 12),
  braking: { byCause: brakeByCause, events: brakeReport.slice(0, 10) },
  perLap,
  regions,
};

console.log(JSON.stringify(report, null, 2));
