/**
 * Lateral-departure initiation analysis.
 *
 * The previous block established the causal chain:
 *
 *   car loses the line -> controller recovers aggressively -> driven path bends
 *   harder -> chassis yaws and slides more than the path curves -> tyre energy
 *   spent laterally -> speed lost
 *
 * It deliberately analysed samples where |q error| > 2 m. By then the
 * controller is already reacting to a failure that has already happened. This
 * tool looks earlier, at the first divergence, and asks which control term
 * leads it.
 *
 * It delivers:
 *   §3   clean-only vs all-lap normalisation for both q-error and braking;
 *   §5   LateralDepartureEvent detection with the pre-history retained;
 *   §6   lead/lag of every steering term against q-error growth;
 *   §7/§8 pure-pursuit chord error -- does the chord lie inside the plan before
 *         the error develops;
 *   §11  whether departures cluster on replans;
 *   §12  measured steering actuator lag against the assumed 0.14 s;
 *   §17  stored-vs-reconstructed curvature field, at several resolutions.
 *
 *   node tools/analyze-departure.mjs
 *   node tools/analyze-departure.mjs --laps 5
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
session.laps = number('--laps', 5);
session.autopilot = true;
session.start({ freshTrack: true });

const driver = session.drivers[0];
const atlas = driver.atlas;
const zero = { steer: 0, throttle: 0, brake: 0 };

const samples = [];
let previousLap = session.player.race.lap;
let lapValid = true;
const laps = [];

for (let step = 0; step < 120 * 900; step++) {
  const priorLap = session.player.race.lap;
  const priorValid = session.player.race.valid;
  session.step(1 / 120, zero);
  const car = session.player;
  if (session.phase !== 'racing') continue;
  if (car.race.lap !== previousLap) {
    laps.push({ lap: priorLap, time: car.race.lastLap, valid: lapValid && priorValid });
    previousLap = car.race.lap;
    lapValid = car.race.valid;
    if (laps.length >= number('--laps', 5)) break;
    continue;
  }
  if (!car.race.valid) lapValid = false;

  const l = driver.limiter ?? {};
  const servo = l.servo ?? {};
  const planPoint = driver.planner.at(car.s);
  const lookahead = l.lookahead ?? 12;
  const target = driver.planner.at(car.s + lookahead);

  // §8 Pure-pursuit chord. The servo aims at a point `lookahead` metres ahead;
  // the straight line from the car to that point is the chord it is actually
  // steering toward. Where that chord sits relative to the planned path at the
  // midpoint is the geometric bias pure pursuit introduces for free.
  const mx = (car.x + target.x) * 0.5, mz = (car.z + target.z) * 0.5;
  const mid = track.nearest(mx, mz);
  const qPlanAtMid = driver.planner.at(mid.s).offset;

  samples.push({
    s: car.s, lap: car.race.lap, x: car.x, z: car.z,
    v: car.speed,
    target: l.targetSpeed ?? car.speed,
    lateral: car.lateral,
    q: planPoint.offset,
    qError: car.lateral - planPoint.offset,
    kPlan: planPoint.curvature ?? 0,
    kYaw: car.speed > 2 ? car.yawRate / car.speed : 0,
    kVel: 0,
    steer: l.steer ?? car.controls.steer,
    steerActual: (car.steering ?? 0) / Math.max(0.05, car.spec?.steeringLock ?? 0.48),
    beta: l.beta ?? 0,
    yawRate: l.yawRate ?? 0,
    yawError: l.yawError ?? 0,
    throttle: l.throttle ?? car.controls.throttle,
    brake: l.brakeCmd ?? car.controls.brake,
    reserve: l.reserve ?? 0,
    planClock: driver.planClock ?? 0,
    lookahead,
    chordError: mid.lateral - qPlanAtMid,
    tPursuit: servo.pursuit ?? 0,
    tSlip: servo.slip ?? 0,
    tTracking: servo.tracking ?? 0,
    tDamping: servo.damping ?? 0,
    tRotation: servo.rotation ?? 0,
    tFeedforward: servo.feedforward ?? 0,
    tYawError: servo.yawError ?? 0,
    tCorrection: servo.correction ?? 0,
    localX: servo.localX ?? 0,
  });
}

// §6 kinematic path curvature (Menger), and q-error derivatives.
const STENCIL = 6;
for (let i = STENCIL; i < samples.length - STENCIL; i++) {
  const a = samples[i - STENCIL], b = samples[i], c = samples[i + STENCIL];
  const ab = Math.hypot(b.x - a.x, b.z - a.z);
  const bc = Math.hypot(c.x - b.x, c.z - b.z);
  const ca = Math.hypot(a.x - c.x, a.z - c.z);
  const cross = (b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x);
  b.kVel = ab * bc * ca > 1e-6 ? (2 * cross) / (ab * bc * ca) : 0;
}
for (let i = 1; i < samples.length; i++) {
  const a = samples[i - 1], b = samples[i];
  const ds = wrap(b.s - a.s, trackLength);
  b.ds = (ds > 0 && ds < 30) ? ds : 0;
  b.dtRegional = b.ds > 0 ? Math.max(0, b.ds / Math.max(2, b.v) - b.ds / Math.max(2, b.target)) : 0;
  b.growth = ((b.target - b.v) - (a.target - a.v));
  b.causalLoss = b.growth > 0 ? b.dtRegional : 0;
  b.qDot = (b.qError - a.qError) * 120;
}
samples[0].ds = 0; samples[0].dtRegional = 0; samples[0].causalLoss = 0; samples[0].qDot = 0;
for (let i = 1; i < samples.length - 1; i++) samples[i].qDotDot = (samples[i + 1].qDot - samples[i - 1].qDot) * 60;
samples[0].qDotDot = 0; samples.at(-1).qDotDot = 0;

const validLaps = new Set(laps.filter((i) => i.valid && i.time > 20).map((i) => i.lap));
const cleanSamples = samples.filter((r) => validLaps.has(r.lap));
if (!validLaps.size) { console.error('no valid lap recorded'); process.exit(1); }

const mean = (rows, f) => (rows.length ? rows.reduce((s, r) => s + f(r), 0) / rows.length : 0);
const quant = (rows, f, p) => {
  if (!rows.length) return 0;
  const v = rows.map(f).sort((a, b) => a - b);
  return v[Math.min(v.length - 1, Math.floor(v.length * p))];
};
const absMean = (rows, f) => mean(rows, (r) => Math.abs(f(r)));

// ---------------------------------------------------------------------------
// §3A q-error buckets, clean and all-lap, kept strictly separate.
// ---------------------------------------------------------------------------
const Q_BUCKETS = [
  { id: '0.00-0.25', lo: 0, hi: 0.25 }, { id: '0.25-0.50', lo: 0.25, hi: 0.5 },
  { id: '0.50-1.00', lo: 0.5, hi: 1.0 }, { id: '1.00-1.50', lo: 1.0, hi: 1.5 },
  { id: '1.50-2.00', lo: 1.5, hi: 2.0 }, { id: '>2.00', lo: 2.0, hi: Infinity },
];
const bucketTable = (rows) => {
  const curved = rows.filter((r) => Math.abs(r.kPlan) > 0.0015 && r.v > 3);
  return Q_BUCKETS.map((b) => {
    const set = curved.filter((r) => Math.abs(r.qError) >= b.lo && Math.abs(r.qError) < b.hi);
    return {
      bucket: b.id, n: set.length,
      kVelRatio: Number(quant(set, (r) => Math.abs(r.kVel / r.kPlan), 0.5).toFixed(3)),
      kYawRatio: Number(quant(set, (r) => Math.abs(r.kYaw / r.kPlan), 0.5).toFixed(3)),
      meanBeta: Number(absMean(set, (r) => r.beta).toFixed(4)),
      meanChordError: Number(absMean(set, (r) => r.chordError).toFixed(3)),
      scrubLoss: Number(rows.filter((r) => Math.abs(r.qError) >= b.lo && Math.abs(r.qError) < b.hi)
        .reduce((s, r) => s + r.causalLoss, 0).toFixed(3)),
    };
  });
};

// §23 line-holding statistics.
const lineHolding = (rows) => {
  const curved = rows.filter((r) => Math.abs(r.kPlan) > 0.0015 && r.v > 3);
  return {
    samples: curved.length,
    fracUnder05: Number((curved.filter((r) => Math.abs(r.qError) < 0.5).length / Math.max(1, curved.length)).toFixed(3)),
    fracUnder10: Number((curved.filter((r) => Math.abs(r.qError) < 1.0).length / Math.max(1, curved.length)).toFixed(3)),
    fracOver20: Number((curved.filter((r) => Math.abs(r.qError) > 2.0).length / Math.max(1, curved.length)).toFixed(3)),
    meanAbsQError: Number(absMean(curved, (r) => r.qError).toFixed(3)),
    p95AbsQError: Number(quant(curved.map((r) => ({ v: Math.abs(r.qError) })), (r) => r.v, 0.95).toFixed(3)),
  };
};

// ---------------------------------------------------------------------------
// §5 Lateral departure events, with pre-history retained so §6 can ask which
// term was already moving before the error existed.
// ---------------------------------------------------------------------------
const CROSSINGS = [0.25, 0.5, 1.0, 2.0];
const events = [];
const armed = new Set();
for (let i = 1; i < samples.length; i++) {
  const e = Math.abs(samples[i].qError);
  const prev = Math.abs(samples[i - 1].qError);
  for (const level of CROSSINGS) {
    if (prev < level && e >= level && !armed.has(level)) {
      armed.add(level);
      if (level === 0.5) events.push({ onset: i, level });
    }
  }
  if (e < 0.25) armed.clear();
}

const TERMS = [
  ['pursuit', 'tPursuit'], ['feedforward', 'tFeedforward'], ['tracking', 'tTracking'],
  ['rotation', 'tRotation'], ['slip', 'tSlip'], ['yawError', 'tYawError'],
  ['damping', 'tDamping'], ['optimizer', 'tCorrection'],
];

// §6 Lead/lag: correlate each term against q-error growth over a lag window.
// A term whose correlation peaks at a NEGATIVE lag (term leads growth) is a
// candidate initiator; a positive-lag peak is a responder.
const leadLag = (rows) => {
  const growth = rows.map((r) => Math.max(0, r.qDot));
  const out = {};
  for (const [label, key] of TERMS) {
    let best = { lag: 0, corr: -Infinity };
    for (let lag = -60; lag <= 60; lag += 6) {
      let n = 0, sx = 0, sy = 0, sxy = 0, sxx = 0, syy = 0;
      for (let i = 0; i < rows.length; i++) {
        const j = i - lag;
        if (j < 0 || j >= rows.length) continue;
        const x = Math.abs(rows[j][key]), y = growth[i];
        n++; sx += x; sy += y; sxy += x * y; sxx += x * x; syy += y * y;
      }
      if (n < 40) continue;
      const denom = Math.sqrt(Math.max(1e-9, (n * sxx - sx * sx) * (n * syy - sy * sy)));
      const c = (n * sxy - sx * sy) / denom;
      if (c > best.corr) best = { lag, corr: c };
    }
    out[label] = { bestLagSamples: best.lag, bestLagMs: Number((best.lag * 1000 / 120).toFixed(0)), correlation: Number(best.corr.toFixed(3)) };
  }
  return out;
};

const departureEvents = events.slice(0, 60).map((ev) => {
  const from = Math.max(0, ev.onset - 240);           // 2.0 s before
  let peakIdx = ev.onset;
  for (let i = ev.onset; i < Math.min(samples.length, ev.onset + 240); i++) {
    if (Math.abs(samples[i].qError) > Math.abs(samples[peakIdx].qError)) peakIdx = i;
  }
  const to = Math.min(samples.length, peakIdx + 240);
  const rows = samples.slice(from, to);
  const before = samples.slice(from, ev.onset);
  const after = samples.slice(ev.onset, to);
  // Which term is already elevated BEFORE the crossing and keeps rising?
  const initiator = TERMS.map(([label, key]) => ({
    label,
    before: Number(absMean(before, (r) => r[key]).toFixed(5)),
    after: Number(absMean(after, (r) => r[key]).toFixed(5)),
  })).map((t) => ({ ...t, growth: Number((t.after / Math.max(1e-6, t.before)).toFixed(2)) }))
    .sort((a, b) => b.growth - a.growth);
  return {
    lap: samples[ev.onset].lap,
    onsetS: Number(samples[ev.onset].s.toFixed(1)),
    peakS: Number(samples[peakIdx].s.toFixed(1)),
    peakQError: Number(Math.abs(samples[peakIdx].qError).toFixed(2)),
    samplesBefore: before.length,
    speedAtOnset: Number(samples[ev.onset].v.toFixed(1)),
    chordErrorBefore: Number(absMean(before, (r) => r.chordError).toFixed(3)),
    chordErrorAfter: Number(absMean(after, (r) => r.chordError).toFixed(3)),
    planClockAtOnsetMs: Number((samples[ev.onset].planClock * 1000).toFixed(0)),
    causalLoss: Number(rows.reduce((s, r) => s + r.causalLoss, 0).toFixed(3)),
    firstRisingTerm: initiator[0].label,
    firstRisingRatio: initiator[0].growth,
    termGrowth: Object.fromEntries(initiator.map((t) => [t.label, t.growth])),
  };
});

const globalLeadLag = leadLag(cleanSamples.filter((r) => Math.abs(r.qError) > 0.15));

// §11 Replan clustering: is q-error onset more likely right after a replan?
const replanAnalysis = (() => {
  const windows = [20, 40, 80, 150];
  const counts = Object.fromEntries(windows.map((w) => [w, 0]));
  for (const ev of departureEvents) {
    for (const w of windows) if (ev.planClockAtOnsetMs <= w) counts[w]++;
  }
  // Exposure: what fraction of ALL curved samples fall in each window?
  const curved = cleanSamples.filter((r) => Math.abs(r.kPlan) > 0.0015);
  const exposure = Object.fromEntries(windows.map((w) => [
    w, Number((curved.filter((r) => r.planClock * 1000 <= w).length / Math.max(1, curved.length)).toFixed(3)),
  ]));
  return {
    departures: departureEvents.length,
    withinMs: counts,
    exposureFraction: exposure,
    // If departure probability >> exposure, replans are implicated.
    enrichment: Object.fromEntries(windows.map((w) => [
      w, Number((counts[w] / Math.max(1, departureEvents.length) / Math.max(0.01, exposure[w])).toFixed(2)),
    ])),
  };
})();

// §12 Actuator lag: cross-correlate commanded steering with actual steering.
const actuatorLag = (() => {
  const bins = [
    { id: '0-20', lo: 0, hi: 20 }, { id: '20-40', lo: 20, hi: 40 },
    { id: '40-60', lo: 40, hi: 60 }, { id: '>60', lo: 60, hi: Infinity },
  ];
  return bins.map((b) => {
    const rows = cleanSamples.filter((r) => r.v >= b.lo && r.v < b.hi && Math.abs(r.steer) > 0.02);
    let best = { lag: 0, corr: -Infinity };
    for (let lag = -4; lag <= 40; lag++) {
      let n = 0, sx = 0, sy = 0, sxy = 0, sxx = 0, syy = 0;
      for (let i = 0; i < rows.length; i++) {
        const j = i + lag;
        if (j < 0 || j >= rows.length) continue;
        const x = rows[i].steer, y = rows[j].steerActual;
        n++; sx += x; sy += y; sxy += x * y; sxx += x * x; syy += y * y;
      }
      if (n < 60) continue;
      const denom = Math.sqrt(Math.max(1e-9, (n * sxx - sx * sx) * (n * syy - sy * sy)));
      const c = (n * sxy - sx * sy) / denom;
      if (c > best.corr) best = { lag, corr: c };
    }
    return {
      speedBin: b.id, samples: rows.length,
      measuredLagSamples: best.lag,
      measuredLagSeconds: Number((best.lag / 120).toFixed(4)),
      correlation: Number(best.corr.toFixed(3)),
    };
  });
})();

// §17 Stored vs reconstructed curvature of the planned Q0 path.
const curvatureFieldAudit = (() => {
  const spacings = [0.25, 0.5, 1.0, 2.0, 4.0];
  const results = {};
  for (const h of spacings) {
    const n = Math.max(60, Math.round(trackLength / h));
    const xs = new Float64Array(n), zs = new Float64Array(n), qs = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      const s = (i * trackLength) / n;
      const q = atlas.lineOffset(s);
      qs[i] = q;
      const p = track.at(s, q);
      xs[i] = p.x; zs[i] = p.z;
    }
    const w = 2;
    let sumAbsStored = 0, sumAbsRecon = 0, maxRatio = 0, worst = 0;
    for (let i = 0; i < n; i++) {
      const a = (i - w + n) % n, c = (i + w) % n;
      const ab = Math.hypot(xs[i] - xs[a], zs[i] - zs[a]);
      const bc = Math.hypot(xs[c] - xs[i], zs[c] - zs[i]);
      const ca = Math.hypot(xs[a] - xs[c], zs[a] - zs[c]);
      const cross = (xs[i] - xs[a]) * (zs[c] - zs[a]) - (zs[i] - zs[a]) * (xs[c] - xs[a]);
      const recon = ab * bc * ca > 1e-6 ? (2 * cross) / (ab * bc * ca) : 0;
      const stored = atlas.lineCurvature((i * trackLength) / n);
      sumAbsStored += Math.abs(stored);
      sumAbsRecon += Math.abs(recon);
      if (Math.abs(stored) > 0.002) {
        const ratio = Math.abs(recon / stored);
        if (ratio > maxRatio) { maxRatio = ratio; worst = (i * trackLength) / n; }
      }
    }
    results[`${h}m`] = {
      sumAbsStored: Number(sumAbsStored.toFixed(3)),
      sumAbsReconstructed: Number(sumAbsRecon.toFixed(3)),
      ratio: Number((sumAbsRecon / Math.max(1e-9, sumAbsStored)).toFixed(3)),
    };
  }
  return { spacingSweep: results };
})();

// §3B / §19 braking normalisation.
const brakeEvents = [];
let br = null;
for (const r of cleanSamples) {
  if (r.brake > 0.02) { if (!br) br = { rows: [] }; br.rows.push(r); }
  else if (br) { brakeEvents.push(br); br = null; }
}
const brakeClassified = brakeEvents.filter((e) => e.rows.length > 15).map((e) => {
  const rows = e.rows;
  const entry = rows[0], peakRow = rows.reduce((a, b) => (a.brake > b.brake ? a : b));
  const release = rows.at(-1);
  const later = cleanSamples.filter((r) => r.s > entry.s && r.s < entry.s + 260);
  const planCross = later.find((r) => r.target < entry.v - 1);
  const earlyBy = planCross ? planCross.s - entry.s : 0;
  const qAtStart = Math.abs(entry.qError);
  let cause = 'NECESSARY';
  if (earlyBy > 12) cause = 'EARLY_START';
  else if (release.throttle < 0.2 && release.v < release.target - 1.5) cause = 'LATE_RELEASE';
  else if (peakRow.brake > 0.9) cause = 'EXCESS_PEAK';
  else if (rows.reduce((s, r) => s + r.causalLoss, 0) > 0.02) cause = 'TARGET_TOO_LOW';
  return {
    sFrom: Number(entry.s.toFixed(0)), sTo: Number(release.s.toFixed(0)),
    causalLoss: Number(rows.reduce((s, r) => s + r.causalLoss, 0).toFixed(3)),
    earlyBy: Number(earlyBy.toFixed(1)),
    qErrorAtStart: Number(qAtStart.toFixed(2)),
    // §20: does braking begin before or after the line has already gone?
    startsAfterLineLoss: qAtStart > 0.75,
    cause,
  };
});
const brakeByCause = {};
for (const e of brakeClassified) brakeByCause[e.cause] = Number(((brakeByCause[e.cause] ?? 0) + e.causalLoss).toFixed(3));
const brakeLaps = validLaps.size;
const brakeAfterLineLoss = brakeClassified.filter((e) => e.startsAfterLineLoss);
const brakeEarlyAfterLine = brakeAfterLineLoss.filter((e) => e.cause === 'EARLY_START')
  .reduce((s, e) => s + e.causalLoss, 0);

const report = {
  startingSha: '2d78d02',
  laps: laps.map((i) => ({ lap: i.lap, time: Number(i.time.toFixed(3)), valid: i.valid })),
  validFlyingLaps: brakeLaps,

  normalization: {
    cleanLineHolding: lineHolding(cleanSamples),
    allLapLineHolding: lineHolding(samples),
    cleanQErrorBuckets: bucketTable(cleanSamples),
    allLapQErrorBuckets: bucketTable(samples),
  },

  departureEvents: {
    count: departureEvents.length,
    events: departureEvents.slice(0, 12),
    leadLagClean: globalLeadLag,
  },

  purePursuit: {
    meanChordError: Number(absMean(cleanSamples, (r) => r.chordError).toFixed(3)),
    p95ChordError: Number(quant(cleanSamples.map((r) => ({ v: Math.abs(r.chordError) })), (r) => r.v, 0.95).toFixed(3)),
    // Positive means the chord sits outside the plan, negative means it cuts in.
    signedMeanChordError: Number(mean(cleanSamples, (r) => r.chordError).toFixed(3)),
    chordErrorAtLowQ: Number(absMean(cleanSamples.filter((r) => Math.abs(r.qError) < 0.3), (r) => r.chordError).toFixed(3)),
    lookaheadBySpeed: [20, 40, 60, 70].map((v) => ({
      speed: v,
      lookahead: Number(Math.min(35, Math.max(8, 5.5 + 0.4 * v)).toFixed(1)),
      lookaheadSeconds: Number((Math.min(35, Math.max(8, 5.5 + 0.4 * v)) / v).toFixed(3)),
    })),
  },

  replanAnalysis,
  actuatorLag,
  curvatureFieldAudit,

  braking: {
    campaignTotal: Number(Object.values(brakeByCause).reduce((a, b) => a + b, 0).toFixed(3)),
    byCauseCampaignTotal: brakeByCause,
    byCausePerValidLap: Object.fromEntries(Object.entries(brakeByCause).map(([k, v]) => [k, Number((v / brakeLaps).toFixed(3))])),
    earlyStartAfterLineLoss: Number(brakeEarlyAfterLine.toFixed(3)),
    fractionOfEarlyStartAfterLineLoss: Number((brakeEarlyAfterLine / Math.max(1e-6, brakeByCause.EARLY_START ?? 0)).toFixed(3)),
  },
};

console.log(JSON.stringify(report, null, 2));
