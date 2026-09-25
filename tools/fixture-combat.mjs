/**
 * Controlled combat fixture: slow rival ahead.
 *
 * Reproduces the reported failure directly -- VORTEX catching a slower car and
 * losing time looking for a way around instead of committing to a gap.
 *
 * Measures, over the pursuit:
 *   - plan id selected at each replan, and how often it changes side;
 *   - target speed versus the free-air profile, i.e. how much pace the planner
 *     voluntarily gives away to the car in front;
 *   - time lost against a free-air counterfactual over the same distance.
 *
 *   node tools/fixture-combat.mjs
 *   node tools/fixture-combat.mjs --gap 22 --rivalSpeed 32
 */
import { Track } from '../src/sim/track.js';
import { wrap } from '../src/sim/math.js';
import { VortexSession } from '../src/vortex-session.js';

const argv = process.argv.slice(2);
const number = (name, fallback) => {
  const i = argv.indexOf(name);
  return i >= 0 && Number.isFinite(Number(argv[i + 1])) ? Number(argv[i + 1]) : fallback;
};

const track = new Track('harbor-ring');
const gap = number('--gap', 22);
const rivalSpeed = number('--rivalSpeed', 32);
const seconds = number('--seconds', 30);

// --- free-air counterfactual -------------------------------------------------
const solo = new VortexSession(track, { classId: 'gt' });
solo.mode = 'practice'; solo.field = 1; solo.laps = 1; solo.autopilot = true;
solo.start({ freshTrack: true });
solo.phase = 'racing'; solo.countdown = 0;
const soloCar = solo.player;
{
  const p = track.at(wrap(number('--startS', 200), track.length), 0);
  soloCar.s = wrap(number('--startS', 200), track.length);
  soloCar.x = p.x; soloCar.z = p.z; soloCar.yaw = p.heading;
  soloCar.lateral = 0;
  soloCar.u = rivalSpeed + 6; soloCar.v = 0; soloCar.speed = rivalSpeed + 6;
}
const zero = { steer: 0, throttle: 0, brake: 0 };
const freeAir = new Map();                       // station -> { time, speed }
let t = 0;
while (t < 60) {
  solo.step(1 / 120, zero);
  t += 1 / 120;
  if (solo.phase !== 'racing') continue;
  const car = solo.player;
  const key = Math.round(car.s);
  if (!freeAir.has(key)) freeAir.set(key, { time: t, speed: car.speed });
  if (t > 45) break;
}

// --- pursuit scenario --------------------------------------------------------
const session = new VortexSession(track, { classId: 'gt' });
session.mode = 'race'; session.field = 2; session.laps = 1; session.autopilot = true;
session.start({ freshTrack: true });
// Skip the standing countdown so the fixture measures pursuit, not the launch.
session.phase = 'racing';
session.countdown = 0;

const START_S = number('--startS', 200);
const ego = session.cars[0];
const rival = session.cars[1];
// Roll both cars at speed onto the main straight so the pursuit is a clean
// catch, not a standing-start artefact.
for (const [car, s, lateral, speed] of [[ego, START_S, 0, rivalSpeed + 6], [rival, START_S + gap, 0, rivalSpeed]]) {
  const p = track.at(wrap(s, track.length), lateral);
  car.s = wrap(s, track.length);
  car.x = p.x; car.z = p.z;
  car.yaw = p.heading;
  car.lateral = lateral;
  car.u = speed; car.v = 0; car.speed = speed;
  car.controls = { throttle: 0, brake: 0, steer: 0 };
}

const egoDriver = session.drivers[0];
const samples = [];
const planSwitches = [];
const scoreLog = [];
let lastScoreLog = -1;
let lastPlanId = null;
let lastPlanOffset = 0;

t = 0;
while (t < seconds) {
  // Hold the rival at a constant, slow, straight-ahead pace.
  rival.speed = rivalSpeed; rival.u = rivalSpeed;
  session.step(1 / 120, zero);
  t += 1 / 120;
  if (session.phase !== 'racing') break;
  const car = session.player;
  const planId = egoDriver.plan?.id ?? null;
  const planOffset = egoDriver.planner.at(car.s + 25).offset;
  if (planId !== lastPlanId) {
    planSwitches.push({
      t: Number(t.toFixed(2)),
      s: Number(car.s.toFixed(1)),
      from: lastPlanId,
      to: planId,
      offsetFrom: Number(lastPlanOffset.toFixed(2)),
      offsetTo: Number(planOffset.toFixed(2)),
    });
    lastPlanId = planId;
    lastPlanOffset = planOffset;
  }
  samples.push({
    t: Number(t.toFixed(2)),
    s: Number(car.s.toFixed(1)),
    v: Number(car.speed.toFixed(2)),
    target: Number((egoDriver.limiter?.targetSpeed ?? car.speed).toFixed(2)),
    profile: Number(egoDriver.atlas.profileSpeed(car.s).toFixed(2)),
    planId,
    planOffset: Number(planOffset.toFixed(2)),
    lateral: Number(car.lateral.toFixed(2)),
    throttle: Number(car.controls.throttle.toFixed(2)),
    brake: Number(car.controls.brake.toFixed(2)),
  });

  // §17 Score decomposition. Sampled sparsely: we need the ranking and the
  // reason, not every one of the 27 Hz replans.
  if (Math.abs(t - lastScoreLog) > 0.75 && egoDriver.planner?.candidates?.length) {
    lastScoreLog = t;
    const ranked = [...egoDriver.planner.candidates].sort((a, b) => a.score - b.score).slice(0, 3);
    scoreLog.push({
      t: Number(t.toFixed(2)),
      s: Number(car.s.toFixed(1)),
      gapToRival: Number(wrap(rival.s - car.s + track.length / 2, track.length) - track.length / 2).toFixed(1),
      contractState: egoDriver.planner?.attack?.state ?? null, contractFlank: egoDriver.planner?.attack?.active?.flank ?? null, contractSwitches: egoDriver.planner?.attack?.switchLog?.length ?? 0, backward: egoDriver.plan?.backwardPropagation ?? egoDriver.planner?.candidates?.[0]?.backwardPropagation ?? null, origins: egoDriver.plan?.interactionOrigins ?? egoDriver.planner?.candidates?.[0]?.interactionOrigins ?? null, ranked: ranked.map((c) => ({
        id: c.id,
        score: Number(c.score.toFixed(3)),
        targetShift: Number((c.targetShift ?? 0).toFixed(2)),
        relation: c.terminalRelation ?? null,
        trafficLimited: c.trafficLimitedFraction ?? null,
        breakdown: c.breakdown ?? null,
      })),
    });
  }
}

// --- metrics ----------------------------------------------------------------
const distanceTravelled = wrap(samples.at(-1).s - samples[0].s + track.length, track.length);
const freeAirRef = [...freeAir.entries()].sort((a, b) => a[0] - b[0]);
const sameDistanceTime = (() => {
  // Time the solo car took to cover the same station span.
  const from = samples[0].s, to = samples.at(-1).s;
  const inSpan = freeAirRef.filter(([s]) => wrap(s - from + track.length / 2, track.length) - track.length / 2 >= 0
    && wrap(s - from + track.length / 2, track.length) - track.length / 2 <= wrap(to - from, track.length));
  if (inSpan.length < 4) return null;
  return inSpan.at(-1)[1].time - inSpan[0][1].time;
})();

const planIdChanges = planSwitches.length;
const sideFlips = planSwitches.filter((s) => Math.abs(s.offsetTo - s.offsetFrom) > 0.6).length;
const meanTargetVsProfile = samples.reduce((s, r) => s + (r.profile - r.target), 0) / Math.max(1, samples.length);
const meanSpeed = samples.reduce((s, r) => s + r.v, 0) / Math.max(1, samples.length);
const actualTime = samples.at(-1).t - samples[0].t;

// §20 Score margin: how decisively the winner wins.
const margins = scoreLog
  .filter((e) => e.ranked.length > 1)
  .map((e) => e.ranked[1].score - e.ranked[0].score);
const marginStats = {
  n: margins.length,
  mean: margins.length ? Number((margins.reduce((a, b) => a + b, 0) / margins.length).toFixed(3)) : 0,
  p50: margins.length ? Number([...margins].sort((a, b) => a - b)[Math.floor(margins.length * 0.5)].toFixed(3)) : 0,
  p95: margins.length ? Number([...margins].sort((a, b) => a - b)[Math.floor(margins.length * 0.95)].toFixed(3)) : 0,
};

const report = {
  config: { gap, rivalSpeed, seconds },
  pursuitSeconds: Number(actualTime.toFixed(2)),
  distanceTravelled: Number(distanceTravelled.toFixed(1)),
  meanSpeed: Number(meanSpeed.toFixed(2)),
  meanTargetVsProfile: Number(meanTargetVsProfile.toFixed(2)),
  freeAirTimeSameSpan: sameDistanceTime === null ? null : Number(sameDistanceTime.toFixed(2)),
  timeLostVersusFreeAir: sameDistanceTime === null ? null : Number((actualTime - sameDistanceTime).toFixed(2)),

  planSelection: {
    idChanges: planIdChanges,
    significantSideFlips: sideFlips,
    switches: planSwitches.slice(0, 25),
    idHistogram: samples.reduce((h, r) => { h[r.planId ?? 'null'] = (h[r.planId ?? 'null'] ?? 0) + 1; return h; }, {}),
    scoreMargins: marginStats,
    scoreTimeline: scoreLog,
  },

  timeline: samples.filter((_, i) => i % 24 === 0),
};

function trackLengthConst(tr) { return tr.length; }

console.log(JSON.stringify(report, null, 2));
