/**
 * Lap-degradation locator.
 *
 * Records two laps of the same stint and aligns them by station so the
 * degraded-lap loss can be attributed to a place rather than to a total.
 *
 *   node tools/compare-lap-degradation.mjs --a 2 --b 4
 */
import { Track } from '../src/sim/track.js';
import { VortexSession } from '../src/vortex-session.js';
import { VortexDriver } from '../src/ai/vortex/vortex-driver.js';

const argv = process.argv.slice(2);
const number = (name, fallback) => {
  const i = argv.indexOf(name);
  return i >= 0 && Number.isFinite(Number(argv[i + 1])) ? Number(argv[i + 1]) : fallback;
};
const lapA = number('--a', 2);
const lapB = number('--b', 4);
const totalLaps = Math.max(lapA, lapB) + 1;

const track = new Track('harbor-ring');
const session = new VortexSession(track, { classId: 'gt' });
session.mode = 'practice';
session.field = 1;
session.laps = totalLaps;
session.autopilot = true;
session.start({ freshTrack: true });

const car = session.cars[0];
const driver = new VortexDriver(0, session.lineFor(car), { aggression: session.aggression });
session.drivers[0] = driver;

const zero = { steer: 0, throttle: 0, brake: 0 };
const capture = new Map();
let step = 0;
let prevLap = car.race.lap;
let lapStart = 0;

while (step < 120 * 60 * 10) {
  session.step(1 / 120, zero);
  step++;
  if (session.phase !== 'racing') continue;

  if (car.race.lap !== prevLap) {
    if (car.race.lastLap !== null) console.log(`Lap ${prevLap}: ${car.race.lastLap.toFixed(3)}s`);
    prevLap = car.race.lap;
    lapStart = session.time;
    if (car.race.lap > totalLaps) break;
  }

  const want = car.race.lap === lapA || car.race.lap === lapB;
  if (!want) continue;
  if (!capture.has(car.race.lap)) capture.set(car.race.lap, []);
  const here = driver.planner.at(car.s);
  const env = driver.envelope.at(car, car.speed, here.curvature, car.lateral);
  const limiter = driver.limiter || {};
  capture.get(car.race.lap).push({
    t: session.time - lapStart,
    s: car.s,
    q: car.lateral,
    qTarget: here.offset ?? 0,
    curv: here.curvature ?? 0,
    profileSpeed: driver.atlas.profileSpeed(car.s),
    planSpeed: driver.planSpeed(car.s),
    target: driver.targetSpeed ?? 0,
    v: car.speed,
    throttle: car.controls.throttle,
    brake: car.controls.brake,
    steer: car.controls.steer,
    beta: limiter.beta ?? 0,
    yawRate: car.yawRate,
    gripUtil: limiter.gripUtil ?? env.utilisation,
    mu: driver.envelope.muScale,
    planGrip: driver.envelope.planGrip,
    health: limiter.gripHealth ?? 1,
    beta: limiter.beta ?? 0,
    yawRate: car.yawRate,
    steerRad: car.steering ?? 0,
    tc: car.tcActive ? 1 : 0,
  });
}

const A = capture.get(lapA) ?? [];
const B = capture.get(lapB) ?? [];
if (!A.length || !B.length) {
  console.error('missing lap capture', lapA, lapB);
  process.exit(1);
}

for (const [lap, arr] of [[lapA, A], [lapB, B]]) {
  console.log(`\ncapture lap ${lap}: ${arr.length} samples, t ${arr[0].t.toFixed(2)}..${arr.at(-1).t.toFixed(2)} s ${arr[0].s.toFixed(1)}..${arr.at(-1).s.toFixed(1)}`);
  console.log('  first 3:', arr.slice(0, 3).map(x => `s=${x.s.toFixed(1)},t=${x.t.toFixed(2)}`).join('  '));
  console.log('  last 3 :', arr.slice(-3).map(x => `s=${x.s.toFixed(1)},t=${x.t.toFixed(2)}`).join('  '));
  let reversals = 0;
  for (let i = 1; i < arr.length; i++) if (arr[i].s < arr[i - 1].s - 5) reversals++;
  console.log(`  station reversals (s jumps back >5 m): ${reversals}`);
}

const nearest = (samples, s) => {
  let best = samples[0], min = Infinity;
  for (const item of samples) {
    let d = Math.abs(item.s - s);
    if (d > track.length / 2) d = track.length - d;
    if (d < min) { min = d; best = item; }
  }
  return best;
};

// Stations are generated from the lap-start station outward so the table
// reads in time order instead of being rotated by the start/finish offset.
const startS = A[0].s;
const stationStep = 20;
const rows = [];
for (let k = 0; k * stationStep < track.length; k++) {
  const d = k * stationStep;
  const s = (startS + d) % track.length;
  const a = nearest(A, s);
  const b = nearest(B, s);
  rows.push({ d, s, dt: b.t - a.t, a, b });
}

console.log('\n=== LAP ' + lapB + ' vs LAP ' + lapA + ' BY DISTANCE FROM LAP START (20 m) ===');
console.log(' d(m) | stn | dT(s) | v_a->v_b | thr_a/b | brk_a/b | str_a/b | beta_a/b | yaw_a/b | mu_a/b | h_a/b | tc_a/b');
for (const r of rows) {
  const { a, b } = r;
  console.log(
    `${String(r.d).padStart(5)} | ${r.s.toFixed(0).padStart(4)} | ${r.dt >= 0 ? '+' : ''}${r.dt.toFixed(2)} | ` +
    `${(a.v * 3.6).toFixed(0).padStart(3)}->${(b.v * 3.6).toFixed(0).padStart(3)} | ` +
    `${a.throttle.toFixed(2)}/${b.throttle.toFixed(2)} | ${a.brake.toFixed(2)}/${b.brake.toFixed(2)} | ` +
    `${a.steer.toFixed(2)}/${b.steer.toFixed(2)} | ${a.beta.toFixed(2)}/${b.beta.toFixed(2)} | ` +
    `${a.yawRate.toFixed(2)}/${b.yawRate.toFixed(2)} | ${a.mu.toFixed(2)}/${b.mu.toFixed(2)} | ` +
    `${a.health.toFixed(2)}/${b.health.toFixed(2)} | ${a.tc}/${b.tc}`
  );
}

console.log('\n=== BIGGEST LOCAL LOSSES (dT jump > 0.15 s per 20 m) ===');
for (let i = 1; i < rows.length; i++) {
  const jump = rows[i].dt - rows[i - 1].dt;
  if (jump > 0.15) {
    const { a, b } = rows[i];
    console.log(
      `d=${rows[i].d}m (stn ${rows[i].s.toFixed(0)}) +${jump.toFixed(2)}s (cum +${rows[i].dt.toFixed(2)}s): ` +
      `v ${a.v.toFixed(1)}->${b.v.toFixed(1)} tgt ${a.target.toFixed(1)}->${b.target.toFixed(1)} ` +
      `thr ${a.throttle.toFixed(2)}->${b.throttle.toFixed(2)} brk ${a.brake.toFixed(2)}->${b.brake.toFixed(2)} ` +
      `steer ${a.steer.toFixed(2)}->${b.steer.toFixed(2)} beta ${a.beta.toFixed(2)}->${b.beta.toFixed(2)} ` +
      `yaw ${a.yawRate.toFixed(2)}->${b.yawRate.toFixed(2)} mu ${a.mu.toFixed(2)}->${b.mu.toFixed(2)} ` +
      `planGrip ${a.planGrip.toFixed(2)}->${b.planGrip.toFixed(2)} health ${a.health.toFixed(2)}->${b.health.toFixed(2)} ` +
      `tc ${a.tc}->${b.tc}`
    );
  }
}
