/**
 * Final-corner robustness laboratory.
 *
 * The question this answers is whether the degraded-stint departure is a
 * throttle problem or a geometry problem. The throttle work measured that no
 * torque policy buys the fresh pace without paying for the final lap, which
 * leaves one untested hypothesis: a line that is time-optimal at peak grip may
 * be dynamically fragile at lower grip, and a slightly different trajectory
 * could make the final sector survivable at a fraction of the throttle cost.
 *
 * Everything here runs the real vehicle, the real servo and the real actuator
 * allocator on a snapshot of the real stint, so the failure has to reproduce
 * before any alternative is trusted.
 *
 *   node tools/robustness-lab.mjs --probe         nominal geometry
 *   node tools/robustness-lab.mjs --capture       snapshot the three tyre states
 *   node tools/robustness-lab.mjs --frontier      candidate x grip-state table
 *   node tools/robustness-lab.mjs --mu            critical-grip sweep
 *   node tools/robustness-lab.mjs --timeline      nominal vs robust causal diff
 */
import { writeFileSync, readFileSync, existsSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

import { Track } from '../src/sim/track.js';
import { Vehicle, SPEC } from '../src/sim/vehicle.js';
import { clamp, wrap, angle, damp } from '../src/sim/math.js';
import { VortexSession } from '../src/vortex-session.js';
import { VortexDriver } from '../src/ai/vortex/vortex-driver.js';
import { VehicleEnvelope } from '../src/ai/vortex/estimation/vehicle-envelope.js';
import { VehicleServo } from '../src/ai/vortex/control/servo.js';
import { ActuatorAllocator } from '../src/ai/vortex/control/actuator-allocator.js';
import { TrackAtlas } from '../src/ai/vortex/atlas/track-atlas.js';

const here = dirname(fileURLToPath(import.meta.url));
const SNAP_PATH = resolve(here, 'robustness-snapshots.json');
const argv = process.argv.slice(2);
const flag = n => argv.includes(n);
const number = (n, d) => { const i = argv.indexOf(n); return i >= 0 && Number.isFinite(Number(argv[i + 1])) ? Number(argv[i + 1]) : d; };

const track = new Track('harbor-ring');
const REGION_START = number('--from', 2450);
const REGION_END = number('--to', track.length);

// ---------------------------------------------------------------- geometry

/** C1 raised-cosine window: zero slope at both ends. */
function window(s, centre, width) {
  const x = (s - (centre - width / 2)) / width;
  if (x <= 0 || x >= 1) return 0;
  return 0.5 * (1 - Math.cos(2 * Math.PI * x));
}

/**
 * Trajectory deformation: q(s) = q_Q0(s) + smooth local offsets.
 *
 * Each term moves the line sideways over a window whose length is set by how
 * much lateral rate the car can afford, not by how tight the corner is. A
 * 2.5 m move over 110 m is a 29 degree path angle and breaks the rear on its
 * own; over a 400 m window the same move is 4 degrees and is a line change
 * rather than a disturbance. Terms are C1, so heading is continuous and the
 * servo is never asked to chase a corner it cannot follow.
 */
export function deform(baseQ, params) {
  // Window centres follow the measured curvature event, not a guess: the
  // driven-path curvature peaks at s≈2630 with a smaller corner at s≈2510.
  const W = params.width ?? 400;
  const terms = [
    { at: params.earlyAt ?? 2510, width: params.earlyWidth ?? W, amp: params.early ?? 0 },
    { at: params.entryAt ?? 2575, width: params.entryWidth ?? W, amp: params.entry ?? 0 },
    { at: params.apexAt ?? 2630, width: params.apexWidth ?? W, amp: params.apex ?? 0 },
    { at: params.exitAt ?? 2685, width: params.exitWidth ?? W, amp: params.exit ?? 0 },
  ];
  return (s) => {
    let q = baseQ(s);
    for (const t of terms) q += t.amp * window(s, t.at, t.width);
    return q;
  };
}

/**
 * Build a path the servo and allocator can both read from a lateral profile.
 * Curvature and heading come from the deformed geometry itself, never from the
 * oracle, because the oracle is only valid for the undisturbed line.
 */
export function buildPath(qFn, speedFn, sFrom, sTo, step = 1.5) {
  const pts = [];
  for (let s = sFrom; s <= sTo; s += step) {
    const ss = wrap(s, track.length);
    const q = qFn(ss);
    const p = track.at(ss, q);
    pts.push({ s: ss, rawS: s, offset: q, x: p.x, z: p.z, heading: p.heading, speed: speedFn(ss, 0) });
  }
  for (let i = 0; i < pts.length; i++) {
    const a = pts[Math.max(0, i - 1)], b = pts[i], c = pts[Math.min(pts.length - 1, i + 1)];
    const h0 = Math.atan2(b.x - a.x, b.z - a.z);
    const h1 = Math.atan2(c.x - b.x, c.z - b.z);
    const ds = Math.max(0.5, (c.rawS - a.rawS) * 0.5);
    b.curvature = Math.atan2(Math.sin(h1 - h0), Math.cos(h1 - h0)) / ds;
    b.heading = Math.atan2(c.x - b.x, c.z - b.z);
  }
  if (pts.length > 1) { pts[0].curvature = pts[1].curvature; pts[0].heading = pts[1].heading; }
  for (const p of pts) p.speedLimit = p.speed;
  return {
    points: pts,
    at(s) {
      const ss = wrap(s, track.length);
      let best = pts[0], bd = Infinity;
      for (const p of pts) {
        let d = Math.abs(p.s - ss);
        if (d > track.length / 2) d = track.length - d;
        if (d < bd) { bd = d; best = p; }
      }
      return best;
    },
  };
}

// ---------------------------------------------------------------- snapshots

function snapshot(car) {
  return {
    x: car.x, z: car.z, y: car.y, yaw: car.yaw,
    vx: car.vx, vz: car.vz, u: car.u, v: car.v, speed: car.speed,
    yawRate: car.yawRate, ax: car.ax, ay: car.ay,
    roll: car.roll, pitch: car.pitch, heave: car.heave,
    steering: car.steering, gear: car.gear, rpm: car.rpm, shiftTimer: car.shiftTimer,
    controls: { ...car.controls }, fuel: car.fuel, damage: car.damage,
    aero: { ...car.aero }, s: car.s, lateral: car.lateral, zone: car.zone,
    impact: car.impact, absActive: car.absActive, tcActive: car.tcActive,
    wheels: car.wheels.map(w => ({
      x: w.x, z: w.z, omega: w.omega, steer: w.steer, compression: w.compression,
      load: w.load, brakeTemp: w.brakeTemp, tyre: { ...w.tyre },
    })),
  };
}

function restore(car, snap) {
  const keys = ['x','z','y','yaw','vx','vz','u','v','speed','yawRate','ax','ay','roll','pitch','heave',
    'steering','gear','rpm','shiftTimer','fuel','damage','s','lateral','zone','impact','absActive','tcActive'];
  for (const k of keys) car[k] = snap[k];
  car.controls = { ...snap.controls };
  car.aero = { ...snap.aero };
  car.wheels = snap.wheels.map(w => ({ ...w, tyre: { ...w.tyre } }));
  return car;
}

function tyreSummary(car) {
  const w = car.wheels;
  const m = f => w.reduce((a, x) => a + f(x), 0) / 4;
  return {
    core: m(x => x.tyre.core), surface: m(x => x.tyre.surface),
    pressure: m(x => x.tyre.pressure), wear: m(x => x.tyre.wear),
    kappa: Math.max(...w.slice(2).map(x => Math.abs(x.tyre.kappa))),
    alpha: Math.max(...w.slice(2).map(x => Math.abs(x.tyre.alpha))),
  };
}

// ---------------------------------------------------------------- capture

function capture() {
  const snaps = {};
  for (const cfg of [
    { key: 'healthy', lap: 2, mode: 'cheap' },
    { key: 'moderate', lap: 3, mode: 'cheap' },
    { key: 'degraded', lap: 4, mode: 'cheap' },
  ]) {
    const session = new VortexSession(track, { classId: 'gt' });
    session.mode = 'practice'; session.field = 1; session.laps = 5; session.autopilot = true;
    session.start({ freshTrack: true });
    const car = session.cars[0];
    const driver = new VortexDriver(0, session.lineFor(car), { aggression: session.aggression });
    driver.allocator = new ActuatorAllocator({ mode: cfg.mode });
    session.drivers[0] = driver;
    const zero = { steer: 0, throttle: 0, brake: 0 };
    let prevLap = car.race.lap, taken = false;
    for (let step = 0; step < 120 * 60 * 10 && !taken; step++) {
      session.step(1 / 120, zero);
      if (session.phase !== 'racing') continue;
      if (car.race.lap !== prevLap) { prevLap = car.race.lap; continue; }
      if (car.race.lap === cfg.lap && car.s >= REGION_START && car.s < REGION_START + 3) {
        snaps[cfg.key] = { lap: cfg.lap, s: car.s, car: snapshot(car), tyre: tyreSummary(car),
          lapTimeSoFar: session.time, u: car.u, beta: Math.atan2(car.v, Math.max(4, car.u)) };
        taken = true;
      }
    }
    if (!taken) console.log(`  ! no snapshot for ${cfg.key} (lap ${cfg.lap})`);
    else console.log(`  ${cfg.key} lap ${cfg.lap}: u=${snaps[cfg.key].u.toFixed(2)} core=${snaps[cfg.key].tyre.core.toFixed(1)} wear=${snaps[cfg.key].tyre.wear.toFixed(5)}`);
  }
  writeFileSync(SNAP_PATH, JSON.stringify(snaps));
  console.log(`wrote ${SNAP_PATH}`);
  return snaps;
}

function loadSnaps() {
  if (!existsSync(SNAP_PATH)) { console.log('no snapshots, capturing...'); return capture(); }
  return JSON.parse(readFileSync(SNAP_PATH, 'utf8'));
}

// ---------------------------------------------------------------- replay

/**
 * Drive a snapshot forward along a candidate trajectory using the real servo
 * and the real actuator allocator. Longitudinal demand is the driver's own
 * braking-point law written against the candidate speed profile, so the only
 * thing under test is the trajectory.
 */
function replay(snap, qFn, opts = {}) {
  const allocatorMode = opts.allocator ?? 'cheap';
  const speedReserve = opts.speedReserve ?? 1;
  const seconds = opts.seconds ?? number('--seconds', 16.0);
  const gripMul = opts.gripMul ?? 1;

  const car = restore(new Vehicle(0, 'PROBE', '#fff', 'gt'), snap.car);
  // Track surface grip multiplier is applied through the tyre load term so the
  // sweep changes available grip without touching anything else in the plant.
  const originalSurface = track.surface.bind(track);
  if (gripMul !== 1) {
    track.surface = (x, z) => { const r = originalSurface(x, z); return { ...r, grip: r.grip * gripMul }; };
  }

  const atlas = new TrackAtlas({ track, offsetAt: () => 0, at: () => ({ speed: 0 }) }, null, {});
  const envelope = new VehicleEnvelope({ classId: 'gt', model: atlas.envelope });
  envelope.muReference = 1;
  const servo = new VehicleServo();
  const allocator = new ActuatorAllocator({ mode: allocatorMode });

  const speedFn = (s) => {
    const base = atlas.profileSpeed(s);
    return base * speedReserve;
  };
  const path = buildPath(qFn, speedFn, snap.s - 40, REGION_END + 60);

  const brakingTarget = (station) => {
    let target = Infinity;
    for (const distance of [0, 2, 4, 7, 12, 20, 32, 50, 75, 110, 160]) {
      const ss = wrap(station + distance, track.length);
      const speed = speedFn(ss);
      const reach = distance <= 0 ? speed
        : Math.sqrt(Math.max(0, speed * speed) + 2 * envelope.at(car, speed, 0, 0).brake * Math.max(0, distance - 1.5));
      if (reach < target) target = reach;
    }
    return Number.isFinite(target) ? target : speedFn(station);
  };

  const dt = 1 / 120;
  const timeline = [];
  let t = 0, spun = false, offtrack = 0;
  let peakBeta = 0, peakYaw = 0, peakKappa = 0, peakSlipPower = 0, peakSteer = 0, peakUtil = 0;
  let minReserve = 1e9, maxQErr = 0, exitSpeed = 0, minSpeed = 1e9, entrySpeed = car.u;
  // Station is tracked without wrapping so the traversal time is the time to
  // actually reach the lap boundary, not just to run for a fixed window.
  let rawS = snap.s;
  const rawStart = snap.s;

  for (let i = 0; i < Math.round(seconds / dt); i++) {
    const pathPt = path.at(car.s);
    const env = envelope.at(car, car.speed, pathPt.curvature ?? 0, car.lateral);
    const target = brakingTarget(car.s);
    const ds = Math.max(5, car.speed * 0.3);
    const here = speedFn(car.s), fwd = speedFn(wrap(car.s + ds, track.length));
    const feedForward = clamp((fwd * fwd - here * here) / (2 * ds), -22, 22);
    const error = target - Math.max(0, car.speed);
    const demand = clamp(error, -40, 40) * (error > 0 ? 3.6 : 6.5) + feedForward;
    const steering = servo.steerTo(car, path, track, 0).steer;
    const act = allocator.allocate(demand, car, env, pathPt.curvature ?? 0);
    car.controls = { steer: steering, throttle: act.throttle, brake: act.brake, reverse: false };
    car.step(dt, track, 0);
    t += dt;

    // Advance the unwrapped station by the distance actually covered.
    const speedNow = Math.max(0, car.speed);
    rawS += speedNow * dt;

    const beta = Math.atan2(car.v, Math.max(4, car.u));
    const util = env.utilisation;
    const rearKappa = Math.max(Math.abs(car.wheels[2].tyre.kappa), Math.abs(car.wheels[3].tyre.kappa));
    const slipPower = (car.wheels[2].tyre.slipPower + car.wheels[3].tyre.slipPower) / 2;
    peakBeta = Math.max(peakBeta, Math.abs(beta));
    peakYaw = Math.max(peakYaw, Math.abs(car.yawRate));
    peakKappa = Math.max(peakKappa, rearKappa);
    peakSlipPower = Math.max(peakSlipPower, slipPower);
    peakSteer = Math.max(peakSteer, Math.abs(car.steering));
    peakUtil = Math.max(peakUtil, util);
    minReserve = Math.min(minReserve, env.lateral - Math.abs(car.ay));
    maxQErr = Math.max(maxQErr, Math.abs(car.lateral - pathPt.offset));
    minSpeed = Math.min(minSpeed, car.speed);
    exitSpeed = car.speed;
    if (Math.abs(car.lateral) > track.halfWidth - car.spec.halfWidth) offtrack += dt;
    if (Math.abs(beta) > 0.9 || Math.abs(car.yawRate) > 2.0) spun = true;
    if (i % 6 === 0) {
      timeline.push({
        t: Number(t.toFixed(3)), s: Number(car.s.toFixed(1)),
        v: Number(car.speed.toFixed(2)), vTarget: Number(target.toFixed(2)),
        q: Number(car.lateral.toFixed(3)), qTarget: Number(pathPt.offset.toFixed(3)),
        curv: Number((pathPt.curvature ?? 0).toFixed(5)),
        steer: Number(car.steering.toFixed(4)), beta: Number(beta.toFixed(4)),
        yaw: Number(car.yawRate.toFixed(3)), thr: Number(act.throttle.toFixed(3)),
        brk: Number(act.brake.toFixed(3)), kappa: Number(rearKappa.toFixed(4)),
        slipPower: Number(slipPower.toFixed(0)), util: Number(util.toFixed(3)),
      });
    }
    if (rawS - rawStart >= (REGION_END - snap.s + track.length) % track.length) break;
    if (t > seconds) break;
  }

  if (gripMul !== 1) track.surface = originalSurface;
  return {
    spin: spun, offtrack: Number(offtrack.toFixed(2)),
    peakBeta: Number(peakBeta.toFixed(3)), peakYaw: Number(peakYaw.toFixed(3)),
    peakKappa: Number(peakKappa.toFixed(3)), peakSlipPower: Number(peakSlipPower.toFixed(0)),
    peakSteer: Number(peakSteer.toFixed(3)), peakUtil: Number(peakUtil.toFixed(3)),
    minReserve: Number(minReserve.toFixed(2)), maxQErr: Number(maxQErr.toFixed(3)),
    entrySpeed: Number(entrySpeed.toFixed(2)), minSpeed: Number(minSpeed.toFixed(2)),
    exitSpeed: Number(exitSpeed.toFixed(2)), elapsed: Number(t.toFixed(3)),
    timeline,
  };
}

// ---------------------------------------------------------------- probes

function probeGeometry() {
  const atlas = new TrackAtlas({ track, offsetAt: () => 0, at: () => ({ speed: 0 }) }, null, {});
  console.log('s      q_Q0    kappa_Q0  profileV');
  for (let s = 2400; s <= track.length; s += 10) {
    const q = atlas.lineOffset(s);
    const k = atlas.lineCurvature(s);
    console.log(`${String(s).padStart(5)}  ${q.toFixed(2).padStart(7)}  ${k.toFixed(5).padStart(9)}  ${(atlas.profileSpeed(s) * 3.6).toFixed(1)}`);
  }
}

const baseAtlas = new TrackAtlas({ track, offsetAt: () => 0, at: () => ({ speed: 0 }) }, null, {});
const baseQ = s => baseAtlas.lineOffset(s);

export { baseQ, replay, loadSnaps, capture, probeGeometry, track, tyreSummary };

/**
 * Candidate set. Each is a shape change to the same corner, expressed in
 * metres of lateral displacement over a smooth C2 window: enter further from
 * the inside, hold a tighter or later apex, and open the exit. Nothing here
 * knows where the corner is on the track - the windows are sized to the
 * curvature event, and the same deformation at any other corner would mean
 * the same thing.
 */
export const CANDIDATES = {
  nominal: {},
  // For a left-hand corner the turn centre is to the left, so pushing the line
  // outward (positive q) at the apex raises the radius and lowers peak
  // curvature - the geometric change that should spend grip rather than cost
  // it. Windows are wide so the move is a line change, not a disturbance.
  apex05: { apex: 0.5 },
  apex10: { apex: 1.0 },
  apex15: { apex: 1.5 },
  apex20: { apex: 2.0 },
  apex25: { apex: 2.5 },
  apex10W700: { apex: 1.0, width: 700 },
  apex15W700: { apex: 1.5, width: 700 },
  apex20W700: { apex: 2.0, width: 700 },
  apex15Late: { apex: 1.5, apexAt: 2680 },
  apex15Early: { apex: 1.5, apexAt: 2580 },
  apex10Plus: { entry: 0.6, apex: 1.0, exit: 0.6 },
  apex10Minus: { apex: -1.0 },
  apex20Minus: { apex: -2.0 },
};

const SNAP_KEYS = ['healthy', 'moderate', 'degraded'];

function runFrontier() {
  const snaps = loadSnaps();
  console.log('\n=== ROBUSTNESS FRONTIER (cheap allocator, pure geometry) ===');
  console.log('candidate    state      spin  off    beta   yaw   kappa  slipPwr  qErr   minV   exitV  elapsed  dT_vs_nominal');
  const rows = [];
  const nominalTime = {};
  for (const key of SNAP_KEYS) {
    nominalTime[key] = replay(snaps[key], deform(baseQ, {}), { allocator: 'cheap' }).elapsed;
  }
  for (const [name, params] of Object.entries(CANDIDATES)) {
    const qFn = deform(baseQ, params);
    const row = { name, states: {} };
    for (const key of SNAP_KEYS) {
      const r = replay(snaps[key], qFn, { allocator: 'cheap' });
      row.states[key] = r;
      const dT = r.elapsed - nominalTime[key];
      console.log(
        name.padEnd(12) + key.padEnd(11) +
        (r.spin ? 'Y   ' : 'n   ') + String(r.offtrack).padStart(5) +
        String(r.peakBeta).padStart(7) + String(r.peakYaw).padStart(7) +
        String(r.peakKappa).padStart(7) + String(r.peakSlipPower).padStart(9) +
        String(r.maxQErr).padStart(7) +
        String(r.minSpeed).padStart(7) + String(r.exitSpeed).padStart(7) +
        String(r.elapsed).padStart(9) +
        (dT >= 0 ? '+' : '') + dT.toFixed(3).padStart(8)
      );
    }
    rows.push(row);
  }
  return rows;
}

function runMu() {
  const snaps = loadSnaps();
  const levels = [1.00, 0.99, 0.98, 0.97, 0.96, 0.94, 0.92];
  console.log('\n=== GRIP SENSITIVITY (degraded snapshot, cheap allocator) ===');
  console.log('candidate    ' + levels.map(l => l.toFixed(2).padStart(7)).join('') + '   muCritical');
  for (const [name, params] of Object.entries(CANDIDATES)) {
    const qFn = deform(baseQ, params);
    const out = [];
    let critical = null;
    for (const g of levels) {
      const r = replay(snaps.degraded, qFn, { allocator: 'cheap', gripMul: g });
      out.push((r.spin ? 'spin' : 'ok').padStart(7));
      if (!r.spin && critical === null) critical = g;
    }
    // Highest grip at which it is still unstable = the margin we actually have.
    const unstable = levels.filter((g, i) => out[i].trim() === 'spin');
    const muCritical = unstable.length ? Math.max(...unstable) + 0.01 : 'no spin in range';
    console.log(name.padEnd(13) + out.join('') + String(muCritical).padStart(13));
  }
}

function runTimeline() {
  const snaps = loadSnaps();
  const rows = [];
  for (const [name, params] of Object.entries(CANDIDATES)) {
    const qFn = deform(baseQ, params);
    const r = replay(snaps.degraded, qFn, { allocator: 'cheap', seconds: 7 });
    rows.push({ name, r });
  }
  rows.sort((a, b) => (a.r.spin === b.r.spin) ? b.r.exitSpeed - a.r.exitSpeed : (a.r.spin ? 1 : -1));
  const best = rows[0];
  console.log(`\n=== CAUSAL TIMELINE: nominal vs best (${best.name}) on degraded snapshot ===`);
  const a = replay(snaps.degraded, deform(baseQ, {}), { allocator: 'cheap', seconds: 7 }).timeline;
  const b = best.r.timeline;
  console.log('s      |  v nom->rob | q nom->rob | curv nom->rob | steer nom->rob | beta nom->rob | yaw nom->rob | util nom->rob');
  for (let i = 0; i < Math.min(a.length, b.length); i += 2) {
    const x = a[i], y = b[i];
    console.log(
      String(x.s).padStart(5) + ' | ' +
      `${x.v.toFixed(1).padStart(5)}->${y.v.toFixed(1).padStart(5)}` + ' | ' +
      `${x.q.toFixed(2).padStart(5)}->${y.q.toFixed(2).padStart(5)}` + ' | ' +
      `${x.curv.toFixed(4).padStart(6)}->${y.curv.toFixed(4).padStart(6)}` + ' | ' +
      `${x.steer.toFixed(2).padStart(5)}->${y.steer.toFixed(2).padStart(5)}` + ' | ' +
      `${x.beta.toFixed(2).padStart(5)}->${y.beta.toFixed(2).padStart(5)}` + ' | ' +
      `${x.yaw.toFixed(2).padStart(5)}->${y.yaw.toFixed(2).padStart(5)}` + ' | ' +
      `${x.util.toFixed(2).padStart(4)}->${y.util.toFixed(2).padStart(4)}`
    );
  }
}

function runValidate() {
  const snaps = loadSnaps();
  console.log('\n=== LAB VALIDATION: nominal Q0 + cheap allocator must reproduce the failure ===');
  for (const key of SNAP_KEYS) {
    const r = replay(snaps[key], deform(baseQ, {}), { allocator: 'cheap' });
    console.log(
      key.padEnd(10) + ` spin=${r.spin ? 'Y' : 'n'} off=${r.offtrack} beta=${r.peakBeta} ` +
      `yaw=${r.peakYaw} kappa=${r.peakKappa} slipPwr=${r.peakSlipPower} ` +
      `v: ${r.entrySpeed} -> min ${r.minSpeed} -> exit ${r.exitSpeed}`
    );
  }
}

if (import.meta.url === `file://${process.argv[1].replace(/\\/g, '/')}` || flag('--run')) {
  if (flag('--probe')) probeGeometry();
  else if (flag('--capture')) capture();
  else if (flag('--validate')) runValidate();
  else if (flag('--frontier')) runFrontier();
  else if (flag('--mu')) runMu();
  else if (flag('--timeline')) runTimeline();
  else console.log('use --probe | --capture | --validate | --frontier | --mu | --timeline');
}
