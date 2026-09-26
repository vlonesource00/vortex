/**
 * Yaw-demand census and fragility selector.
 *
 * The previous block measured that the corner which departs is not the tightest
 * one on the lap: s=2360 runs 0.097 of curvature and s=860 runs 0.086 against
 * the failing corner's 0.036, and both sit at similar lateral utilisation. What
 * the failing corner does have is a large yaw transient on its approach - the
 * nominal line spends its rear-axle margin rotating into the corner before the
 * apex arrives - so this measures that demand directly and compares it against
 * what the live vehicle can actually generate and settle.
 *
 * Reference yaw rate and its spatial derivative along the planned line:
 *
 *   r_ref    = v * kappa
 *   rDot_ref = aLong * kappa + v^2 * dKappa/ds
 *
 * The second term is the one that matters: a moderate curvature carried through
 * a rapid curvature transition asks for more yaw acceleration than a tighter
 * steady-state corner does.
 *
 *   node tools/yaw-census.mjs                trajectory census over the lap
 *   node tools/yaw-census.mjs --live         with live axle reserves
 *   node tools/yaw-census.mjs --rank         fragility ranking per snapshot
 *   node tools/yaw-census.mjs --compare      Q0 vs robust geometry
 */
import { readFileSync, existsSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

import { Track } from '../src/sim/track.js';
import { SPEC } from '../src/sim/vehicle.js';
import { clamp, wrap } from '../src/sim/math.js';
import { VortexSession } from '../src/vortex-session.js';
import { VortexDriver } from '../src/ai/vortex/vortex-driver.js';
import { TrackAtlas } from '../src/ai/vortex/atlas/track-atlas.js';
import { ActuatorAllocator } from '../src/ai/vortex/control/actuator-allocator.js';
import { TyrePredictor, gripSensitivityToCore } from '../src/ai/vortex/control/tyre-predictor.js';

const here = dirname(fileURLToPath(import.meta.url));
const SNAP_PATH = resolve(here, 'robustness-snapshots.json');
const argv = process.argv.slice(2);
const flag = n => argv.includes(n);

const track = new Track('harbor-ring');
const atlas = new TrackAtlas({ track, offsetAt: () => 0, at: () => ({ speed: 0 }) }, null, {});

// Chassis geometry from the plant (src/sim/vehicle.js resetState).
const FRONT_ARM = (1 - SPEC.frontWeight) * SPEC.wheelbase;   // 1.4734 m ahead of CG
const REAR_ARM = SPEC.frontWeight * SPEC.wheelbase;          // 1.3066 m behind CG
const YAW_INERTIA = SPEC.yawInertia;

// ------------------------------------------------------------------ demand

/**
 * Driven-path curvature derived from the geometry the car actually follows,
 * not from the oracle's stored kappa array. The two differ: the stored value
 * is whatever the solver recorded at 2.2 m spacing, while the vehicle follows
 * q(s) sampled continuously, and the yaw transient lives in that difference.
 */
const PATH_STEP = 2;
const pathCache = new Map();
function pathGeometry() {
  if (pathCache.size) return pathCache;
  const samples = [];
  for (let i = 0; i * PATH_STEP < track.length; i++) {
    const s = i * PATH_STEP;
    const q = atlas.lineOffset(s);
    const p = track.at(s, q);
    samples.push({ s, q, x: p.x, z: p.z });
  }
  const n = samples.length;
  for (let i = 0; i < n; i++) {
    const a = samples[(i - 1 + n) % n], b = samples[i], c = samples[(i + 1) % n];
    const h0 = Math.atan2(b.x - a.x, b.z - a.z);
    const h1 = Math.atan2(c.x - b.x, c.z - b.z);
    const ds = Math.max(0.5, (c.s - a.s + track.length) % track.length * 0.5 || PATH_STEP);
    const k = Math.atan2(Math.sin(h1 - h0), Math.cos(h1 - h0)) / Math.max(0.5, PATH_STEP);
    pathCache.set(b.s, { s: b.s, q: b.q, k, x: b.x, z: b.z });
  }
  return pathCache;
}

function kappaAt(s) {
  const g = pathGeometry();
  const key = Math.round(wrap(s, track.length) / PATH_STEP) * PATH_STEP % track.length;
  return g.get(key)?.k ?? 0;
}
function dkappa(s, h = 4) {
  return (kappaAt(wrap(s + h, track.length)) - kappaAt(wrap(s - h, track.length))) / (2 * h);
}
function d2kappa(s, h = 6) {
  return (kappaAt(wrap(s + h, track.length)) - 2 * kappaAt(s) + kappaAt(wrap(s - h, track.length))) / (h * h);
}

/**
 * Trajectory yaw demand at a station. `aLong` is the planned longitudinal
 * acceleration from the profile itself, so the derivative term is the total
 * reference yaw acceleration the line asks for, not just its geometric part.
 */
function yawDemand(s, aLong = 0) {
  const v = Math.max(4, atlas.profileSpeed(s));
  const k = kappaAt(s);
  const dk = dkappa(s);
  const rRef = v * k;
  const rDot = aLong * k + v * v * dk;
  return { s, v, k, dk, d2k: d2kappa(s), rRef, rDot };
}

/** Planned longitudinal acceleration from the profile's own speed gradient. */
function plannedALong(s, h = 10) {
  const v0 = atlas.profileSpeed(wrap(s - h, track.length));
  const v1 = atlas.profileSpeed(wrap(s + h, track.length));
  return (v1 * v1 - v0 * v0) / (2 * 2 * h);
}

// ------------------------------------------------------------------ capability

/**
 * Available yaw acceleration, split by axle.
 *
 * The front axle is what generates rotation (steering can call on its unused
 * lateral capacity) and the rear axle is what settles it. A rotation the car
 * cannot generate is a slow line; a rotation it cannot settle is the departure
 * that was measured, so the limiting side is the smaller of the two.
 */
function axleReserves(ego) {
  const w = ego.wheels;
  const peak = x => {
    const load = Math.max(0, x.load);
    const g = 1.48 * clamp(1 - ((x.tyre.core - 85) / 105) ** 2, 0.65, 1)
      * clamp(1 - Math.abs(x.tyre.pressure - 2.15) * 0.13, 0.8, 1)
      * clamp(1 - 0.13 * Math.log(Math.max(0.1, load / 3300)), 0.68, 1.18)
      * (1 - x.tyre.wear * 0.35);
    return Math.max(0, load * g);
  };
  let frontRes = 0, rearRes = 0, frontUsed = 0, rearUsed = 0;
  for (let i = 0; i < 4; i++) {
    const used = Math.abs(w[i].tyre.fy);
    const reserve = Math.max(0, peak(w[i]) - used);
    if (i < 2) { frontRes += reserve; frontUsed += used; }
    else { rearRes += reserve; rearUsed += used; }
  }
  const mass = SPEC.mass + (ego.fuel ?? 0) * 0.75;
  return {
    frontReserve: frontRes, rearReserve: rearRes,
    frontUsed, rearUsed,
    yawGen: frontRes * FRONT_ARM / YAW_INERTIA,
    yawSettle: rearRes * REAR_ARM / YAW_INERTIA,
    yawAccelAvailable: Math.min(frontRes * FRONT_ARM, rearRes * REAR_ARM) / YAW_INERTIA,
    mass,
  };
}

// ------------------------------------------------------------------ census

function census() {
  console.log('=== FULL-LAP YAW DEMAND CENSUS (nominal Q0) ===');
  console.log('s      v(m/s)   kappa     dkappa/ds    r_ref    rDot_ref   d2kappa');
  const rows = [];
  for (let s = 0; s < track.length; s += 10) {
    const d = yawDemand(s, plannedALong(s));
    rows.push(d);
  }
  // Print the strongest events only: local maxima of |rDot_ref|.
  const peaks = [];
  for (let i = 1; i < rows.length - 1; i++) {
    const a = Math.abs(rows[i].rDot), b = Math.abs(rows[i - 1].rDot), c = Math.abs(rows[i + 1].rDot);
    if (a > b && a >= c && a > 0.02) peaks.push(rows[i]);
  }
  peaks.sort((x, y) => Math.abs(y.rDot) - Math.abs(x.rDot));
  for (const p of peaks.slice(0, 14)) {
    console.log(
      String(p.s).padStart(5) + '  ' + p.v.toFixed(2).padStart(6) + '  ' + p.k.toFixed(5).padStart(9) +
      '  ' + p.dk.toExponential(2).padStart(11) + '  ' + p.rRef.toFixed(3).padStart(7) +
      '  ' + p.rDot.toFixed(4).padStart(8) + '  ' + p.d2k.toExponential(2).padStart(10)
    );
  }

  console.log('\n=== PEAK COMPARISON OF THE THREE NAMED REGIONS ===');
  const regions = [
    { name: 's 840-900', lo: 840, hi: 900 },
    { name: 's 2340-2400', lo: 2340, hi: 2400 },
    { name: 's 2550-2640', lo: 2550, hi: 2640 },
    { name: 's 2480-2530', lo: 2480, hi: 2530 },
    { name: 's 1400-1450', lo: 1400, hi: 1450 },
    { name: 's 2240-2280', lo: 2240, hi: 2280 },
  ];
  console.log('region          peakK     peakUtil  peakRref  peakRdot  peak_dk');
  for (const r of regions) {
    let pk = 0, pu = 0, pr = 0, pd = 0, pdk = 0;
    for (let s = r.lo; s <= r.hi; s += 4) {
      const d = yawDemand(s, plannedALong(s));
      const v = Math.max(6, atlas.profileSpeed(s));
      const cap = atlas.envelope.lateralAt(v);
      pk = Math.max(pk, Math.abs(d.k));
      pu = Math.max(pu, v * v * Math.abs(d.k) / Math.max(1, cap));
      pr = Math.max(pr, Math.abs(d.rRef));
      pd = Math.max(pd, Math.abs(d.rDot));
      pdk = Math.max(pdk, Math.abs(d.dk));
    }
    console.log(
      r.name.padEnd(16) + pk.toFixed(5).padStart(8) + '  ' + pu.toFixed(3).padStart(8) +
      '  ' + pr.toFixed(3).padStart(8) + '  ' + pd.toFixed(4).padStart(8) + '  ' + pdk.toExponential(2).padStart(9)
    );
  }
  return rows;
}

// ------------------------------------------------------------------ live rank

function loadSnaps() {
  if (!existsSync(SNAP_PATH)) return null;
  return JSON.parse(readFileSync(SNAP_PATH, 'utf8'));
}

/**
 * Live census: drive a full lap and record, at every station, the trajectory's
 * yaw demand against what the vehicle can actually generate and settle there.
 * The ratio is the selector's quantity; the raw demand is not.
 */
function liveCensus(label, allocatorMode, forcedAmp) {
  const session = new VortexSession(track, { classId: 'gt' });
  session.mode = 'practice'; session.field = 1; session.laps = 4; session.autopilot = true;
  session.start({ freshTrack: true });
  const car = session.cars[0];
  const driver = new VortexDriver(0, session.lineFor(car), { aggression: session.aggression });
  driver.allocator = new ActuatorAllocator({ mode: allocatorMode });
  if (forcedAmp !== undefined) {
    driver.planner.generator.robustness.enabled = true;
    driver.planner.generator.robustness.forcedAmp = forcedAmp;
  }
  session.drivers[0] = driver;

  const zero = { steer: 0, throttle: 0, brake: 0 };
  let prevLap = car.race.lap;
  const lapRows = new Map();
  for (let step = 0; step < 120 * 60 * 12; step++) {
    session.step(1 / 120, zero);
    if (session.phase !== 'racing') continue;
    if (car.race.lap !== prevLap) {
      prevLap = car.race.lap;
      if (car.race.lap > 4) break;
      continue;
    }
    const lap = car.race.lap;
    if (!lapRows.has(lap)) lapRows.set(lap, []);
    const s = car.s;
    const demand = yawDemand(s, car.ax);
    const res = axleReserves(car);
    const beta = Math.atan2(car.v, Math.max(4, car.u));
    lapRows.get(lap).push({
      s, lap, v: car.speed,
      k: demand.k, dk: demand.dk, rRef: demand.rRef, rDot: demand.rDot,
      beta, yaw: car.yawRate,
      frontReserve: res.frontReserve, rearReserve: res.rearReserve,
      yawGen: res.yawGen, yawSettle: res.yawSettle,
      avail: res.yawAccelAvailable,
      ratio: Math.abs(demand.rDot) / Math.max(1e-3, res.yawAccelAvailable),
      rearRatio: Math.abs(demand.rDot) / Math.max(1e-3, res.yawSettle),
    });
  }
  return { label, lapRows };
}

function summariseLive(run, lap) {
  const rows = run.lapRows.get(lap) ?? [];
  if (!rows.length) return;
  // Fold into ~20 m bins and take the worst ratio in each, which is the
  // question the selector answers: does this event exceed the live reserve?
  const bins = new Map();
  for (const r of rows) {
    const b = Math.floor(r.s / 20) * 20;
    if (!bins.has(b) || bins.get(b).ratio < r.ratio) bins.set(b, r);
  }
  const events = [...bins.values()].sort((a, b) => b.ratio - a.ratio).slice(0, 12);
  console.log(`\n=== ${run.label} LAP ${lap}: yaw-transient ratio by station ===`);
  console.log('s      v(m/s)   rDot_ref  yawGen  yawSettle  RATIO   rearRATIO  rearRes  frontRes   beta');
  for (const r of events) {
    console.log(
      String(r.s).padStart(5) + '  ' + r.v.toFixed(2).padStart(6) + '  ' + r.rDot.toFixed(4).padStart(8) +
      '  ' + r.yawGen.toFixed(3).padStart(6) + '  ' + r.yawSettle.toFixed(3).padStart(9) +
      '  ' + r.ratio.toFixed(3).padStart(6) + '  ' + r.rearRatio.toFixed(3).padStart(9) +
      '  ' + r.rearReserve.toFixed(0).padStart(7) + '  ' + r.frontReserve.toFixed(0).padStart(8) +
      '  ' + r.beta.toFixed(3).padStart(6)
    );
  }
  return events;
}

// ------------------------------------------------------------------ compare

function compareGeometry() {
  console.log('\n=== Q0 vs ROBUST GEOMETRY: YAW DEMAND AT THE FRAGILE EVENT ===');
  // The measured deformation opens the line outward through the event. The
  // equivalent geometry change on the trajectory is a reduction in the peak
  // curvature the line carries and in its rate of change, which is what the
  // robustness lab measured as lower approach yaw rate.
  const eventS = 2630;
  console.log('variant        peakK     dkappa     rDot_ref   (nominal line sampled, then deformed)');
  for (const [name, amp] of [['Q0 nominal', 0], ['apex +1.5', 1.5], ['apex +2.0', 2.0], ['apex +2.5', 2.5]]) {
    let pk = 0, pdk = 0, pd = 0;
    for (let s = 2520; s <= 2700; s += 4) {
      const d = yawDemand(s, plannedALong(s));
      // The outward offset raises the radius of the arc it passes through, so
      // the curvature the line actually carries is reduced in proportion to the
      // offset relative to the local radius. Measured in the robustness lab:
      // 1.5 m of offset at this event cut the peak beta from 1.28 to 0.66 and
      // the approach yaw transient from 0.97 to 0.75 rad/s.
      const radius = 1 / Math.max(1e-4, Math.abs(d.k));
      const factor = 1 / (1 + amp / Math.max(4, radius));
      const kEff = d.k * factor;
      const dkEff = d.dk * factor;
      const rDotEff = d.v * d.v * dkEff;
      pk = Math.max(pk, Math.abs(kEff));
      pdk = Math.max(pdk, Math.abs(dkEff));
      pd = Math.max(pd, Math.abs(rDotEff));
    }
    console.log(name.padEnd(14) + pk.toFixed(5).padStart(8) + '  ' + pdk.toExponential(2).padStart(10) + '  ' + pd.toFixed(4).padStart(9));
  }
}

if (flag('--run') || argv[0]?.endsWith('yaw-census.mjs')) {
  if (flag('--compare')) compareGeometry();
  else if (flag('--live')) {
    // Laps 1-4 of the real stint, so the healthy / moderate / degraded ranking
    // comes from the actual tyre states the car had at each point.
    const run = liveCensus('physical+nominal', 'physical', 0);
    for (const lap of [1, 2, 3, 4]) summariseLive(run, lap);
  } else census();
}
