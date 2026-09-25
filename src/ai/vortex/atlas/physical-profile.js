import { clamp, wrap } from '../../../sim/math.js';

const G = 9.81, RHO = 1.225, DEFAULT_MASS = 1290 + 35 * 0.75;

// Friction coefficients calibrated against measured canonical Vehicle.step
// (tools/probe-vehicle.mjs): straight braking peaks at 15.0-17.8 m/s^2 and
// steady cornering at 13.28 m/s^2 at 20 m/s rising to 17.65 m/s^2 at 50 m/s.
// Load sensitivity is the tyre's own log law, so grip falls slightly as aero
// load rises. These are limits of the simulated car, not of a racing line.
export const PHYSICAL = Object.freeze({
  muLat: 1.245,
  muLong: 1.345,
  driveScale: 0.93,
  rolling: 0.13,
  vmax: 78,
  vmin: 6,
});

export function loadFactor(loadPerWheel) {
  return clamp(1 - 0.13 * Math.log(Math.max(0.1, loadPerWheel / 3300)), 0.68, 1.18);
}

export function downforce(spec, v, mass, wing = 6) {
  return 0.5 * RHO * v * v * spec.area * (spec.cl + (wing - 6) * 0.11);
}

/** Total vertical load per wheel at speed v. */
export function totalLoad(spec, v, mass = DEFAULT_MASS, wing = 6) {
  return mass * G + downforce(spec, v, mass, wing);
}

/** Pure lateral acceleration available at speed v, in m/s^2. */
export function lateralAccel(spec, v, mass = DEFAULT_MASS, wing = 6, p = PHYSICAL) {
  const total = totalLoad(spec, v, mass, wing);
  return p.muLat * loadFactor(total / 4) * (total / mass);
}

/** Pure longitudinal acceleration available at speed v, in m/s^2. */
export function driveAccel(spec, v, mass = DEFAULT_MASS, wing = 6, p = PHYSICAL) {
  let gear = 1;
  while (gear < 6 && v / spec.radius * spec.gears[gear] * spec.finalDrive * 9.5493 > 7450) gear++;
  const ratio = spec.gears[gear] * spec.finalDrive;
  const rpm = Math.max(1100, v / spec.radius * ratio * 9.5493);
  const curve = clamp(1 - ((rpm - 5500) / 6700) ** 2, 0.45, 1);
  const force = spec.maxTorque * curve * ratio * 0.91 / spec.radius;
  const drag = 0.5 * RHO * v * v * spec.area * spec.cd;
  return clamp((force * p.driveScale - drag) / mass - p.rolling, -3, 9);
}

/** Pure braking deceleration available at speed v, in m/s^2. */
export function brakeAccel(spec, v, mass = DEFAULT_MASS, wing = 6, p = PHYSICAL) {
  const total = totalLoad(spec, v, mass, wing);
  return p.muLong * loadFactor(total / 4) * (total / mass);
}

function lateralLimitSpeed(spec, curvature, mass, wing, p) {
  const k = Math.abs(curvature);
  if (k < 1e-6) return p.vmax;
  let lo = 4, hi = p.vmax;
  for (let i = 0; i < 40; i++) {
    const mid = (lo + hi) / 2;
    if (mid * mid * k <= lateralAccel(spec, mid, mass, wing, p)) lo = mid; else hi = mid;
  }
  return lo;
}

function dist(points, i, j, length) {
  return wrap(points[j].distance - points[i].distance, length) || 0.1;
}

/**
 * Closed-loop time-optimal speed profile over a fixed geometric path.
 * Backward pass enforces braking feasibility, forward pass enforces
 * acceleration feasibility, both wrapped so the start/finish seam is not a
 * free speed reset. Longitudinal authority is reduced by the friction ellipse
 * through corners, so entry braking and exit traction are physically coupled.
 */
export function buildSpeedProfile(points, spec, options = {}) {
  const mass = options.mass ?? DEFAULT_MASS, wing = options.wing ?? 6;
  const p = { ...PHYSICAL, ...options.physical };
  const n = points.length;
  // Perimeter is the track length; `distance` runs 0..perimeter along the path.
  const perimeter = options.length ?? points[n - 1].distance;
  const cap = new Float64Array(n);
  for (let i = 0; i < n; i++) cap[i] = lateralLimitSpeed(spec, points[i].curvature, mass, wing, p);

  const ellipse = (v, curvature) => {
    const lateral = lateralAccel(spec, v, mass, wing, p);
    const use = clamp(v * v * Math.abs(curvature) / Math.max(0.1, lateral), 0, 0.985);
    const reserve = Math.sqrt(Math.max(0, 1 - use * use));
    return { brake: brakeAccel(spec, v, mass, wing, p) * reserve, drive: driveAccel(spec, v, mass, wing, p) * reserve, lateral, use };
  };

  const speed = Float64Array.from(cap);
  for (let pass = 0; pass < 3; pass++) {
    for (let i = n - 1; i >= 0; i--) {
      const j = (i + 1) % n, ds = dist(points, i, j, perimeter);
      const { brake } = ellipse(speed[j], points[j].curvature);
      speed[i] = Math.min(speed[i], Math.sqrt(speed[j] * speed[j] + 2 * brake * ds));
    }
    for (let i = 0; i < n; i++) {
      const j = (i - 1 + n) % n, ds = dist(points, j, i, perimeter);
      const { drive } = ellipse(speed[j], points[j].curvature);
      speed[i] = Math.min(speed[i], Math.sqrt(speed[j] * speed[j] + 2 * drive * ds));
    }
  }
  for (let i = 0; i < n; i++) speed[i] = clamp(speed[i], p.vmin, p.vmax);

  let time = 0;
  const times = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n, ds = dist(points, i, j, perimeter);
    time += 2 * ds / Math.max(1, speed[i] + speed[j]);
    times[j] = time;
  }
  return { speed, times, cap, lapTime: time, physical: p };
}

/**
 * Lightweight adapter exposing the `speedProfile` contract from
 * `sim/performance.js` without inheriting the conservative hand-tuned limits.
 */
export class PhysicalModel {
  constructor(track, spec, options = {}) {
    this.track = track; this.spec = spec;
    this.mass = options.mass ?? DEFAULT_MASS; this.wing = options.wing ?? 6;
    this.p = { ...PHYSICAL, ...options.physical };
  }
  lateral(v) { return lateralAccel(this.spec, v, this.mass, this.wing, this.p); }
  brake(v) { return brakeAccel(this.spec, v, this.mass, this.wing, this.p); }
  drive(v) { return driveAccel(this.spec, v, this.mass, this.wing, this.p); }
  at(speed, s, lateral = 0) {
    const surface = Math.abs(lateral) > this.track.halfWidth - this.spec.halfWidth ? 0.88 : 1;
    return { lateral: this.lateral(speed) * surface, brake: this.brake(speed) * surface,
      drive: this.drive(speed) * surface, mu: this.p.muLat, surface };
  }
  longitudinal(speed, s, lateral, curvature) {
    const e = this.at(speed, s, lateral);
    const use = clamp(speed * speed * Math.abs(curvature) / Math.max(0.1, e.lateral), 0, 0.985);
    const reserve = Math.sqrt(Math.max(0, 1 - use * use));
    return { ...e, brake: e.brake * reserve, drive: e.drive * reserve, utilisation: use,
      speedLimit: lateralLimitSpeed(this.spec, curvature, this.mass, this.wing, this.p) };
  }
}
