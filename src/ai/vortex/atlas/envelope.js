import { clamp } from '../../../sim/math.js';
import { tyreGrip } from '../../../sim/tyre.js';
import { carSpecFor } from '../../../sim/car-specs.js';

const RHO = 1.225;
const AMBIENT = 24;

/** Linear lookup table over a uniform grid. */
export class Table1D {
  constructor(min, max, count, fn) {
    this.min = min; this.max = max;
    this.step = (max - min) / (count - 1);
    this.values = new Float64Array(count);
    for (let i = 0; i < count; i++) this.values[i] = fn(min + i * this.step);
  }
  at(x) {
    const t = clamp((x - this.min) / this.step, 0, this.values.length - 1.000001);
    const i = t | 0, f = t - i;
    return this.values[i] + (this.values[i + 1] - this.values[i]) * f;
  }
}

/**
 * Reference tyre thermal/pressure state. The ideal-gas law matches
 * src/sim/tyre.js so the predicted grip factor is the one the plant will
 * actually present once the tyre is at that core temperature.
 */
export function referenceTyre({ core = 85, coldPressure = 1.65, wear = 0 } = {}) {
  return {
    core,
    surface: core,
    coldPressure,
    pressure: (coldPressure + 1.01325) * ((core + 273.15) / (AMBIENT + 273.15)) - 1.01325,
    wear,
    alpha: 0,
    kappa: 0,
    fx: 0,
    fy: 0,
  };
}

/**
 * Physical GG envelope derived from the frozen plant in src/sim/tyre.js and
 * src/sim/vehicle.js. It predicts what the canonical simulator permits; it
 * never edits tyre, aero, mass, geometry or collision code.
 *
 * Peak tyre force is the plant's own expression
 *   F = surface.grip * spec.tyreGrip * load * tyreGrip(tyre, load)
 * evaluated at the four wheel loads the chassis would carry, including the
 * plant's longitudinal and lateral load transfer. Load transfer matters: the
 * tyre force law is concave in load, so a transferred axle pair produces less
 * total force than an equally loaded one.
 */
export class EnvelopeModel {
  constructor(options = {}) {
    this.spec = carSpecFor(options.classId ?? 'gt');
    this.fuel = options.fuel ?? 35;
    this.wing = options.wing ?? 6;
    this.gripScale = options.gripScale ?? 1;
    this.surfaceGrip = options.surfaceGrip ?? 1;
    this.tyre = options.tyre ?? referenceTyre(options);
    // tanh(slip) saturation peak at the ABS hold point (kappa ~ -0.2) plus the
    // plant's camber residual. Measured against the plant in
    // tools/calibrate-envelope.mjs.
    this.shape = options.shape ?? 0.948;
    this.mass = this.spec.mass + this.fuel * 0.75;
    this.cl = this.spec.cl + (this.wing - 6) * 0.11;
    this.cd = this.spec.cd + (this.wing - 6) * 0.013;
    this.rearAero = this.spec.key === 'gt' ? 0.57 : 1 - this.spec.frontAero;
    this.muScale = 1;
    this.buildTables();
  }

  /** Re-derive the cached lookups after a change to grip or tyre state. */
  setGripScale(scale) { this.gripScale = scale; return this.buildTables(); }
  /** Bounded online grip modulation. Applied as a multiplier, never a rebuild. */
  setMuScale(scale) { this.muScale = scale; return this; }

  downforce(v) { return this.aero(v, 0).downforce; }
  dragForce(v) { return 0.5 * RHO * v * v * this.spec.area * this.cd; }

  /**
   * Aerodynamic state at a given speed and longitudinal acceleration. The
   * plant's `platform` factor is load dependent: heave from downforce and
   * pitch from longitudinal acceleration both shed vertical load, so the
   * downforce the tyre actually sees is well below the raw aero number. It is
   * solved self-consistently because downforce drives heave.
   */
  aero(v, ax) {
    const raw = 0.5 * RHO * v * v * this.spec.area * this.cl;
    let platform = 1;
    for (let i = 0; i < 24; i++) {
      const downforce = raw * platform;
      const heave = downforce / 310000;
      const ride = 0.066 - heave;
      const pitch = clamp(-ax * 0.0028, -0.06, 0.07);
      const next = clamp(1 - Math.abs(pitch) * 1.4 - Math.max(0, 0.03 - ride) * 14, 0.55, 1);
      if (Math.abs(next - platform) < 1e-4) { platform = next; break; }
      platform += (next - platform) * 0.6;
    }
    return { platform, downforce: raw * platform, raw };
  }

  /** Drag plus the plant's asphalt rolling resistance. */
  parasiticAccel(v) {
    return (this.dragForce(v) + 0.013 * (this.mass * 9.81 + this.aero(v, 0).downforce)) / this.mass;
  }

  tyreFactor(load) {
    const l = Math.max(0, load);
    if (l < 1) return 0;
    return this.gripScale * this.surfaceGrip * this.spec.tyreGrip * l * tyreGrip(this.tyre, l) * this.shape;
  }

  /** Wheel loads used by Vehicle.step for a given longitudinal/lateral demand. */
  wheelLoads(v, ax = 0, ay = 0) {
    const m = this.mass, S = this.spec, df = this.aero(v, ax).downforce;
    const long = clamp(ax, -22, 18) * m * S.cg / S.wheelbase;
    const lat = clamp(ay, -25, 25) * m * S.cg / S.track;
    const front = m * 9.81 * S.frontWeight - long + df * S.frontAero;
    const rear = m * 9.81 * (1 - S.frontWeight) + long + df * this.rearAero;
    return [
      Math.max(0, front / 2 + lat * 0.52),
      Math.max(0, front / 2 - lat * 0.52),
      Math.max(0, rear / 2 + lat * 0.48),
      Math.max(0, rear / 2 - lat * 0.48),
    ];
  }

  totalPeak(v, ax, ay) {
    let total = 0;
    for (const load of this.wheelLoads(v, ax, ay)) total += this.tyreFactor(load);
    return total * this.muScale;
  }

  /** Saturated pure-lateral acceleration, including load transfer. */
  lateralAccel(v) {
    let ay = 10;
    for (let i = 0; i < 48; i++) {
      const next = this.totalPeak(v, 0, ay) / this.mass;
      if (!Number.isFinite(next)) break;
      ay += (next - ay) * 0.55;
      if (Math.abs(next - ay) < 1e-4) break;
    }
    return Math.max(1, ay);
  }

  brakeTorqueAccel() {
    const S = this.spec;
    return 2 * S.brakeTorque / S.radius / this.mass;
  }

  /** Saturated pure-braking deceleration magnitude, including transfer and drag. */
  brakeAccel(v) {
    let a = 12;
    for (let i = 0; i < 48; i++) {
      const next = Math.min(this.totalPeak(v, -a, 0) / this.mass, this.brakeTorqueAccel()) + this.parasiticAccel(v);
      if (!Number.isFinite(next)) break;
      a += (next - a) * 0.55;
      if (Math.abs(next - a) < 1e-4) break;
    }
    return Math.max(1, a);
  }

  gearAt(v) {
    const S = this.spec;
    let gear = 1;
    while (gear < 6 && Math.abs(v) / S.radius * S.gears[gear] * S.finalDrive * 9.5493 > 7450) gear++;
    return gear;
  }

  /** Engine force at full throttle, before traction limiting. */
  engineAccel(v) {
    const S = this.spec, gear = this.gearAt(v);
    const ratio = S.gears[gear] * S.finalDrive;
    const rpm = Math.max(1100, Math.abs(v) / S.radius * ratio * 9.5493);
    const torqueCurve = clamp(1 - ((rpm - 5500) / 6700) ** 2, 0.45, 1);
    const force = S.maxTorque * torqueCurve * ratio * 0.91 / S.radius;
    return force / this.mass;
  }

  /** Saturated pure-drive acceleration, traction limited on the driven axle. */
  driveAccel(v) {
    const S = this.spec;
    const engine = this.engineAccel(v);
    let a = engine;
    for (let i = 0; i < 48; i++) {
      const loads = this.wheelLoads(v, a, 0);
      const rear = S.drive === 'front' ? loads[0] + loads[1] : loads[2] + loads[3];
      const peak = this.tyreFactor(rear / 2) * 2 * this.muScale;
      const next = Math.min(engine, peak / this.mass) - this.parasiticAccel(v);
      if (!Number.isFinite(next)) break;
      a += (next - a) * 0.55;
      if (Math.abs(next - a) < 1e-4) break;
    }
    return Math.max(0.2, a);
  }

  /** Speed a given path curvature allows under the pure-lateral limit. */
  curvatureSpeed(curvature) {
    const k = Math.abs(curvature);
    if (k < 1e-5) return 82;
    let v = Math.sqrt(this.lateralAccel(40) / k);
    for (let i = 0; i < 24; i++) {
      const next = Math.sqrt(this.lateralAccel(v) / k);
      if (!Number.isFinite(next)) break;
      v += (next - v) * 0.6;
    }
    return clamp(v, 4, 86);
  }

  /** Friction-ellipse reserve left for longitudinal work at a given use. */
  reserve(utilisation) { return Math.sqrt(Math.max(0, 1 - clamp(utilisation, 0, 0.995) ** 2)); }

  /**
   * Cached one-dimensional lookups. The exact solves above are correct but far
   * too slow for a 120 Hz control loop, so every hot path reads these tables.
   */
  buildTables() {
    // Tables are always built at muScale 1 so the online modulation can be
    // applied as a multiplier without double counting.
    const modulation = this.muScale;
    this.muScale = 1;
    this.latTable = new Table1D(0, 95, 191, v => this.lateralAccel(v));
    this.brakeTable = new Table1D(0, 95, 191, v => this.brakeAccel(v));
    this.driveTable = new Table1D(0, 95, 191, v => this.driveAccel(v));
    this.cornerTable = new Table1D(0.0002, 0.22, 221, k => {
      let lo = 2, hi = 92;
      for (let i = 0; i < 40; i++) {
        const mid = (lo + hi) * 0.5;
        if (mid * mid * k <= this.latTable.at(mid)) lo = mid; else hi = mid;
      }
      return lo;
    });
    this.muScale = modulation;
    return this;
  }

  lateralAt(v) { return this.latTable.at(v) * this.muScale; }
  brakeAt(v) { return this.brakeTable.at(v) * this.muScale; }
  driveAt(v) { return this.driveTable.at(v) * this.muScale; }
  cornerSpeedAt(curvature) {
    return this.cornerTable.at(Math.abs(curvature)) * Math.sqrt(clamp(this.muScale, 0.5, 1.5));
  }
  reserveAt(v, curvature) {
    return this.reserve(v * v * Math.abs(curvature) / Math.max(1, this.lateralAt(v)));
  }

  snapshot(v = 40) {
    return {
      mass: this.mass,
      muScale: this.muScale,
      gripScale: this.gripScale,
      lateral: this.lateralAccel(v),
      brake: this.brakeAccel(v),
      drive: this.driveAccel(v),
      downforce: this.aero(v, 0).downforce,
      drag: this.parasiticAccel(v),
    };
  }
}

export { clamp };
