import { clamp, wrap } from '../../../sim/math.js';
import { EnvelopeModel } from './envelope.js';
import oracleRecord from './oracle-data.js';

const FAMILIES = Object.freeze([
  { id: 'Q0', offset: 0 }, { id: 'QL1', offset: -1.3 }, { id: 'QR1', offset: 1.3 },
  { id: 'QL2', offset: -2.65 }, { id: 'QR2', offset: 2.65 },
  { id: 'QDEF-L', offset: -3.35 }, { id: 'QDEF-R', offset: 3.35 },
  { id: 'QEXIT-L', offset: -1.9 }, { id: 'QEXIT-R', offset: 1.9 },
  { id: 'QCROSS', offset: 0 }, { id: 'QLATE', offset: 2.2 },
]);

const ORACLE_FALLBACK = Object.freeze({
  n: 0,
  spacing: 1,
  lapTime: 0,
  q: [],
  v: [],
  kappa: [],
});

/**
 * Adaptive track atlas: one optimised free-air trajectory plus a family of
 * continuously deformable corridors around it.
 *
 * The atlas is the authority on two things the driver cannot invent at runtime:
 * the fastest legal line, and the fastest physically reachable speed profile
 * along it. Both come from the offline oracle so the runtime is limited by the
 * simulated car rather than by a hand-tuned racing line.
 */
export class TrackAtlas {
  constructor(line, memory = null, options = {}) {
    this.line = line;
    this.track = line.track;
    this.memory = memory;
    this.options = options;
    this.families = FAMILIES.map(item => ({ ...item }));
    this.envelope = options.envelope ?? new EnvelopeModel(options);
    this.oracle = ORACLE_FALLBACK;
    this.usingOracle = false;
    this.installOracle(options.oracle ?? oracleRecord ?? null);
  }

  /** Install an oracle record produced by tools/build-vortex-oracle.mjs. */
  installOracle(payload) {
    if (!payload || !Number.isFinite(payload.n) || payload.n < 8) return false;
    const n = payload.n | 0;
    if (payload.q?.length !== n || payload.v?.length !== n) return false;
    this.oracle = {
      n,
      spacing: Number(payload.spacing) || this.track.length / n,
      lapTime: Number(payload.lapTime) || 0,
      q: Float64Array.from(payload.q),
      v: Float64Array.from(payload.v),
      kappa: Float64Array.from(payload.kappa ?? []),
    };
    this.qMax = Number(payload.legalOffset) || (this.track.halfWidth - 0.99 - 0.12);
    // The record was solved for a specific grip level; the runtime envelope
    // must agree with it or displaced corridors get a corner budget the
    // optimiser never had.
    if (Number.isFinite(payload.gripScale) && this.envelope.setGripScale) {
      this.envelope.setGripScale(payload.gripScale);
    }
    this.usingOracle = true;
    return true;
  }

  /** Lateral position of the optimised free-air line at a station. */
  lineOffset(s) {
    if (!this.usingOracle) return this.line.offsetAt(s);
    const t = wrap(s, this.track.length) / this.oracle.spacing;
    const i = Math.floor(t) % this.oracle.n, j = (i + 1) % this.oracle.n, f = t - Math.floor(t);
    return this.oracle.q[i] + (this.oracle.q[j] - this.oracle.q[i]) * f;
  }

  /** Free-air speed profile at a station: the physically reachable maximum. */
  profileSpeed(s) {
    if (!this.usingOracle) return this.line.at(s).speed;
    const t = wrap(s, this.track.length) / this.oracle.spacing;
    const i = Math.floor(t) % this.oracle.n, j = (i + 1) % this.oracle.n, f = t - Math.floor(t);
    const base = this.oracle.v[i] + (this.oracle.v[j] - this.oracle.v[i]) * f;
    // The stored profile is a solution at one particular grip level. The live
    // tyre state modulates the whole envelope, so the plan has to scale with
    // it: corner speeds go as sqrt(mu). Without this the car keeps targeting a
    // grip that stopped existing mid-stint and departs once the tyres are gone.
    return base * Math.sqrt(clamp(this.envelope.muScale, 0.55, 1.45));
  }

  /** Driven-path curvature of the optimised line at a station. */
  lineCurvature(s) {
    if (!this.usingOracle || !this.oracle.kappa.length) return this.track.at(s).curvature;
    const t = wrap(s, this.track.length) / this.oracle.spacing;
    const i = Math.floor(t) % this.oracle.n, j = (i + 1) % this.oracle.n, f = t - Math.floor(t);
    return this.oracle.kappa[i] + (this.oracle.kappa[j] - this.oracle.kappa[i]) * f;
  }

  sample(s, extra = 0) {
    const track = this.track;
    const residual = this.memory?.residualAt(s);
    const base = this.lineOffset(s) + (residual?.residualQ ?? 0);
    const limit = this.qMax ?? (track.halfWidth - 0.99 - 0.12);
    const offset = clamp(base + extra, -limit, limit);
    const point = track.at(wrap(s, track.length), offset);
    const speed = Math.max(6, this.profileSpeed(s) + (residual?.residualSpeed ?? 0) * 6);
    return {
      ...point,
      s: wrap(s, track.length),
      offset,
      speed,
      conservativeSpeed: speed * 0.88,
      speedLimit: speed,
      curvature: this.lineCurvature(s),
      legalLimit: limit,
    };
  }

  corridor(id, s, extra = 0) {
    const family = this.families.find(item => item.id === id);
    return this.sample(s, extra + (family?.offset ?? 0));
  }

  familyAt(s) { return this.families.map(item => ({ ...item, ...this.corridor(item.id, s) })); }

  /**
   * Fastest speed the car may hold at `s` while still being able to reach the
   * stored profile everywhere ahead under braking.
   *
   * This is the physical braking-point law. Targeting the speed of some point a
   * fixed distance ahead instead makes the car brake to corner speed from that
   * distance out, which is exactly the phantom early-braking failure.
   */
  brakingTarget(s, horizon = 240, margin = 2) {
    const track = this.track;
    let target = Infinity;
    let travelled = 0;
    const step = 4;
    const limit = Math.min(horizon, track.length * 0.5);
    while (travelled <= limit) {
      const station = wrap(s + travelled, track.length);
      const speed = this.profileSpeed(station);
      const curvature = this.lineCurvature(station);
      const lateral = this.envelope.lateralAt(speed);
      const braking = this.envelope.brakeAt(speed);
      const reserve = this.envelope.reserve(clamp(speed * speed * Math.abs(curvature) / Math.max(1, lateral), 0, 0.99));
      const reach = Math.sqrt(speed * speed + 2 * braking * reserve * Math.max(0, travelled - margin));
      if (reach < target) target = reach;
      travelled += step;
    }
    return Number.isFinite(target) ? target : this.profileSpeed(s);
  }
}
