import { clamp } from '../../../sim/math.js';
import { pathCurvature } from '../../../sim/path-geometry.js';
import { EnvelopeModel, Table1D } from './envelope.js';

/**
 * Cyclic projection of a lateral line onto a station-varying |dq/ds| budget.
 *
 * A car cannot change its line while it is already using all of its grip to
 * hold a corner. A quasi-steady oracle has no such notion and will happily
 * demand a multi-metre excursion through a fast bend, which the closed-loop
 * driver then physically cannot make: it stays off the planned line for a
 * whole sector and loses seconds. Limiting the line's station slope in
 * proportion to the grip the profile is already using forces the optimiser to
 * acquire its offset on the straights and hold the line through the bends.
 */
export function slopeLimitVarying(q, h, slopes) {
  const n = q.length;
  const out = Float64Array.from(q);
  for (let pass = 0; pass < 6; pass++) {
    for (let i = 0; i < n; i++) {
      const j = (i + 1) % n;
      const step = Math.min(slopes[i], slopes[j]) * h;
      const lo = out[i] - step, hi = out[i] + step;
      if (out[j] < lo) out[j] = lo; else if (out[j] > hi) out[j] = hi;
    }
    for (let i = n - 1; i >= 0; i--) {
      const j = (i + 1) % n;
      const step = Math.min(slopes[i], slopes[j]) * h;
      const lo = out[j] - step, hi = out[j] + step;
      if (out[i] < lo) out[i] = lo; else if (out[i] > hi) out[i] = hi;
    }
  }
  return out;
}

/**
 * Offline time-optimal oracle for the canonical Harbor Ring.
 *
 * The decision variable is the lateral line q(s) on a dense station grid. For
 * a candidate line the driven path geometry gives a curvature field, the
 * vehicle envelope gives a corner-speed limit, and a cyclic braking /
 * acceleration feasibility sweep gives the fastest physically reachable speed
 * profile. Lap time is the trapezoidal integral of that profile. The line is
 * then deformed against the gradient of lap time.
 *
 * Everything is deterministic: fixed grid, fixed iteration count, no random
 * search. The plant in src/sim is never modified; the oracle only predicts
 * what it permits and the campaign tools then drive the real Vehicle over the
 * result.
 */
export class TrackOracle {
  constructor(track, options = {}) {
    this.track = track;
    this.envelope = options.envelope ?? new EnvelopeModel(options);
    this.qMax = options.qMax ?? (track.halfWidth - 0.99 - 0.12);
    this.spacing = options.spacing ?? 2.2;
    this.stencil = options.stencil ?? 3;
    this.passes = options.passes ?? 3;
    this.smoothness = options.smoothness ?? 0.004;
    this.qSlopeMax = options.qSlopeMax ?? Infinity;
    this.qSlopeMin = options.qSlopeMin ?? 0.01;
    this.buildGrid();
    this.buildTables();
  }

  buildGrid() {
    const track = this.track;
    const n = Math.max(220, Math.round(track.length / this.spacing));
    this.n = n;
    this.h = track.length / n;
    this.s = new Float64Array(n);
    this.cx = new Float64Array(n);
    this.cz = new Float64Array(n);
    this.nx = new Float64Array(n);
    this.nz = new Float64Array(n);
    this.tx = new Float64Array(n);
    this.tz = new Float64Array(n);
    this.centreCurvature = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      const s = i * this.h;
      const p = track.at(s);
      this.s[i] = s;
      this.cx[i] = p.x; this.cz[i] = p.z;
      this.nx[i] = p.nx; this.nz[i] = p.nz;
      this.tx[i] = p.tx; this.tz[i] = p.tz;
      this.centreCurvature[i] = p.curvature;
    }
  }

  buildTables() {
    const e = this.envelope;
    this.lateral = new Table1D(0, 95, 191, v => e.lateralAccel(v));
    this.brake = new Table1D(0, 95, 191, v => e.brakeAccel(v));
    this.drive = new Table1D(0, 95, 191, v => e.driveAccel(v));
    // Corner speed: solve v^2 * k = aLat(v) by bisection on the table.
    this.corner = new Table1D(0.0002, 0.22, 221, k => {
      let lo = 2, hi = 92;
      for (let i = 0; i < 40; i++) {
        const mid = (lo + hi) * 0.5;
        if (mid * mid * k <= this.lateral.at(mid)) lo = mid; else hi = mid;
      }
      return lo;
    });
  }

  /** Speed the friction ellipse still allows for longitudinal work. */
  reserve(v, kappa) {
    const util = clamp(v * v * Math.abs(kappa) / Math.max(1, this.lateral.at(v)), 0, 0.995);
    return Math.sqrt(1 - util * util);
  }

  /** Path geometry for a candidate lateral line. */
  geometry(q) {
    const n = this.n;
    const px = new Float64Array(n), pz = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      px[i] = this.cx[i] + this.nx[i] * q[i];
      pz[i] = this.cz[i] + this.nz[i] * q[i];
    }
    const d = new Float64Array(n), kappa = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      const j = (i + 1) % n;
      d[i] = Math.hypot(px[j] - px[i], pz[j] - pz[i]);
    }
    const w = this.stencil;
    for (let i = 0; i < n; i++) {
      const a = (i - w + n) % n, c = (i + w) % n;
      kappa[i] = pathCurvature({ x: px[a], z: pz[a] }, { x: px[i], z: pz[i] }, { x: px[c], z: pz[c] });
    }
    return { px, pz, d, kappa };
  }

  /** Cyclic braking / acceleration feasibility sweep on a closed circuit. */
  profile(d, kappa) {
    const n = this.n;
    const v = new Float64Array(n);
    for (let i = 0; i < n; i++) v[i] = this.corner.at(Math.abs(kappa[i]));
    for (let pass = 0; pass < this.passes; pass++) {
      for (let sweep = 0; sweep < 2; sweep++) {
        for (let i = n - 1; i >= 0; i--) {
          const j = (i + 1) % n;
          const limit = Math.sqrt(v[j] * v[j] + 2 * this.brake.at(v[j]) * this.reserve(v[j], kappa[j]) * d[i]);
          if (limit < v[i]) v[i] = limit;
        }
      }
      for (let sweep = 0; sweep < 2; sweep++) {
        for (let i = 0; i < n; i++) {
          const j = (i - 1 + n) % n;
          const limit = Math.sqrt(v[j] * v[j] + 2 * this.drive.at(v[j]) * this.reserve(v[j], kappa[j]) * d[j]);
          if (limit < v[i]) v[i] = limit;
        }
      }
    }
    return v;
  }

  /** Full evaluation: geometry, profile, lap time and a regularisation term. */
  evaluate(q) {
    const { px, pz, d, kappa } = this.geometry(q);
    const v = this.profile(d, kappa);
    const n = this.n;
    let time = 0;
    for (let i = 0; i < n; i++) {
      const j = (i + 1) % n;
      time += 2 * d[i] / Math.max(1, v[i] + v[j]);
    }
    let rough = 0;
    for (let i = 0; i < n; i++) {
      const a = q[(i - 1 + n) % n], b = q[i], c = q[(i + 1) % n];
      rough += (c - 2 * b + a) ** 2;
    }
    return { time, cost: time + this.smoothness * rough, v, kappa, d, px, pz };
  }

  gradient(q, baseCost) {
    const n = this.n;
    const g = new Float64Array(n);
    const delta = 0.05;
    const work = Float64Array.from(q);
    for (let i = 0; i < n; i++) {
      work[i] = q[i] + delta;
      g[i] = (this.evaluate(work).cost - baseCost) / delta;
      work[i] = q[i];
    }
    return g;
  }

  smooth(vector, sigmaIndex) {
    const n = this.n;
    const radius = Math.max(1, Math.round(sigmaIndex * 2.5));
    const kernel = [];
    let sum = 0;
    for (let k = -radius; k <= radius; k++) {
      const weight = Math.exp(-(k * k) / (2 * sigmaIndex * sigmaIndex));
      kernel.push(weight); sum += weight;
    }
    const out = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      let acc = 0;
      for (let k = -radius; k <= radius; k++) acc += vector[(i + k + n) % n] * kernel[k + radius];
      out[i] = acc / sum;
    }
    return out;
  }

  /**
   * Deterministic gradient descent on lap time. Returns the best line found
   * together with the full speed profile so the result can be replayed and
   * measured against the physical simulator.
   */
  optimize(options = {}) {
    const n = this.n;
    const iterations = options.iterations ?? 120;
    const qMax = options.qMax ?? this.qMax;
    let q = options.initial ? Float64Array.from(options.initial) : new Float64Array(n);
    for (let i = 0; i < n; i++) q[i] = clamp(q[i], -qMax, qMax);
    q = this.smooth(q, 2);
    let best = this.evaluate(q);
    let bestQ = Float64Array.from(q);
    // The lateral-rate budget shrinks with the grip the profile is already
    // using: a car at the friction limit cannot also translate across the
    // road. Recomputed from the current best profile at every step.
    const project = value => {
      if (!(this.qSlopeMax < Infinity)) return value;
      const slopes = new Float64Array(n);
      for (let i = 0; i < n; i++) {
        const v = Math.max(1, best.v[i]);
        const used = clamp(v * v * Math.abs(best.kappa[i]) / Math.max(1, this.envelope.lateralAt(v)), 0, 1);
        slopes[i] = Math.max(this.qSlopeMin, this.qSlopeMax * (1 - used));
      }
      return slopeLimitVarying(value, this.h, slopes);
    };
    q = project(q);
    best = this.evaluate(q);
    bestQ = Float64Array.from(q);
    const initialStep = options.step ?? 1.4;
    const minStep = initialStep * 1e-3;
    let step = initialStep;
    const sigmaIndex = (options.smoothing ?? 12) / this.h;
    const history = [];
    let stalls = 0;

    for (let iteration = 0; iteration < iterations; iteration++) {
      const g = this.smooth(this.gradient(q, best.cost), sigmaIndex);
      let improved = false;
      let tryStep = step;
      for (let attempt = 0; attempt < 9; attempt++) {
        const candidate = new Float64Array(n);
        for (let i = 0; i < n; i++) candidate[i] = clamp(q[i] - tryStep * g[i], -qMax, qMax);
        const smoothed = project(this.smooth(candidate, 1.2));
        const result = this.evaluate(smoothed);
        if (result.cost < best.cost - 1e-7) {
          best = result; bestQ = smoothed; q = smoothed;
          step = Math.min(initialStep * 8, tryStep * 1.25);
          improved = true;
          break;
        }
        tryStep *= 0.62;
      }
      history.push({ iteration, time: Number(best.time.toFixed(4)), step: Number(step.toFixed(5)) });
      if (improved) { stalls = 0; continue; }
      step = Math.max(minStep, step * 0.5);
      if (++stalls >= 4) break;
    }

    return {
      n,
      spacing: this.h,
      q: bestQ,
      time: best.time,
      v: best.v,
      kappa: best.kappa,
      d: best.d,
      history,
    };
  }

  /**
   * Resample the optimised line at an arbitrary track station. Used by the
   * runtime atlas so the planner can query the oracle continuously.
   */
  sampler(result) {
    const { q, v, kappa, d } = result;
    const n = this.n, track = this.track;
    return (station, extraOffset = 0) => {
      const s = ((station % track.length) + track.length) % track.length;
      const t = s / this.h;
      const i = Math.floor(t) % n, j = (i + 1) % n, f = t - Math.floor(t);
      const lerp = (a, b) => a + (b - a) * f;
      return {
        s,
        offset: lerp(q[i], q[j]) + extraOffset,
        speed: lerp(v[i], v[j]),
        curvature: lerp(kappa[i], kappa[j]),
        segment: lerp(d[i], d[j]),
      };
    };
  }

  /** Serialisable, deterministic record of an optimised oracle. */
  static toJSON(result, meta = {}) {
    return {
      version: 1,
      generatedBy: 'build-vortex-oracle',
      ...meta,
      n: result.n,
      spacing: result.spacing,
      lapTime: Number(result.time.toFixed(5)),
      q: Array.from(result.q, value => Number(value.toFixed(5))),
      v: Array.from(result.v, value => Number(value.toFixed(4))),
      kappa: Array.from(result.kappa, value => Number(value.toFixed(6))),
    };
  }
}
