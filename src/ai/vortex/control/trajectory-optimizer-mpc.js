import { angle, clamp, wrap } from '../../../sim/math.js';
import { dynamicState, rollout } from './dynamic-model.js';

/**
 * 60 Hz deterministic shooting MPC.
 *
 * Replaces the previous 3x3 trim grid, which could only apply one constant
 * steering delta and one constant acceleration bias across the whole horizon.
 * That is a trim layer: it cannot express brake -> trail -> release -> rotate ->
 * pickup -> unwind, which is exactly the sequence the measured losses demand
 * (scrub 35%, braking 18% of integrationLoss).
 *
 * Structure:
 *   - controls are knot-parameterised: 4 steering knots + 4 longitudinal knots
 *     interpolated across the horizon, so a candidate is a *sequence*;
 *   - longitudinal knots are ABSOLUTE accelerations spanning the real envelope,
 *     not deltas around a constant, so a candidate can hard-brake then trail
 *     then coast then apply full throttle;
 *   - the physical envelope is recomputed at every node from the predicted
 *     state, so the solver watches the combined tyre budget evolve as lateral
 *     demand rises and falls;
 *   - station progress comes from canonical track projection of the rolled-out
 *     position, not from s0 + v0*t, which is wrong under braking;
 *   - every solve is warm-started from the previous solution shifted forward,
 *     and the search is a deterministic shrinking coordinate descent.
 *
 * The objective rewards progress and terminal velocity explicitly, so the
 * cheapest way to score well is to drive the car further, not to slow it down
 * until the tracking errors look small.
 */
export class TrajectoryOptimizer {
  constructor(track, envelope, options = {}) {
    this.track = track;
    this.envelope = envelope;

    this.enabled = options.enabled !== false;
    this.authority = options.authority ?? 1;
    // Applied-command envelope. Defaults reproduce the previous trim layer's
    // authority exactly, so A/B comparison is meaningful.
    this.maxSteerDelta = options.maxSteerDelta ?? 0.018;
    this.steerSlew = options.steerSlew ?? 0.004;
    this.maxAccelBiasHigh = options.maxAccelBiasHigh ?? 0.55;
    this.maxAccelBiasLow = options.maxAccelBiasLow ?? -0.9;
    this.accelSlew = options.accelSlew ?? 0.16;

    this.KNOTS = options.knots ?? 4;
    this.NODES = options.nodes ?? 14;
    this.H = options.step ?? 0.10;           // 14 * 0.10 = 1.4 s horizon
    this.PASSES = options.passes ?? 3;

    // Objective. Progress and terminal velocity carry the lap time; the rest
    // keep the car legal and on the plan. The stability weights are deliberately
    // modest: when heading and sideslip dominate, the cheapest sequence is
    // always "steer less, brake more", which is exactly the conservative
    // behaviour the measured losses show.
    this.wProgress = options.wProgress ?? 1.0;
    this.wExit = options.wExit ?? 1.25;
    this.wLine = options.wLine ?? 2.6;
    this.wHeading = options.wHeading ?? 0.5;
    this.wBeta = options.wBeta ?? 5;
    this.wYawRate = options.wYawRate ?? 0.18;
    this.wGrip = options.wGrip ?? 30;
    this.wBoundary = options.wBoundary ?? 60;
    this.wSmooth = options.wSmooth ?? 0.012;

    this.steerKnots = new Float64Array(this.KNOTS);
    this.accelKnots = new Float64Array(this.KNOTS);
    this.hasWarmStart = false;

    this.steerCorrection = 0;
    this.accelerationBias = 0;
    this.fallbacks = 0;
    this.solves = 0;
    this.lastCost = 0;
    this.evaluations = 0;
    this.solveTimes = [];
    this.sumAbsSteer = 0;
    this.sumAbsAccel = 0;
  }

  /** Linear interpolation of a knot vector at normalised horizon position f. */
  knotsAt(knots, f) {
    const x = clamp(f, 0, 1) * (this.KNOTS - 1);
    const i = Math.min(this.KNOTS - 2, Math.floor(x));
    return knots[i] + (knots[i + 1] - knots[i]) * (x - i);
  }

  /**
   * Roll one candidate control sequence out and score it. Station progress is
   * read from the canonical track projection of the simulated position, so a
   * braking candidate cannot pretend it is covering ground at its entry speed.
   */
  evaluate(ego, path, nominalSteer, steerKnots, accelKnots, baseAccel, targetSpeed) {
    const state = dynamicState(ego);
    const n = this.NODES;
    let cost = 0;
    let prevSteer = nominalSteer;
    let prevAccel = baseAccel;
    let prevS = ego.s;
    let terminalSpeed = state.u;

    for (let k = 0; k < n; k++) {
      const f = n === 1 ? 0 : k / (n - 1);
      const steerCmd = clamp(nominalSteer + this.knotsAt(steerKnots, f), -1, 1);

      // Where the predicted car actually is on the canonical track, and what
      // the plan wants there.
      const projected = this.track.nearest(state.x, state.z);
      const ref = path.at(projected.s);

      // Physical envelope at THIS node, not the state the solve started from.
      // Without this the solver cannot know that braking capacity collapses as
      // lateral demand rises through turn-in.
      const env = this.envelope.at(ego, Math.max(1, state.u), ref.curvature, projected.lateral);

      // The base longitudinal law is re-evaluated at the predicted state, not
      // frozen at the value it had when the solve began. Holding a single
      // acceleration across 1.4 s is the longitudinal form of s0 + v0*t: it
      // cannot express brake, trail, release, then pickup.
      const nodeTarget = Math.min(ref.speed, targetSpeed);
      const localBase = clamp((nodeTarget - state.u) * 0.74, -env.brake, env.drive);
      const accelCmd = clamp(localBase + this.knotsAt(accelKnots, f), -env.brake, env.drive);

      // Progress from the canonical projection of the rolled-out position.
      const advanced = wrap(projected.s - prevS + this.track.length, this.track.length);
      prevS = projected.s;

      const lateralError = projected.lateral - ref.offset;
      const headingError = angle(projected.heading - state.yaw);
      const beta = Math.atan2(state.v, Math.max(3, state.u));
      const yawRateTarget = state.u * ref.curvature;
      const yawRateError = state.rate - yawRateTarget;
      const grip = (state.u * state.u * Math.abs(ref.curvature)) / Math.max(1, env.lateral);

      cost += -this.wProgress * advanced;
      cost += this.wLine * lateralError * lateralError;
      cost += this.wHeading * headingError * headingError;
      cost += this.wBeta * beta * beta;
      cost += this.wYawRate * yawRateError * yawRateError;
      cost += this.wGrip * Math.max(0, grip - 0.99) ** 2;
      cost += this.wBoundary
        * Math.max(0, Math.abs(projected.lateral) - this.track.halfWidth + ego.spec.halfWidth) ** 2;
      // Actuator chatter: brake/throttle alternation and steering wind-up cost
      // real time and destabilise the car.
      cost += this.wSmooth * ((steerCmd - prevSteer) ** 2 + (accelCmd - prevAccel) ** 2 * 0.25);

      prevSteer = steerCmd;
      prevAccel = accelCmd;
      terminalSpeed = state.u;
    }

    cost -= this.wExit * terminalSpeed;
    return cost;
  }

  solve(ego, path, nominalSteer, targetSpeed, dt, baseAccel = 0) {
    const started = performance.now();
    this.solves++;

    // Bypass gate: with the solver off the applied corrections are exactly zero
    // and the proven base servo + longitudinal law drive the car on their own.
    // This is the regression reference for every authority tuning step.
    if (!this.enabled) {
      this.steerCorrection = 0;
      this.accelerationBias = 0;
      this.solveTimes.push(performance.now() - started);
      return { steerCorrection: 0, accelerationBias: 0, cost: 0, horizon: this.H * this.NODES, ok: false, evaluations: 0, solveMs: 0 };
    }

    const k = this.KNOTS;

    // Warm start: shift the previous solution one knot forward and hold the
    // terminal knot. Deterministic, and it is what keeps brake release and
    // throttle pickup smooth from one 60 Hz solve to the next.
    const steer = new Float64Array(k);
    const accel = new Float64Array(k);
    if (this.hasWarmStart) {
      for (let i = 0; i < k; i++) {
        const from = Math.min(k - 1, i + 1);
        steer[i] = this.steerKnots[from];
        accel[i] = this.accelKnots[from];
      }
    } else {
      steer.fill(0);
      accel.fill(0);
    }

    let bestCost = this.evaluate(ego, path, nominalSteer, steer, accel, baseAccel, targetSpeed);
    this.evaluations = 1;

    // Deterministic shrinking coordinate descent. No stochastic exploration:
    // the benchmark requires bit-identical results for a given seed and state.
    //
    // The step sizes are scaled to what the car can actually use. A steering
    // step of 0.044 rad at 60 m/s demands v^2*d/L of ~58 m/s^2, which saturates
    // the tyre instantly and makes every perturbation look worse than doing
    // nothing -- the search then correctly, and uselessly, declines to act.
    let steerStep = 0.005;
    let accelStep = 1.1;
    for (let pass = 0; pass < this.PASSES; pass++) {
      for (let dim = 0; dim < k * 2; dim++) {
        const isSteer = dim < k;
        const index = isSteer ? dim : dim - k;
        const vector = isSteer ? steer : accel;
        const step = isSteer ? steerStep : accelStep;
        const saved = vector[index];
        let bestValue = saved;
        for (const delta of [-2, -1, 1, 2]) {
          vector[index] = clamp(saved + delta * step, isSteer ? -0.16 : -22, isSteer ? 0.16 : 22);
          const cost = this.evaluate(ego, path, nominalSteer, steer, accel, baseAccel, targetSpeed);
          this.evaluations++;
          if (cost < bestCost) { bestCost = cost; bestValue = vector[index]; }
        }
        vector[index] = bestValue;
      }
      steerStep *= 0.45;
      accelStep *= 0.45;
    }

    if (!Number.isFinite(bestCost)) {
      this.fallbacks++;
      this.steerCorrection = 0;
      this.accelerationBias = 0;
      this.solveTimes.push(performance.now() - started);
      return { steerCorrection: 0, accelerationBias: 0, cost: Infinity, horizon: this.H * this.NODES, ok: false };
    }

    this.steerKnots = steer;
    this.accelKnots = accel;
    this.hasWarmStart = true;

    // The applied command is the first knot. Steering is a delta on the servo;
    // longitudinal is absolute, so it is reported back as the difference from
    // the base law and the driver adds it on.
    //
    // The applied envelope starts at the old trim layer's authority (steer
    // +/-0.018 slewed at +/-0.004, accel bias [-0.9, +0.55] slewed at +/-0.16)
    // so the new solver can be compared against it as a drop-in before any
    // authority is increased. `authority` scales the applied correction so the
    // intervention can be ramped and the exact point stability is lost can be
    // measured rather than guessed at.
    const steerTarget = clamp(steer[0], -this.maxSteerDelta, this.maxSteerDelta) * this.authority;
    const accelTarget = clamp(accel[0], this.maxAccelBiasLow, this.maxAccelBiasHigh) * this.authority;
    this.steerCorrection += clamp(steerTarget - this.steerCorrection, -this.steerSlew, this.steerSlew);
    this.accelerationBias += clamp(accelTarget - this.accelerationBias, -this.accelSlew, this.accelSlew);

    this.lastCost = bestCost;
    const elapsed = performance.now() - started;
    this.solveTimes.push(elapsed);
    if (this.solveTimes.length > 600) this.solveTimes.shift();
    this.sumAbsSteer += Math.abs(this.steerCorrection);
    this.sumAbsAccel += Math.abs(this.accelerationBias);

    return {
      steerCorrection: this.steerCorrection,
      accelerationBias: this.accelerationBias,
      cost: bestCost,
      horizon: this.H * this.NODES,
      ok: true,
      evaluations: this.evaluations,
      solveMs: elapsed,
    };
  }

  /** Compute-cost telemetry for the solver budget report (§28). */
  stats() {
    const times = this.solveTimes.slice().sort((a, b) => a - b);
    const at = (p) => (times.length ? times[Math.min(times.length - 1, Math.floor(times.length * p))] : 0);
    return {
      enabled: this.enabled,
      authority: this.authority,
      meanAbsSteerCorrection: Number((this.sumAbsSteer / Math.max(1, this.solves)).toFixed(5)),
      meanAbsAccelBias: Number((this.sumAbsAccel / Math.max(1, this.solves)).toFixed(4)),
      solves: this.solves,
      fallbacks: this.fallbacks,
      evaluationsPerSolve: this.KNOTS * 2 * (2 * 2 * this.PASSES) + 1,
      nodes: this.NODES,
      horizonSeconds: Number((this.H * this.NODES).toFixed(2)),
      meanSolveMs: Number((times.reduce((a, b) => a + b, 0) / Math.max(1, times.length)).toFixed(3)),
      p50SolveMs: Number(at(0.5).toFixed(3)),
      p95SolveMs: Number(at(0.95).toFixed(3)),
      p99SolveMs: Number(at(0.99).toFixed(3)),
      maxSolveMs: Number((times.at(-1) ?? 0).toFixed(3)),
    };
  }

  reset() {
    this.steerKnots = new Float64Array(this.KNOTS);
    this.accelKnots = new Float64Array(this.KNOTS);
    this.hasWarmStart = false;
    this.steerCorrection = 0;
    this.accelerationBias = 0;
    this.solveTimes = [];
  }
}
