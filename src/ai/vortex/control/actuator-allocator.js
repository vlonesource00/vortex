import { clamp } from '../../../sim/math.js';
import { TyrePredictor } from './tyre-predictor.js';

/**
 * Turns a required net longitudinal acceleration into throttle and brake.
 *
 * The demand is a physical quantity in m/s^2 derived from the station speed
 * profile, not an instantaneous speed error, so braking follows the planned
 * event instead of reacting to whatever the car is doing right now. The
 * available longitudinal authority is what remains of the friction ellipse
 * after the lateral demand at this station is satisfied.
 *
 * The envelope's drive figure is already net of drag and rolling resistance,
 * so holding speed requires a throttle that exactly cancels them; without that
 * compensation a zero demand coasts and the car bleeds speed on every
 * straight.
 *
 * HOW THE THROTTLE COMMAND IS CHOSEN
 *
 * Two denominators are available for the same demand. Dividing by the
 * friction-ellipse remainder asks for full torque whenever the lateral term is
 * non-zero, because the remainder shrinks faster than the demand does; dividing
 * by what the envelope can actually hold asks only for the torque the model
 * believes is there. The gap between them is the over-ask, and the over-ask is
 * worth real time on a healthy tyre and is what loses the car on a worn one.
 *
 * Every attempt to settle that with a static rule failed, because the answer is
 * not a constant: it depends on what the driven tyres are about to do. So the
 * over-ask is now spent on prediction instead of policy. Between the two
 * endpoints a small set of throttle candidates is rolled forward through the
 * plant's own combined-slip tyre law on a copy of the live wheel state, and the
 * extra torque is taken only when the predicted acceleration is worth more race
 * time than the predicted slip energy and sideslip cost.
 *
 * The value terms are all in seconds of race time, so nothing here is an
 * arbitrary weight:
 *
 *   benefit  = extra acceleration / speed, the time a second of that extra
 *              pace is worth
 *   thermal  = the grip the sustained slip power would cost, priced through
 *              the derivative of the plant's own grip law at the current
 *              temperature - so heat is nearly free on a cool tyre and
 *              expensive on one already past its peak
 *   stability = a share of the measured cost of losing the car, growing as the
 *              predicted sideslip approaches the value the spin actually
 *              reached
 *
 * This is longitudinal only. It scales no lateral limit, no corridor, no
 * candidate score and no speed target, so it is inert in combat: a car with
 * rivals alongside is limited by the same envelope either way.
 */
export class ActuatorAllocator {
  constructor(options = {}) {
    this.predictor = new TyrePredictor({ horizon: options.horizon ?? 0.2 });
    // Measured anchors. The fastest clean lap of the stint peaks at 0.33 rad of
    // body slip; the final-corner spin reaches 1.29 rad. Those two numbers are
    // the scale the stability term is read against, not a tuning knob.
    this.betaClear = options.betaClear ?? 0.33;
    this.betaSpin = options.betaSpin ?? 1.20;
    this.spinCostSeconds = options.spinCostSeconds ?? 10;
    // Below this the two maps agree and there is nothing to decide.
    this.minGap = options.minGap ?? 0.02;
    // Ablation switches (see tools/ab-allocator.mjs).
    this.useThermal = options.useThermal ?? true;
    this.useStability = options.useStability ?? true;
    // Multiplier on the thermal charge, for calibration sweeps. One is the
    // physically derived value (both terms priced over one thermal time
    // constant); larger values price the heat as if it persisted longer.
    this.thermalScale = options.thermalScale ?? 1;
    // 'physical' is the bounded map, 'cheap' is the full-ask map, 'predictive'
    // spends the over-ask only where the rollout says it is worth the cost.
    //
    // The default is 'physical' because of what the ablation measured, not
    // because the prediction is untrusted. Twenty configurations were run -
    // the two endpoints, the five ablations, six thermal scales and seven
    // stability thresholds - and every one that spends the over-ask buys about
    // 0.32 s per fresh lap and pays about 11 s on the final lap of a four-lap
    // stint. Over that stint the bounded map is 10 s faster than anything the
    // prediction can produce, and it is the only configuration that keeps the
    // car on the track. The over-ask that buys the fresh pace is the same
    // over-ask that heats the tyre into the final-corner departure; they are
    // one behaviour, not two, so no state-dependent weighting separates them.
    // See tools/ab-allocator.mjs and tools/sweep-thermal.mjs for the front.
    this.mode = options.mode ?? 'physical';
    this.forcedThrottle = options.forcedThrottle ?? null;
    this.log = options.log ?? null;
  }

  /** Candidate throttles spanning the physical map up to the full-ask map. */
  candidates(tPhys, tCheap) {
    const gap = tCheap - tPhys;
    return [tPhys, tPhys + gap * 0.25, tPhys + gap * 0.5, tPhys + gap * 0.75, tCheap]
      .map(t => clamp(t, 0, 1));
  }

  evaluate(pred, baseline, ego, envelope) {
    const u = Math.max(4, Math.abs(ego.u));
    const slipPowerDelta = pred.slipPowerMean - baseline.slipPowerMean;
    const thermal = this.useThermal
      ? this.predictor.thermalCost(ego, slipPowerDelta)
      : { dGrip: 0, dSurface: 0, dCore: 0, tau: 1 };
    const grip = Math.max(0.4, envelope.mu ?? 1);

    // Both of the first two terms are priced over one thermal time constant,
    // the tyre's own response time taken straight from the plant's thermal
    // ODE. That window is not a tuning knob: it is the period over which an
    // operating point's heat actually becomes grip. Putting both on the same
    // window means the comparison between them needs no conversion factor.
    //
    // Benefit: extra acceleration is extra pace, and extra pace is time. Over
    // one second the car covers u metres at a mean speed dAx/2 higher when it
    // is accelerating dAx harder, so it covers them dAx/(2u) seconds sooner.
    const dAx = pred.ax - baseline.ax;
    const benefit = (dAx / (2 * u)) * thermal.tau;

    // Thermal: the steady-state grip loss the extra slip power would cost at
    // the temperature the tyre is at now, which is the derivative of the
    // plant's own grip law. The sign is kept: a tyre still climbing toward its
    // 85 C peak genuinely earns grip from the extra heat, and one past the peak
    // genuinely pays for every degree it is handed. Turning the negative branch
    // into a flat zero would throw away the one part of the stint where extra
    // slip energy is actually free.
    const thermalCost = this.useThermal
      ? -thermal.dGrip / grip * this.thermalScale * 0.5
      : 0;

    // Stability: predicted sideslip, charged as a share of the measured cost of
    // losing the car. This is a one-off event cost rather than a per-window
    // rate, and deliberately so: a spin is worth about ten seconds and nothing
    // the tyre does smoothly is comparable to it. Squared so the charge is
    // invisible well inside the envelope and rises hard only as the car
    // actually approaches the spin. Note the rollout already sees the degraded
    // tyre, so for the same throttle increase it predicts more force theft and
    // more sideslip on hot worn rubber than on new cold rubber - the thermal
    // state arrives here through the physics rather than a separate penalty.
    const betaAbs = Math.abs(pred.beta1);
    const stability = this.useStability
      ? this.spinCostSeconds
        * clamp((betaAbs - this.betaClear) / (this.betaSpin - this.betaClear), 0, 1) ** 2
      : 0;

    return {
      benefit,
      thermalCost,
      stability,
      dGrip: thermal.dGrip,
      dSurface: thermal.dSurface,
      dCore: thermal.dCore,
      slipPowerDelta,
      J: benefit - thermalCost - stability,
    };
  }

  allocate(requiredAccel, ego, envelope, curvature = 0) {
    const lateralDemand = clamp(ego.speed * ego.speed * Math.abs(curvature) / Math.max(1, envelope.lateral), 0, .985);
    const reserve = Math.sqrt(Math.max(0, 1 - lateralDemand * lateralDemand));
    const drive = Math.max(.35, envelope.drive * reserve);
    const braking = Math.max(1.2, envelope.brake * reserve);
    const drag = Math.max(0, envelope.drag ?? 0) * reserve;
    const accel = clamp(requiredAccel, -braking, drive);
    const demand = accel + drag;

    const held = Math.max(.4, (envelope.drive ?? drive) + drag);
    const cheap = Math.max(.4, drive + drag);
    const tPhys = demand > 0 ? clamp(demand / held, 0, 1) : 0;
    const tCheap = demand > 0 ? clamp(demand / cheap, 0, 1) : 0;

    let throttle = tPhys;
    let decision = null;

    if (this.mode === 'cheap') throttle = tCheap;

    // Only worth predicting where the two maps disagree enough to matter. On a
    // straight the denominators coincide and the physical map is exact.
    const gap = tCheap - tPhys;
    const wheels = ego.wheels;
    if (this.mode === 'predictive' && demand > 0 && gap >= this.minGap && wheels && wheels.length >= 4) {
      const set = this.candidates(tPhys, tCheap);
      const baseline = this.predictor.predict(ego, envelope, tPhys);
      if (baseline) {
        let bestT = tPhys, bestJ = 0, bestTerms = null, bestPred = baseline;
        const evaluations = [];
        for (let i = 1; i < set.length; i++) {
          const c = set[i];
          if (c <= bestT + 1e-4) continue;
          const pred = this.predictor.predict(ego, envelope, c);
          if (!pred) continue;
          // Hard physical constraint, before any cost is considered: the extra
          // longitudinal force may only be paid for out of lateral capacity the
          // rear axle is not already using. This is the friction circle stated
          // in force space, and unlike a sideslip penalty it fires from the
          // force state the car is in now rather than from the misbehaviour a
          // bad choice would eventually produce.
          const steal = Math.abs(pred.fyRearCurrent) - Math.abs(pred.fyRear);
          if (steal > pred.lateralReserve) continue;
          const terms = this.evaluate(pred, baseline, ego, envelope);
          evaluations.push({ throttle: c, pred, terms });
          if (terms.J > bestJ) { bestJ = terms.J; bestT = c; bestTerms = terms; bestPred = pred; }
        }
        if (bestTerms) throttle = bestT;
        if (this.log) {
          decision = {
            tPhys, tCheap, selected: throttle,
            baseline: {
              ax: baseline.ax, kappaMax: baseline.kappaMax, utilMax: baseline.utilMax,
              slipPowerMean: baseline.slipPowerMean, beta: baseline.beta1,
            },
            evaluations: evaluations.map(e => ({
              throttle: e.throttle,
              ax: e.pred.ax,
              kappaMax: e.pred.kappaMax,
              alphaMax: e.pred.alphaMax,
              utilMax: e.pred.utilMax,
              slipPowerMean: e.pred.slipPowerMean,
              tcMean: e.pred.tcMean,
              beta: e.pred.beta1,
              dBeta: e.pred.dBeta,
              dFy: e.pred.dFy,
              benefit: e.terms.benefit,
              thermalCost: e.terms.thermalCost,
              stability: e.terms.stability,
              J: e.terms.J,
            })),
          };
          this.log.push(decision);
          if (this.log.length > 4000) this.log.shift();
        }
      }
    }

    if (this.forcedThrottle !== null) throttle = this.forcedThrottle;

    let brake = accel < 0 ? clamp(-accel / braking, 0, 1) : 0;
    if (brake > .012) throttle = 0;
    // Coast window: a small demand band lets the car roll rather than
    // alternating between throttle and brake at the apex.
    if (Math.abs(requiredAccel) < .12) { throttle = clamp(drag / held, 0, 1); brake = 0; }
    if (ego.fuel <= 0) throttle = 0;

    return {
      throttle, brake, acceleration: accel, lateralDemand, reserve, drive, braking,
      throttlePhysical: tPhys, throttleFullAsk: tCheap, decision,
    };
  }

  reset() { if (this.log) this.log.length = 0; }
}
