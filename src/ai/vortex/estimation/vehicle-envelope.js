import { clamp, damp } from '../../../sim/math.js';
import { tyreGrip } from '../../../sim/tyre.js';
import { EnvelopeModel } from '../atlas/envelope.js';

/**
 * Diagnostic switch, safe in both Node and the browser bundle. `process` does
 * not exist in the page, and touching it there throws every frame.
 */
const flag = (name) => typeof process !== 'undefined' && Boolean(process.env?.[name]);

/**
 * Online vehicle-envelope identification.
 *
 * The physical prior is the validated EnvelopeModel derived from the frozen
 * plant. Telemetry then modulates it with three bounded scales: grip, braking
 * and drive. Every update is rate limited and clipped so one anomalous sample
 * cannot destroy the model, and the estimate only moves on settled, clean,
 * single-axis measurements.
 */
export class VehicleEnvelope {
  constructor(options = {}) {
    this.model = options.model ?? new EnvelopeModel(options);
    this.muScale = 1;
    this.brakeScale = 1;
    this.driveScale = 1;
    this.muReference = null;
    this.planGrip = 1;
    this.samples = 0;
    this.confidence = 0;
    this.lastReason = 'initialising';
    // ---- axle-level stability state (observability only) ----
    // muScale is the mean of all four tyres, so an axle that is running out of
    // capability while the other is fine is invisible in it. These separate
    // the two axles so the rear limit can be seen before the car is lost.
    this.frontGripScale = 1;
    this.rearGripScale = 1;
    this.rearToFrontGrip = 1;
    this.frontReserve = 1;
    this.rearReserve = 1;
    this.frontUtilisation = 0;
    this.rearUtilisation = 0;
    this.yawGenerationReserve = 1;
    this.yawSettlementReserve = 1;
  }

  /**
   * Per-axle capability from the plant's own tyre law.
   *
   * `tyre.utilisation` is exactly hypot(fx, fy) / peak inside tyreForce, so
   * the live peak force is recoverable as used / utilisation and no capacity
   * threshold has to be invented here.
   */
  updateAxles(wheels) {
    const axle = (a, b) => {
      const ws = [wheels[a], wheels[b]];
      let grip = 0, peak = 0, used = 0;
      for (const w of ws) {
        if (!w?.tyre) continue;
        grip += tyreGrip(w.tyre, Math.max(1500, w.load || 3300));
        const u = w.tyre.utilisation ?? 0;
        const f = Math.hypot(w.tyre.fx ?? 0, w.tyre.fy ?? 0);
        used += f;
        peak += u > 1e-6 ? f / u : 0;
      }
      return {
        gripScale: grip / 2,
        peak, used,
        reserve: peak > 0 ? clamp((peak - used) / peak, 0, 1) : 1,
        utilisation: peak > 0 ? clamp(used / peak, 0, 1) : 0,
      };
    };
    const f = axle(0, 1), r = axle(2, 3);
    this.frontGripScale = f.gripScale;
    this.rearGripScale = r.gripScale;
    this.rearToFrontGrip = clamp(r.gripScale / Math.max(0.2, f.gripScale), 0.4, 1.4);
    this.frontReserve = f.reserve;
    this.rearReserve = r.reserve;
    this.frontUtilisation = f.utilisation;
    this.rearUtilisation = r.utilisation;
    // Front lateral reserve is what rotates the car; rear lateral reserve is
    // what settles it. Read against the same live capability, not a constant.
    this.yawGenerationReserve = f.reserve * this.rearToFrontGrip;
    this.yawSettlementReserve = r.reserve;
  }

  /** Bounded identification from the car's own measured response. */
  update(ego, dt) {
    if (flag('VORTEX_NO_ADAPT')) { this.confidence = 1; return this; }
    const wheels = ego.wheels ?? [];
    if (wheels.length === 4) {
      const tyreEstimate = wheels.reduce((sum, wheel) => sum + tyreGrip(wheel.tyre, Math.max(1500, wheel.load || 3300)), 0) / 4;
      const reference = tyreGrip(this.model.tyre, 3300);
      const ratio = clamp(tyreEstimate / Math.max(0.2, reference), 0.72, 1.22);
      // Anchor the stint at the very first tyre reading so the plan tracks
      // degradation rather than the small constant bias of the identification.
      if (this.muReference === null) this.muReference = ratio;
      this.muScale = damp(this.muScale, ratio, 0.35, dt);
      this.updateAxles(wheels);
    }

    const settled = Math.abs(ego.ay) < 2.2 && Math.abs(ego.lateral) < 6 && ego.impact < 0.01 && ego.zone === 'asphalt';

    // Braking identification: hard, straight, clean deceleration only.
    if (settled && ego.controls.brake > 0.8 && ego.speed > 18 && ego.ax < -3) {
      const predicted = this.model.brakeAt(ego.speed) * this.muScale;
      const observed = clamp(-ego.ax / Math.max(1, predicted), 0.78, 1.14);
      this.brakeScale = damp(this.brakeScale, observed, 0.22, dt);
      this.samples += dt;
      this.lastReason = 'braking';
    } else if (settled && ego.controls.throttle > 0.7 && ego.speed < 55 && ego.ax > 1.2) {
      const predicted = this.model.driveAt(ego.speed) * this.muScale;
      const observed = clamp(ego.ax / Math.max(1, predicted), 0.78, 1.16);
      this.driveScale = damp(this.driveScale, observed, 0.16, dt);
      this.samples += dt;
      this.lastReason = 'traction';
    } else {
      // Lateral identification from sustained cornering load.
      if (settled && Math.abs(ego.ay) > 4 && ego.speed > 8) {
        const predicted = this.model.lateralAt(ego.speed) * this.muScale;
        const observed = clamp(Math.abs(ego.ay) / Math.max(1, predicted), 0.8, 1.08);
        this.muScale = damp(this.muScale, clamp(this.muScale * observed, 0.78, 1.12), 0.12, dt);
        this.samples += dt;
        this.lastReason = 'lateral';
      }
    }

    this.muScale = clamp(this.muScale, 0.78, 1.12);
    this.brakeScale = clamp(this.brakeScale, 0.8, 1.1);
    this.driveScale = clamp(this.driveScale, 0.8, 1.1);
    this.confidence = 1 - Math.exp(-this.samples / 10);
    // The offline oracle is solved for one grip level; what the runtime needs
    // is the change in grip since the stint began, because the stored profile
    // already carries the solved level. The plan modulation is deliberately
    // slow: a per-frame grip figure would make the speed target chatter and
    // the longitudinal loop chase noise instead of driving the car.
    const degraded = this.muReference === null ? 1 : clamp(this.muScale / this.muReference, 0.55, 1.45);
    this.planGrip = damp(this.planGrip, degraded, 0.25, dt);
    this.model.setMuScale(this.planGrip);
    return this;
  }

  /** Combined physical + identified envelope at a given operating point. */
  at(ego, speed, curvature, lateral = ego?.lateral ?? 0) {
    const v = Math.max(0, speed);
    const model = this.model;
    const lateralCapacity = model.lateralAt(v);
    const brake = clamp(model.brakeAt(v) * this.brakeScale, 4, 30);
    const drive = clamp(model.driveAt(v) * this.driveScale, 0.2, 12);
    const utilisation = clamp(v * v * Math.abs(curvature) / Math.max(1, lateralCapacity), 0, 0.99);
    const off = Math.abs(lateral) > (ego?.trackHalfWidth ?? 8.2) - 0.9;
    return {
      mu: this.muScale,
      // Grip remaining relative to the start of the stint. Exposed as a
      // diagnostic: measured over a stint this is what separates a tyre that
      // can still carry the cornering demand from one that has gone away.
      gripHealth: this.planGrip,
      // Full-throttle driven-axle torque in N.m at this speed. This is the
      // plant's own `drive` term at throttle 1 (engineAccel is force/mass, so
      // force * radius is the axle torque), which is what the tyre rollout
      // needs to predict slip rather than infer it after the fact.
      driveTorque: model.engineAccel(v) * model.mass * model.spec.radius,
      lateral: lateralCapacity,
      brake,
      drive,
      drag: model.parasiticAccel(v),
      utilisation,
      reserve: model.reserve(utilisation),
      speedLimit: Math.min(82, model.cornerSpeedAt(curvature)),
      throttleLimit: off ? 0.35 : model.reserve(utilisation),
      // Axle-level stability state. Diagnostics: nothing downstream is
      // required to read them, and they are not wired into any control law.
      frontReserve: this.frontReserve,
      rearReserve: this.rearReserve,
      rearToFrontGrip: this.rearToFrontGrip,
      yawGenerationReserve: this.yawGenerationReserve,
      yawSettlementReserve: this.yawSettlementReserve,
    };
  }

  get state() {
    return {
      muScale: this.muScale,
      brakeScale: this.brakeScale,
      driveScale: this.driveScale,
      confidence: this.confidence,
      source: this.lastReason,
      frontGripScale: this.frontGripScale,
      rearGripScale: this.rearGripScale,
      rearToFrontGrip: this.rearToFrontGrip,
      frontReserve: this.frontReserve,
      rearReserve: this.rearReserve,
      frontUtilisation: this.frontUtilisation,
      rearUtilisation: this.rearUtilisation,
      yawGenerationReserve: this.yawGenerationReserve,
      yawSettlementReserve: this.yawSettlementReserve,
    };
  }

  reset() {
    this.muScale = 1; this.brakeScale = 1; this.driveScale = 1;
    this.muReference = null;
    this.planGrip = 1;
    this.samples = 0; this.confidence = 0; this.lastReason = 'initialising';
    this.frontGripScale = 1; this.rearGripScale = 1; this.rearToFrontGrip = 1;
    this.frontReserve = 1; this.rearReserve = 1;
    this.frontUtilisation = 0; this.rearUtilisation = 0;
    this.yawGenerationReserve = 1; this.yawSettlementReserve = 1;
    this.model.setMuScale(1);
  }
}
