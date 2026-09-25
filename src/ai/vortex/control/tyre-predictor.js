import { clamp, damp } from '../../../sim/math.js';

/**
 * Short-horizon driven-tyre rollout.
 *
 * The allocator has to answer "is this extra torque worth it?" before the
 * plant has had a chance to show the slip. That cannot be done from throttle
 * or from tyre temperature alone: the quantity that decides it is the marginal
 * slip energy the extra torque creates, and the lateral force it steals from
 * the same contact patch to do so. Both come out of the plant's own combined-
 * slip tyre law, so the prediction is that law run forward on a copy of the
 * live wheel state.
 *
 * Nothing here touches the vehicle. Every tyre scalar is read into a local
 * object, integrated, and thrown away, so candidate evaluation can never
 * corrupt the state the car is actually driving on.
 *
 * The constants below are lifted verbatim from src/sim/tyre.js and
 * src/sim/vehicle.js. They are not a second opinion about the car, they are
 * the car's own equations evaluated ahead of time.
 */

const AMBIENT = 24;
const CAMBER = -0.035;

/** Grip model shared with the plant (src/sim/tyre.js). */
function tyreGrip(t, load) {
  return 1.48 * clamp(1 - ((t.core - 85) / 105) ** 2, 0.65, 1)
    * clamp(1 - Math.abs(t.pressure - 2.15) * 0.13, 0.8, 1)
    * clamp(1 - 0.13 * Math.log(Math.max(0.1, load / 3300)), 0.68, 1.18)
    * (1 - t.wear * 0.35);
}

/**
 * Marginal grip per degree of core temperature, at the current state.
 *
 * This is the derivative of the plant's own grip law, and it is the whole
 * reason a thermal budget can be a state rather than a switch: at 85 C the
 * tyre is at its grip peak and a degree either way costs almost nothing, while
 * a tyre already past the peak pays for every extra degree it is given. The
 * pressure term rides along because pressure follows core through the ideal
 * gas law in the plant.
 */
export function gripSensitivityToCore(t, load) {
  const core = t.core;
  const dCoreTerm = -2 * (core - 85) / (105 * 105);
  const coreTerm = clamp(1 - ((core - 85) / 105) ** 2, 0.65, 1);
  const pressure = (t.coldPressure + 1.01325) * ((core + 273.15) / (AMBIENT + 273.15)) - 1.01325;
  const dPressure = (t.coldPressure + 1.01325) / (AMBIENT + 273.15);
  const dPressureTerm = pressure > 2.15 ? -0.13 * dPressure : 0.13 * dPressure;
  const pressureTerm = clamp(1 - Math.abs(pressure - 2.15) * 0.13, 0.8, 1);
  const loadTerm = clamp(1 - 0.13 * Math.log(Math.max(0.1, load / 3300)), 0.68, 1.18);
  return 1.48 * loadTerm * (1 - t.wear * 0.35)
    * (dCoreTerm * pressureTerm + coreTerm * dPressureTerm);
}

function cloneTyre(t) {
  return {
    surface: t.surface, core: t.core, coldPressure: t.coldPressure, pressure: t.pressure,
    wear: t.wear, alpha: t.alpha, kappa: t.kappa, fx: t.fx, fy: t.fy,
    utilisation: t.utilisation, slipPower: t.slipPower,
  };
}

/** One combined-slip force evaluation, mirroring src/sim/tyre.js exactly. */
function tyreForce(t, state, dt) {
  const { vx, vy, omega, radius, load, grip } = state;
  const speed = Math.max(2.5, Math.abs(vx));
  t.kappa = damp(t.kappa, clamp((omega * radius - vx) / speed, -2, 2), speed / 0.32, dt);
  t.alpha = damp(t.alpha, Math.atan2(vy, speed), speed / 0.45, dt);
  const peak = Math.max(0, grip * load * tyreGrip(t, load));
  const sx = t.kappa * 10.5, sy = Math.tan(clamp(t.alpha, -1.2, 1.2)) * 8.6;
  const slip = Math.hypot(sx, sy);
  const shape = Math.tanh(slip) * (1 - 0.16 * clamp((slip - 1.4) / 5, 0, 1));
  t.fx = slip > 0.00001 ? peak * shape * sx / slip : 0;
  t.fy = slip > 0.00001 ? -peak * shape * sy / slip : 0;
  t.fy -= CAMBER * load * 0.035;
  if (load < 1) { t.fx = 0; t.fy = 0; }
  t.utilisation = peak > 0 ? Math.hypot(t.fx, t.fy) / peak : 0;
  const sliding = Math.abs(t.fx * (omega * radius - vx)) + Math.abs(t.fy * vy);
  t.slipPower = clamp(sliding, 0, 180000);
  const rolling = load * Math.abs(vx) * 0.012;
  const cooling = (t.surface - AMBIENT) * (23 + Math.abs(vx) * 1.1);
  t.surface = clamp(t.surface + (t.slipPower * 0.55 + rolling - cooling - (t.surface - t.core) * 75) / 6000 * dt, AMBIENT, 210);
  t.core = clamp(t.core + ((t.surface - t.core) * 75 + rolling * 0.3 - (t.core - AMBIENT) * 4) / 18000 * dt, AMBIENT, 170);
  t.pressure = (t.coldPressure + 1.01325) * ((t.core + 273.15) / (AMBIENT + 273.15)) - 1.01325;
  t.wear = clamp(t.wear + t.slipPower * dt * 1.7e-10 * (1 + Math.max(0, t.surface - 115) / 35), 0, 1);
}

export class TyrePredictor {
  /**
   * @param horizon  prediction window in seconds (100-300 ms is the useful band)
   * @param gripMul  track surface grip; constant across candidates so it cancels
   *                 in the comparison, which is all the allocator needs.
   */
  constructor({ horizon = 0.2, gripMul = 1 } = {}) {
    this.horizon = horizon;
    this.gripMul = gripMul;
  }

  /**
   * Roll the two driven wheels forward under a constant throttle command.
   * Pure: reads the observation's wheel snapshot and writes only to locals.
   */
  predict(ego, envelope, throttle) {
    const spec = ego.spec;
    const setup = ego.setup ?? {};
    const wheels = ego.wheels ?? [];
    if (wheels.length < 4) return null;

    const u = Math.max(0.5, ego.u);
    const v = ego.v ?? 0;
    const yawRate = ego.yawRate ?? 0;
    const mass = spec.mass + (ego.fuel ?? 0) * 0.75;
    const inertia = spec.wheelInertia;
    const radius = spec.radius;
    // Full-throttle driven-axle torque, from the envelope's own engine model.
    // This is exactly the plant's `drive` at throttle 1, damage 0, so the
    // prediction sees the same torque curve and gearing the car does.
    const axleTorque = (envelope.driveTorque ?? 3200) * (1 - (ego.damage ?? 0) * 0.28);
    const tcGain = setup.tc > 0 ? setup.tc : 0;
    const tcThreshold = 0.14 - tcGain * 0.009;

    // Rear wheels carry no steering in the plant (Ackermann is front only), so
    // the tyre frame is the body frame rotated by nothing. Wheel placement is
    // fixed chassis geometry (src/sim/vehicle.js resetState) and is not carried
    // on the observation's wheel snapshot, so it is rebuilt from the spec
    // rather than read.
    const rearZ = -spec.frontWeight * spec.wheelbase;
    const driven = [2, 3].map(i => {
      const w = wheels[i];
      const x = (i === 2 ? -1 : 1) * spec.track / 2;
      return {
        tyre: cloneTyre(w.tyre),
        omega: w.omega,
        load: Math.max(0, w.load),
        vx: u - yawRate * x,
        vy: v + yawRate * rearZ,
        x,
        z: rearZ,
        fx0: w.tyre.fx,
        fy0: w.tyre.fy,
      };
    });

    const dt = 1 / 60;
    const steps = Math.max(1, Math.round(this.horizon / dt));
    const sub = 4;

    let kappaMax = 0, alphaMax = 0, utilMax = 0, slipPowerSum = 0, slipEnergy = 0;
    let tcSum = 0, fySum = 0, fxSum = 0, n = 0;

    for (let s = 0; s < steps; s++) {
      // Traction control reads the worst driven wheel, exactly as the plant does.
      const rearSlip = Math.max(driven[0].tyre.kappa, driven[1].tyre.kappa);
      const tc = tcGain > 0
        ? clamp(1 - Math.max(0, rearSlip - tcThreshold) * tcGain * 0.7, 0.18, 1)
        : 1;
      tcSum += tc;

      for (const d of driven) {
        const wheelTorque = throttle * axleTorque * tc * 0.5;
        const state = {
          vx: d.vx, vy: d.vy, omega: d.omega, radius,
          load: d.load, grip: this.gripMul * (spec.tyreGrip ?? 1),
        };
        for (let k = 0; k < sub; k++) {
          const h = dt / sub;
          // The wheel speed must be live inside the sub-stepping: kappa is
          // driven by omega minus road speed, so freezing it here would break
          // the torque-to-slip feedback the whole prediction depends on.
          state.omega = d.omega;
          tyreForce(d.tyre, state, h);
          const unbraked = d.omega + (wheelTorque - d.tyre.fx * radius) / inertia * h;
          d.omega = clamp(Math.sign(unbraked) * Math.max(0, Math.abs(unbraked)), -300, 520);
          slipPowerSum += d.tyre.slipPower;
          slipEnergy += d.tyre.slipPower * h;
          n++;
        }
        kappaMax = Math.max(kappaMax, Math.abs(d.tyre.kappa));
        alphaMax = Math.max(alphaMax, Math.abs(d.tyre.alpha));
        utilMax = Math.max(utilMax, d.tyre.utilisation);
        fySum += d.tyre.fy;
        fxSum += d.tyre.fx;
      }
    }

    const fyCur = driven[0].fy0 + driven[1].fy0;
    // fxSum is the sum over every sub-step of both wheels, so the mean per
    // wheel is fxSum/n and the axle force is twice that.
    const axEstimate = (2 * fxSum / Math.max(1, n)) / mass;

    // Peak force each rear tyre can produce at all (the friction-circle
    // radius), evaluated at the live tyre state. The gap between this and the
    // lateral force the axle is actually carrying is the lateral capacity the
    // extra torque would be spending from.
    const peakRear = driven.reduce((a, d) => {
      const g = tyreGrip(d.tyre, Math.max(0, d.load));
      return a + Math.max(0, this.gripMul * (spec.tyreGrip ?? 1) * d.load * g);
    }, 0);

    // Stability: the extra longitudinal force is taken out of the same contact
    // patch as the lateral force, so predict what the rear axle will no longer
    // be able to hold and what that does to the chassis over the horizon.
    const dFy = fySum / Math.max(1, steps) - fyCur;
    const rearArm = Math.abs(driven[0].z);
    const dYawRate = (dFy * -rearArm) / (spec.yawInertia || 2030) * this.horizon;
    const dV = dFy / mass * this.horizon;
    const beta0 = Math.atan2(v, Math.max(4, u));
    const beta1 = Math.atan2(v + dV, Math.max(4, u));

    return {
      throttle,
      ax: axEstimate,
      kappaMax,
      alphaMax,
      utilMax,
      slipPowerMean: n ? slipPowerSum / n : 0,
      slipEnergy,
      tcMean: tcSum / Math.max(1, steps),
      fyRear: fySum / Math.max(1, steps),
      fyRearCurrent: fyCur,
      peakRear,
      lateralReserve: peakRear - Math.abs(fyCur),
      dFy,
      dYawRate,
      beta0,
      beta1,
      dBeta: beta1 - beta0,
      steps,
    };
  }

  /**
   * Steady-state surface and core rise the plant would settle at if this slip
   * power persisted, and the grip it would cost at the current state.
   *
   * A per-horizon energy charge is useless here - one horizon of wear moves
   * grip by parts in ten million - so the question is answered the way a race
   * engineer would answer it: if I keep driving like this, how much hotter does
   * the tyre get and what is that worth? The surface equilibrium follows
   * directly from the plant's own cooling law, and the core tracks the surface
   * through its 75:4 exchange against ambient.
   */
  /**
   * Race-time cost of the extra slip power, per second of sustained operation.
   *
   * The steady-state surface rise from the plant's own cooling balance is the
   * right magnitude but the wrong window: it is what the tyre would reach if
   * the demand were kept up indefinitely, while the grip consequence of an
   * operating point is realised over the tyre's own thermal response time and
   * then persists. Dividing the steady-state grip loss by that time constant
   * gives a rate, which is what can be compared against the rate at which the
   * extra acceleration buys race time.
   */
  thermalCost(ego, slipPowerDelta) {
    const wheels = ego.wheels ?? [];
    if (wheels.length < 4 || !slipPowerDelta) {
      return { dSurface: 0, dCore: 0, dGrip: 0, tau: 1, costPerSecond: 0 };
    }
    const tyre = wheels[2].tyre;
    const load = Math.max(1, wheels[2].load);
    const speed = Math.max(2.5, Math.abs(ego.u));
    // Cooling balance from the plant's surface ODE, including the surface-core
    // exchange that also carries heat away from the surface.
    const cooling = 23 + speed * 1.1 + 3.8;
    const dSurface = slipPowerDelta * 0.55 / cooling;
    // Core tracks surface at equilibrium through the same ODE's 75:4 exchange
    // against ambient: 0 = 75*(surface - core) - 4*(core - ambient).
    const dCore = (75 / 79) * dSurface;
    const dGrip = gripSensitivityToCore(tyre, load) * dCore;
    // Surface thermal time constant, straight out of the ODE's /6000 mass.
    const tau = 6000 / (23 + speed * 1.1 + 75);
    // Grip-fraction lost per second. The caller turns it into race time.
    const costPerSecond = 0.5 * Math.max(0, -dGrip) / tau;
    return { dSurface, dCore, dGrip, tau, costPerSecond };
  }
}
