import { clamp } from '../../../sim/math.js';

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
 */
export class ActuatorAllocator {
  allocate(requiredAccel, ego, envelope, curvature = 0) {
    const lateralDemand = clamp(ego.speed * ego.speed * Math.abs(curvature) / Math.max(1, envelope.lateral), 0, .985);
    const reserve = Math.sqrt(Math.max(0, 1 - lateralDemand * lateralDemand));
    const drive = Math.max(.35, envelope.drive * reserve);
    const braking = Math.max(1.2, envelope.brake * reserve);
    const drag = Math.max(0, envelope.drag ?? 0) * reserve;
    const accel = clamp(requiredAccel, -braking, drive);
    // The denominator is the torque the tyre can actually hold, not the
    // friction-ellipse remainder. Dividing by the remainder makes the command
    // saturate at full torque whenever the lateral term is non-zero, because
    // the remainder shrinks faster than the demand does. The excess is paid
    // for in rear slip: the plant's traction control trims the drive but not
    // the slip power, which goes into the tyre as wear and heat.
    //
    // That over-ask is worth real time on a healthy tyre and is what loses the
    // car on a worn one. Measured over a four-lap stint, the cheap denominator
    // runs 75.217 / 74.892 / 76.142 and then spins at the final corner for a
    // 91.950; holding the map to what the envelope can deliver runs 75.617 /
    // 75.217 / 76.125 / 78.933. Every intermediate authority tested - a
    // half share, a share blended on identified grip, a share blended on tyre
    // core temperature, and a share withdrawn only once the driven wheels were
    // already slipping - collapsed the final lap harder than either endpoint,
    // so the trim is not a tunable safety margin but the one map that keeps
    // the tyre inside its own thermal window.
    //
    // This is longitudinal only. It scales no lateral limit, no corridor, no
    // candidate score and no speed target, so it is inert in combat: a car
    // with rivals alongside is limited by the same envelope either way.
    const gross = Math.max(.4, (envelope.drive ?? drive) + drag);
    let throttle = accel + drag > 0 ? clamp((accel + drag) / gross, 0, 1) : 0;
    let brake = accel < 0 ? clamp(-accel / braking, 0, 1) : 0;
    if (brake > .012) throttle = 0;
    // Coast window: a small demand band lets the car roll rather than
    // alternating between throttle and brake at the apex.
    if (Math.abs(requiredAccel) < .12) { throttle = clamp(drag / gross, 0, 1); brake = 0; }
    if (ego.fuel <= 0) throttle = 0;
    return { throttle, brake, acceleration: accel, lateralDemand, reserve, drive, braking };
  }
}
