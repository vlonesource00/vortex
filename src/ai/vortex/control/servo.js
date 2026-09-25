import { angle, clamp, damp } from '../../../sim/math.js';
import { pathCurvature } from '../../../sim/path-geometry.js';

/**
 * 120 Hz steering servo.
 *
 * Pure pursuit with a speed-scaled lookahead, plus slip compensation, a
 * lateral tracking term, a yaw-rate rotation trim and a curvature feed-forward.
 * The law is deliberately cheap and deterministic: the optimiser above it
 * proposes a corridor, and this loop holds the car on it.
 */
export class VehicleServo {
  constructor() { this.steer = 0; this.lastTarget = null; this.lookahead = 12; }

  steerTo(ego, path, track, optimizerCorrection = 0) {
    // A long pursuit lookahead makes the car cut every apex, but a short one
    // amplifies steering chatter through the actuator lag. The vehicle model
    // already filters steering at 12 rad/s, so this loop keeps only enough
    // smoothing to stop discrete plan jumps from reaching the wheels.
    const lookahead = clamp(5.5 + ego.speed * 0.4, 8, 35);
    const target = path.at(ego.s + lookahead);
    const dx = target.x - ego.x, dz = target.z - ego.z;
    const localX = dx * Math.cos(ego.yaw) - dz * Math.sin(ego.yaw);
    const pursuit = Math.atan2(2 * ego.spec.wheelbase * localX, Math.max(5, dx * dx + dz * dz));

    const slip = Math.atan2(ego.v, Math.max(4, ego.u));
    // The steering actuator and the vehicle's own steering filter together lag
    // the command by roughly 0.14 s, so the curvature the wheels will actually
    // be following is the one that far ahead, not the one under the car.
    const lag = ego.speed * 0.09;
    const here = path.at(ego.s);
    const ahead = path.at(ego.s + 3 + lag);
    const further = path.at(ego.s + 6 + lag);
    const localCurvature = pathCurvature(path.at(ego.s + lag), ahead, further);

    const trackingError = ego.lateral - here.offset;
    const errorRate = (trackingError - (this.prevError ?? trackingError)) * 120;
    this.prevError = trackingError;
    const tracking = clamp(-Math.atan2(trackingError * 0.85, Math.max(12, ego.speed)), -0.05, 0.05);
    const damping = clamp(-errorRate * 0.0012, -0.018, 0.018);
    const rotation = clamp(
      (localCurvature * ego.speed - ego.yawRate) * ego.spec.wheelbase / Math.max(8, ego.speed) * 2.5,
      -0.035, 0.035,
    );
    // The feed-forward is the exact steering angle the plan's curvature needs.
    // Under-weighting it forces pure pursuit to make up the difference, and
    // pure pursuit cuts every apex it points at, which is the standing 2 m
    // line error.
    const feedforward = Math.atan(ego.spec.wheelbase * localCurvature) * 0.55;
    const yawError = angle(target.heading - ego.yaw) * 0.1;

    const raw = pursuit + slip * 0.85 + tracking + damping + rotation + feedforward + yawError + optimizerCorrection;
    const desired = raw / ego.spec.steeringLock;
    this.steer = damp(this.steer, clamp(desired, -1, 1), 45, 1 / 120);
    this.lastTarget = target;
    this.lookahead = lookahead;
    this.terms = {
      pursuit, slip: slip * 0.85, tracking, damping, rotation, feedforward, yawError,
      correction: optimizerCorrection, raw, desired, trackingError, localX,
      hereOffset: here.offset, targetOffset: target.offset,
    };
    return { steer: this.steer, target, lookahead };
  }

  reset() { this.steer = 0; this.lastTarget = null; this.lookahead = 12; this.prevError = null; }
}
