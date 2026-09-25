import { predictChassis } from '../../../sim/performance.js';

export function dynamicState(ego) {
  return { x: ego.x, z: ego.z, yaw: ego.yaw, rate: ego.yawRate, u: Math.max(1, ego.u), v: ego.v,
    steering: ego.steering, frontForce: ego.wheels[0].tyre.fy + ego.wheels[1].tyre.fy,
    rearForce: ego.wheels[2].tyre.fy + ego.wheels[3].tyre.fy };
}

export function rollout(state, steering, acceleration, envelope, ego, dt) {
  return predictChassis(state, steering, acceleration, { ...envelope, frontMu: envelope.mu, rearMu: envelope.mu,
    lateralFraction: .9 }, ego.spec.mass + ego.fuel * .75, ego.setup.brakeBias, dt, ego.spec);
}
