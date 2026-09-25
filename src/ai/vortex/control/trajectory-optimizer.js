import { angle, clamp, damp, wrap } from '../../../sim/math.js';
import { dynamicState, rollout } from './dynamic-model.js';

/** Small bounded shooting search over coupled steering and longitudinal inputs. */
export class TrajectoryOptimizer {
  constructor(track, envelope) { this.track = track; this.envelope = envelope; this.steerCorrection = 0; this.accelerationBias = 0; this.fallbacks = 0; this.solves = 0; this.lastCost = 0; }
  solve(ego, path, nominalSteer, targetSpeed, dt) {
    const controls = [-.018, 0, .018], longitudinal = [-.9, 0, .55];
    const h = .15, horizon = 8;
    let best = Infinity, selectedSteer = 0, selectedAccel = 0;
    const envelope = this.envelope.at(ego, ego.speed, ego.curvature, ego.lateral);
    for (const steerDelta of controls) for (const bias of longitudinal) {
      const state = dynamicState(ego); let cost = 0, command = nominalSteer;
      for (let step = 0; step < horizon; step++) {
        const t = (step + 1) * h;
        const s = wrap(ego.s + Math.max(0, ego.speed) * t, this.track.length), target = path.at(s);
        const setpoint = clamp(nominalSteer + steerDelta * (step < 3 ? 1 : .45), -1, 1);
        command = damp(command, setpoint, 12, h);
        const speedError = Math.min(target.speed, targetSpeed + Math.max(0, target.speed - ego.speed) * Math.min(1, t * .35)) - state.u;
        const accel = clamp(speedError * .74 + bias, -envelope.brake, envelope.drive);
        rollout(state, command * ego.spec.steeringLock, accel, envelope, ego, h);
        const projected = this.track.nearest(state.x, state.z);
        const ref = path.at(projected.s);
        const lateralError = projected.lateral - ref.offset;
        const headingError = angle(projected.heading - state.yaw);
        const slip = Math.atan2(state.v, Math.max(3, state.u));
        const grip = state.u * state.u * Math.abs(ref.curvature) / Math.max(1, envelope.lateral);
        cost += lateralError * lateralError * (.7 + t) + headingError ** 2 * 25 + slip ** 2 * 16
          + (state.u - target.speed) ** 2 * .012 + Math.max(0, grip - .98) ** 2 * 140
          + Math.max(0, Math.abs(projected.lateral) - this.track.halfWidth + ego.spec.halfWidth) ** 2 * 150;
      }
      if (cost < best) { best = cost; selectedSteer = steerDelta; selectedAccel = bias; }
    }
    if (!Number.isFinite(best)) { this.fallbacks++; this.steerCorrection = 0; this.accelerationBias = 0; }
    else {
      this.steerCorrection += clamp(selectedSteer - this.steerCorrection, -.004, .004);
      this.accelerationBias += clamp(selectedAccel - this.accelerationBias, -.16, .16);
    }
    this.solves++; this.lastCost = best;
    return { steerCorrection: this.steerCorrection, accelerationBias: this.accelerationBias, cost: best, horizon: h * horizon };
  }
}
