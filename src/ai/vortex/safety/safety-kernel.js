import { clamp, wrap } from '../../../sim/math.js';

/** Reluctant last resort for imminent high-energy impact or unrecoverable edge motion. */
export class SafetyKernel {
  constructor(track) { this.track = track; this.interventions = 0; this.reason = 'clear'; }
  evaluate(ego, opponents, dt) {
    let intervention = null;
    const lateralVelocity = ego.vx * ego.nx + ego.vz * ego.nz;
    const edge = this.track.halfWidth - ego.spec.halfWidth;
    if (Math.abs(ego.lateral) > edge - .12 && Math.sign(ego.lateral) * lateralVelocity > 3.2) {
      intervention = { steer: clamp(-Math.sign(ego.lateral) * .34, -.55, .55), throttle: 0,
        brake: ego.speed > 17 ? .32 : 0, reverse: false, reason: 'edge trajectory' };
    }
    for (const rival of opponents) {
      const ds = wrap(rival.s - ego.s + this.track.length / 2, this.track.length) - this.track.length / 2;
      const relative = ego.speed - rival.speed, ttc = relative > 0 ? Math.max(0, ds) / relative : Infinity;
      const lateralClosing = Math.abs(rival.lateral - ego.lateral) < ego.spec.halfWidth + rival.spec.halfWidth + .15;
      if (lateralClosing && ds >= 0 && ds < ego.spec.halfLength + rival.spec.halfLength + 1.1 && relative > 5 && ttc < .24) {
        intervention = { steer: 0, throttle: 0, brake: 1, reverse: false, reason: 'imminent high-energy contact' }; break;
      }
    }
    if (intervention) { this.interventions++; this.reason = intervention.reason; }
    else this.reason = 'clear';
    return intervention;
  }
  reset() { this.interventions = 0; this.reason = 'clear'; }
}
