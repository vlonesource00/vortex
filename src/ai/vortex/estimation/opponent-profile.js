import { clamp, damp } from '../../../sim/math.js';

export class OpponentProfile {
  constructor(id) { this.id = id; this.brakeTendency = 0; this.defensive = .5; this.overlapHold = .5; this.exitQuality = .5; this.reactionDelay = .28; this.samples = 0; }
  update(previous, current, dt) {
    if (!previous || dt <= 0) return this;
    const alpha = clamp(dt * .18, .002, .035), brake = current.controls?.brake ?? 0;
    this.brakeTendency = damp(this.brakeTendency, brake > .45 ? 1 : 0, .16, dt);
    const qdot = (current.lateral - previous.lateral) / dt;
    this.defensive = damp(this.defensive, Math.abs(qdot) > .75 ? 1 : .48, 1.7, dt);
    this.overlapHold = damp(this.overlapHold, Math.abs(current.lateral - previous.lateral) < .18 ? .72 : .45, .65, dt);
    this.exitQuality = damp(this.exitQuality, clamp((current.speed - previous.speed) / Math.max(.1, dt) / 4 + .5, 0, 1), .45, dt);
    this.reactionDelay = clamp(this.reactionDelay * (1 - alpha) + .28 * alpha, .12, .65);
    this.samples++; return this;
  }
}
