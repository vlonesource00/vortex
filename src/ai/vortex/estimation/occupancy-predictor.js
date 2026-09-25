import { clamp, wrap } from '../../../sim/math.js';

export class OccupancyPredictor {
  constructor(track) { this.track = track; }
  at(opponent, time) {
    const t = Math.max(0, time), acceleration = clamp(opponent.acceleration ?? 0, -8, 4);
    const speed = clamp((opponent.longitudinalSpeed ?? opponent.speed) + acceleration * t, 0, 85);
    const distance = (opponent.longitudinalSpeed ?? opponent.speed) * t + .5 * acceleration * t * t;
    const beliefs = opponent.hypotheses ?? { hold: 1 };
    const coverDirection = Math.sign(opponent.lateralSpeed || opponent.lateral || 1);
    return {
      s: wrap(opponent.s + distance, this.track.length), lateral: clamp(opponent.lateral + (opponent.lateralSpeed ?? 0) * Math.min(t, .7), -10, 10),
      speed, halfLength: opponent.spec.halfLength, halfWidth: opponent.spec.halfWidth,
      branches: [
        { weight: beliefs.hold ?? 1, s: wrap(opponent.s + distance, this.track.length), lateral: opponent.lateral },
        { weight: beliefs.cover ?? 0, s: wrap(opponent.s + distance, this.track.length), lateral: clamp(opponent.lateral + coverDirection * Math.min(1.2, t * 1.5), -7, 7) },
        { weight: (beliefs.lateBrake ?? 0) + (beliefs.earlyBrake ?? 0), s: wrap(opponent.s + distance - (opponent.longitudinalSpeed ?? speed) * t * .12, this.track.length), lateral: opponent.lateral },
      ],
    };
  }
}
