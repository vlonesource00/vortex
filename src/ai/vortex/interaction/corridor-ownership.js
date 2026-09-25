import { clamp, wrap } from '../../../sim/math.js';

export class CorridorOwnership {
  constructor(track) { this.track = track; this.owners = new Map(); }
  update(ego, opponents) {
    for (const rival of opponents) {
      const ds = wrap(rival.s - ego.s + this.track.length / 2, this.track.length) - this.track.length / 2;
      const longitudinalOverlap = Math.abs(ds) < ego.spec.halfLength + rival.spec.halfLength + .8;
      const lateralOverlap = Math.abs(rival.lateral - ego.lateral) < ego.spec.halfWidth + rival.spec.halfWidth + .85;
      const current = this.owners.get(rival.id);
      if (longitudinalOverlap && lateralOverlap) this.owners.set(rival.id, {
        lateral: current?.lateral ?? ego.lateral, since: current?.since ?? 0,
        flank: current?.flank ?? Math.sign(rival.lateral - ego.lateral || 1), clearTicks: 0,
      });
      else if (current) {
        current.clearTicks++;
        if (current.clearTicks >= 4 && Math.abs(ds) > ego.spec.halfLength + rival.spec.halfLength + 1.4)
          this.owners.delete(rival.id);
      }
    }
    for (const id of this.owners.keys()) if (!opponents.some(rival => rival.id === id)) this.owners.delete(id);
    return [...this.owners.entries()].map(([id, owner]) => ({ id, ...owner }));
  }
  constrain(candidate, opponents) {
    for (const owner of this.owners.values()) {
      const min = clamp(owner.lateral - 1.55, -this.track.halfWidth + 1.15, this.track.halfWidth - 1.15);
      const max = clamp(owner.lateral + 1.55, -this.track.halfWidth + 1.15, this.track.halfWidth - 1.15);
      candidate.ownership = candidate.points.some(point => point.s < this.track.length && point.offset > min && point.offset < max);
    }
    return candidate;
  }
}
