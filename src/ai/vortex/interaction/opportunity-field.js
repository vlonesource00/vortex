import { clamp, wrap } from '../../../sim/math.js';

/** Shared progress, retained-pass, exit-energy, contact and legality value. */
export class OpportunityField {
  constructor(track) { this.track = track; }
  score(path, ego, opponents, occupancy, ownership = []) {
    let cost = 0, time = 0, distance = 0, minClearance = 99, interactions = 0, exitSpeed = ego.speed;
    for (let i = 0; i < path.points.length; i++) {
      const p = path.points[i], prior = path.points[i - 1];
      if (prior) { const ds = Math.max(.1, p.distance - prior.distance); distance += ds; time += ds / Math.max(5, (p.speed + prior.speed) * .5); }
      p.time = time; exitSpeed = p.speed;
      const legal = this.track.halfWidth - ego.spec.halfWidth - .14;
      cost += Math.max(0, Math.abs(p.offset) - legal) ** 2 * 900;
      cost += Math.max(0, p.demand - .99) ** 2 * 80;
      cost += Math.abs(p.curvature) * Math.max(0, p.speed ** 2 * Math.abs(p.curvature) / Math.max(1, p.lateralLimit) - .82) * .035;
      for (const rival of opponents) {
        const predicted = occupancy.at(rival, time);
        let ds = wrap(predicted.s - p.s + this.track.length / 2, this.track.length) - this.track.length / 2;
        const along = Math.max(0, Math.abs(ds) - (ego.spec.halfLength + predicted.halfLength + .65));
        const lateral = Math.max(0, Math.abs(predicted.lateral - p.offset) - (ego.spec.halfWidth + predicted.halfWidth + .34));
        const clearance = Math.hypot(along, lateral); minClearance = Math.min(minClearance, clearance);
        const collision = Math.exp(-((along / 2.6) ** 2 + (lateral / .8) ** 2));
        const energy = Math.abs((ego.speed - predicted.speed) * Math.sign(ds || 1));
        cost += collision * (175 + Math.min(120, energy * 13));
        if (collision > .12) interactions++;
        const flank = Math.abs(ds) < 12 && Math.abs(p.offset - predicted.lateral) > ego.spec.halfWidth + predicted.halfWidth;
        if (flank && distance > 25) cost -= Math.min(2.5, p.speed * .025); // retained position has downstream value.
      }
      for (const owner of ownership) if (Math.abs(p.offset - owner.lateral) < 1.25) cost += 20;
    }
    const progressReward = distance / Math.max(.1, time);
    const exitValue = Math.max(0, exitSpeed - ego.speed) * (path.points.at(-1)?.s > 0 ? .12 : .04);
    cost += time * 13 - progressReward * .11 - exitValue;
    path.score = cost; path.time = time; path.minClearance = minClearance;
    path.interactions = interactions; path.exitSpeed = exitSpeed; path.progressReward = progressReward;
    return cost;
  }
}
