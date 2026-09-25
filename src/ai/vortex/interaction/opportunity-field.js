import { clamp, wrap } from '../../../sim/math.js';

/**
 * Shared progress, retained-pass, exit-energy, contact and legality value.
 *
 * The scoring terms are unchanged from the proven baseline: a time-unit
 * reformulation of this objective was implemented and measured against the
 * slow-rival fixture, and it LOST time on every variant tried (see
 * tools/fixture-combat.mjs and the block report). Per the block's own rule
 * that measurement decides, the original weighting is retained.
 *
 * What is new is the decomposition. Every candidate now reports exactly which
 * term produced its score, so "why did Q0 win over the open flank" is
 * answerable in units rather than by inspection. One record score unit is
 * 1/13 of a second, because the trajectory term is `time * 13`.
 */
export class OpportunityField {
  constructor(track, atlas = null) { this.track = track; this.atlas = atlas; }

  score(path, ego, opponents, occupancy, ownership = []) {
    let cost = 0, time = 0, distance = 0, minClearance = 99, interactions = 0, exitSpeed = ego.speed;
    let legality = 0, grip = 0, contact = 0, flankReward = 0, ownershipCost = 0, trafficLimited = 0;
    const legal = this.track.halfWidth - ego.spec.halfWidth - 0.14;

    for (let i = 0; i < path.points.length; i++) {
      const p = path.points[i], prior = path.points[i - 1];
      if (prior) {
        const ds = Math.max(.1, p.distance - prior.distance);
        distance += ds;
        time += ds / Math.max(5, (p.speed + prior.speed) * .5);
        if ((p.freeSpeedLimit ?? p.speed) > p.speed + 0.25) trafficLimited++;
      }
      p.time = time;
      exitSpeed = p.speed;

      const cLegal = Math.max(0, Math.abs(p.offset) - legal) ** 2 * 900;
      const cGrip = Math.max(0, p.demand - .99) ** 2 * 80
        + Math.abs(p.curvature) * Math.max(0, p.speed ** 2 * Math.abs(p.curvature) / Math.max(1, p.lateralLimit) - .82) * .035;
      legality += cLegal;
      grip += cGrip;
      cost += cLegal + cGrip;

      for (const rival of opponents) {
        const predicted = occupancy.at(rival, time);
        let ds = wrap(predicted.s - p.s + this.track.length / 2, this.track.length) - this.track.length / 2;
        const along = Math.max(0, Math.abs(ds) - (ego.spec.halfLength + predicted.halfLength + .65));
        const lateral = Math.max(0, Math.abs(predicted.lateral - p.offset) - (ego.spec.halfWidth + predicted.halfWidth + .34));
        const clearance = Math.hypot(along, lateral);
        minClearance = Math.min(minClearance, clearance);
        const collision = Math.exp(-((along / 2.6) ** 2 + (lateral / .8) ** 2));
        const energy = Math.abs((ego.speed - predicted.speed) * Math.sign(ds || 1));
        const cContact = collision * (175 + Math.min(120, energy * 13));
        contact += cContact;
        cost += cContact;
        if (collision > .12) interactions++;
        const flank = Math.abs(ds) < 12 && Math.abs(p.offset - predicted.lateral) > ego.spec.halfWidth + predicted.halfWidth;
        if (flank && distance > 25) {
          const reward = Math.min(2.5, p.speed * .025);
          flankReward += reward;
          cost -= reward;
        }
      }
      for (const owner of ownership) {
        if (Math.abs(p.offset - owner.lateral) < 1.25) { ownershipCost += 20; cost += 20; }
      }
    }

    const progressReward = distance / Math.max(.1, time);
    const exitValue = Math.max(0, exitSpeed - ego.speed) * (path.points.at(-1)?.s > 0 ? .12 : .04);
    const progressTerm = -progressReward * .11;
    const exitTerm = -exitValue;
    cost += time * 13 + progressTerm + exitTerm;

    path.score = cost;
    path.time = time;
    path.minClearance = minClearance;
    path.interactions = interactions;
    path.exitSpeed = exitSpeed;
    path.progressReward = progressReward;
    path.trafficLimitedFraction = Number((trafficLimited / Math.max(1, path.points.length - 1)).toFixed(3));
    // §5 decomposition. Values are in record score units; divide by 13 for
    // seconds of equivalent race time.
    path.breakdown = {
      timeTerm: Number((time * 13).toFixed(2)),
      progressTerm: Number(progressTerm.toFixed(2)),
      exitTerm: Number(exitTerm.toFixed(2)),
      contact: Number(contact.toFixed(2)),
      legality: Number(legality.toFixed(2)),
      grip: Number(grip.toFixed(2)),
      ownership: Number(ownershipCost.toFixed(2)),
      flankReward: Number((-flankReward).toFixed(2)),
      total: Number(cost.toFixed(2)),
      trajectoryTime: Number(time.toFixed(3)),
      exitSpeed: Number(exitSpeed.toFixed(2)),
    };
    return cost;
  }
}
