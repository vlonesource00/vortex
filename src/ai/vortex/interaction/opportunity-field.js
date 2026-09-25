import { clamp, wrap } from '../../../sim/math.js';
import {
  PROXIMITY_LAT_DECAY, PROXIMITY_LONG_DECAY,
} from './clearance.js';

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
        const future = occupancy.at(rival, time + 1 / 27);
        let ds = wrap(predicted.s - p.s + this.track.length / 2, this.track.length) - this.track.length / 2;

        // SIGNED body clearances, in metres of free gap between the two bodies.
        // >0 separated, <0 overlapping. The previous form subtracted a margin
        // and clamped at zero, which saturated the risk Gaussian below 0.34 m
        // and charged a legal flank 99% of what a real impact costs.
        const bodyLat = ego.spec.halfWidth + predicted.halfWidth;
        const bodyLong = ego.spec.halfLength + predicted.halfLength;
        const latGap = Math.abs(predicted.lateral - p.offset) - bodyLat;
        const alongGap = Math.abs(ds) - bodyLong;

        // Proximity in the GAP domain: a car in a separate route costs nothing.
        const proximity = Math.exp(-Math.max(0, latGap) / PROXIMITY_LAT_DECAY)
          * Math.exp(-Math.max(0, alongGap) / PROXIMITY_LONG_DECAY);

        // Overlap depth. Zero unless the body rectangles intersect on BOTH axes.
        // Depth is the shallow axis of penetration: a 0.1 m sliver along the
        // full car length is a 0.1 m penetration, not a 4.6 m one.
        const overLat = Math.max(0, -latGap);
        const overLong = Math.max(0, -alongGap);
        const overlap = overLat > 0 && overLong > 0 ? Math.min(overLat, overLong) : 0;

        // Relative energy and crossing angle. §15: a parallel car at small dv
        // and positive clearance must NOT cost the same as a trajectory crossing
        // through its door.
        const relLong = Math.abs(ego.speed - predicted.speed);
        const relLat = Math.abs((future.lateral - predicted.lateral) * 27 - (prior ? (p.offset - prior.offset) * 27 : 0));
        const relSpeed = Math.hypot(relLong, relLat);
        const crossing = relSpeed > 0.5 ? relLat / relSpeed : 0;

        // Track-edge pinch. Pinching a rival against the barrier is worse than
        // the same clearance in open space.
        const edgeRoom = this.track.halfWidth - Math.abs(p.offset);
        const edgePinch = edgeRoom < ego.spec.halfWidth + 0.55
          ? 1 + (ego.spec.halfWidth + 0.55 - edgeRoom) * 1.6 : 1;

        // §16 physics, not a global weight reduction. High-energy collision
        // remains extremely expensive; parallel close racing is acceptable.
        const overlapCost = overlap > 0 ? 190 + overlap * 260 : 0;
        const crossingCost = crossing ** 2 * Math.min(150, relSpeed ** 2 * 0.85);
        const parallelCost = proximity * (2.5 + relLong * 0.35);
        const cContact = (overlapCost * edgePinch + proximity * crossingCost * edgePinch + parallelCost);
        contact += cContact;
        cost += cContact;
        if (cContact > 40) interactions++;
        const clearance = Math.hypot(Math.max(0, alongGap), Math.max(0, latGap));
        minClearance = Math.min(minClearance, clearance);
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
