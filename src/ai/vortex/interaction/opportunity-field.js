import { clamp, wrap } from '../../../sim/math.js';
import {
  PROXIMITY_LAT_DECAY, PROXIMITY_LONG_DECAY, candidateLateralVelocity,
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
        // Traffic restriction is a difference between PLAN LIMITS, never
        // between a limit and the forward-reachable speed. `p.speed` lags
        // `speedLimit` whenever the car is still accelerating, so the old form
        // counted ordinary acceleration lag as traffic limitation. That fed
        // AttackContract.isCredible() and silently blocked commitment.
        const freeLimit = p.freeSpeedLimit ?? p.speedLimit ?? p.speed;
        const planLimit = p.speedLimit ?? p.speed;
        if (freeLimit > planLimit + 0.25) trafficLimited++;
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
        //
        // Candidate lateral velocity in PHYSICAL units. Candidate points are
        // spatially separated by the corridor step (10 m), not by one planner
        // tick, so the old `(p.offset - prior.offset) * 27` invented a lateral
        // speed up to 54 m/s for a routine 2 m transition over 10 m. Use
        // v_q = (dq/ds) * v instead.
        //
        // `crossing` must be driven by the LATERAL closing rate alone. Dividing
        // by total relative speed diluted a hard cut-in to near zero whenever
        // the longitudinal closing rate was large -- which is precisely when
        // swinging across a car you are lapping is most dangerous.
        const relLong = Math.abs(ego.speed - predicted.speed);
        const dsStep = prior ? Math.max(0.1, p.distance - prior.distance) : 0.1;
        const vLatEgo = prior ? candidateLateralVelocity(p.offset - prior.offset, dsStep, ego.speed) : 0;
        const vLatRival = (future.lateral - predicted.lateral) * 27;
        const relLat = Math.abs(vLatRival - vLatEgo);
        const relSpeed = Math.hypot(relLong, relLat);
        const crossing = clamp(relLat / 8, 0, 1);

        // Track-edge pinch. Pinching a rival against the barrier is worse than
        // the same clearance in open space.
        const edgeRoom = this.track.halfWidth - Math.abs(p.offset);
        const edgePinch = edgeRoom < ego.spec.halfWidth + 0.55
          ? 1 + (ego.spec.halfWidth + 0.55 - edgeRoom) * 1.6 : 1;

        // §16 physics & Section 5 swept-body conflict:
        // High-energy collision and swept-body overlap are disqualified; parallel close racing inside
        // the legal corridor is accepted.
        let sweptConflict = false;
        if (prior && time < 2.5) {
          const minLat = Math.min(prior.offset, p.offset) - ego.spec.halfWidth;
          const maxLat = Math.max(prior.offset, p.offset) + ego.spec.halfWidth;
          const rivalMinLat = predicted.lateral - predicted.halfWidth;
          const rivalMaxLat = predicted.lateral + predicted.halfWidth;
          const latIntersect = Math.max(0, Math.min(maxLat, rivalMaxLat) - Math.max(minLat, rivalMinLat));

          const sA = Math.min(prior.s, p.s) - ego.spec.halfLength;
          const sB = Math.max(prior.s, p.s) + ego.spec.halfLength;
          const rA = predicted.s - predicted.halfLength;
          const rB = predicted.s + predicted.halfLength;
          const longIntersect = Math.max(0, Math.min(sB, rB) - Math.max(sA, rA));

          if (latIntersect > 0 && longIntersect > 0 && Math.abs(p.offset - prior.offset) > 1.2) {
            sweptConflict = true;
          }
        }

        const hasCollisionOverlap = (overlap > 0 || sweptConflict) && time < 2.5;
        if (hasCollisionOverlap) path.hasCollision = true;
        const overlapCost = hasCollisionOverlap ? 1200 + (overlap + (sweptConflict ? 0.8 : 0)) * 500 : (overlap > 0 ? 190 + overlap * 260 : 0);
        const crossingCost = crossing ** 2 * Math.min(250, relSpeed ** 2 * 1.2);
        // Parallel racing: if separated by >= 0.28m, parallelCost is zero!
        const parallelCost = latGap >= 0.28 ? 0 : proximity * (2.5 + relLong * 0.35);
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
        if (owner.flank < 0 && p.offset > owner.qMax + 0.2) {
          ownershipCost += 80;
          cost += 80;
          path.violatesOwnership = true;
        } else if (owner.flank > 0 && p.offset < owner.qMin - 0.2) {
          ownershipCost += 80;
          cost += 80;
          path.violatesOwnership = true;
        }
      }
    }

    if (path.violatesOwnership) {
      cost += 800;
    }
    if (path.isOwnedCorridor && !path.hasCollision && !path.violatesOwnership) {
      cost -= 3.5;
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
