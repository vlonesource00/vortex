import { clamp, wrap } from '../../../sim/math.js';
import { INTERACTION_MARGIN, OWNERSHIP_MARGIN } from './clearance.js';

/**
 * Dynamic Corridor Ownership (Section 5 & 6).
 *
 * Implements true side-by-side owned corridor calculation:
 * - Rival acts as a dynamic moving wall boundary.
 * - During longitudinal overlap or close-quarters approach, ego owns [q_min, q_max]
 *   on its side of the rival.
 * - The free-air line Q0 is marked infeasible if it crosses through the rival's space.
 * - Candidates are evaluated for compliance with the owned corridor.
 */
export class CorridorOwnership {
  constructor(track) {
    this.track = track;
    this.owners = new Map();
    this.egoCorridor = { qMin: -6.5, qMax: 6.5, active: false, primaryRival: null, flank: 0 };
  }

  update(ego, opponents) {
    const legalMargin = (ego.spec?.halfWidth ?? 0.99) + 0.35;
    const trackHalfW = this.track.halfWidth;
    let netQMin = -trackHalfW + legalMargin;
    let netQMax = trackHalfW - legalMargin;
    let hasActiveOverlap = false;
    let primaryRival = null;
    let primaryFlank = 0;

    for (const rival of opponents) {
      const ds = wrap(rival.s - ego.s + this.track.length / 2, this.track.length) - this.track.length / 2;
      const egoL = ego.spec?.halfLength ?? 2.3;
      const rivalL = rival.spec?.halfLength ?? 2.3;
      const egoW = ego.spec?.halfWidth ?? 0.99;
      const rivalW = rival.spec?.halfWidth ?? 0.99;

      const longDist = Math.abs(ds);
      const isLongitudinalOverlap = longDist < (egoL + rivalL + 1.2);
      const relLat = rival.lateral - ego.lateral;
      const closing = (ego.speed ?? ego.longitudinalSpeed ?? 0) - (rival.longitudinalSpeed ?? rival.speed ?? 0);
      const latThreat = Math.abs(relLat) < (egoW + rivalW + 2.2);
      const isApproach = ds > (egoL + rivalL) && ds < Math.max(12.0, closing * 2.2) && closing > 0.5 && latThreat;
      const isActive = isLongitudinalOverlap || isApproach;
      const current = this.owners.get(rival.id);

      if (isActive) {
        // Determine or retain flank
        // If rival is to the right (relLat > 0), ego is on the left (flank = -1)
        const newFlank = relLat >= 0 ? -1 : 1;
        const flank = (current && (isLongitudinalOverlap || current.overlap)) ? current.flank : newFlank;

        const buffer = egoW + rivalW + (OWNERSHIP_MARGIN ?? 0.85);

        let qMin = -trackHalfW + legalMargin;
        let qMax = trackHalfW - legalMargin;

        if (flank < 0) {
          // Ego owns LEFT corridor
          qMax = Math.min(trackHalfW - legalMargin, rival.lateral - buffer);
        } else {
          // Ego owns RIGHT corridor
          qMin = Math.max(-trackHalfW + legalMargin, rival.lateral + buffer);
        }

        // Sanity guard so bounds don't invert
        if (qMin > qMax) {
          const mid = (qMin + qMax) * 0.5;
          qMin = mid - 0.5;
          qMax = mid + 0.5;
        }

        this.owners.set(rival.id, {
          id: rival.id,
          flank,
          qMin,
          qMax,
          rivalQ: rival.lateral,
          rivalS: rival.s,
          ds,
          overlap: isLongitudinalOverlap,
          clearTicks: 0,
        });

        // Tighten net corridor
        if (flank < 0) {
          netQMax = Math.min(netQMax, qMax);
        } else {
          netQMin = Math.max(netQMin, qMin);
        }

        hasActiveOverlap = hasActiveOverlap || isLongitudinalOverlap;
        if (!primaryRival || isLongitudinalOverlap) {
          primaryRival = rival.id;
          primaryFlank = flank;
        }
      } else if (current) {
        current.clearTicks++;
        if (current.clearTicks >= 4 && !isActive) {
          this.owners.delete(rival.id);
        }
      }
    }

    for (const id of this.owners.keys()) {
      if (!opponents.some(rival => rival.id === id)) this.owners.delete(id);
    }

    this.egoCorridor = {
      qMin: Math.min(netQMin, netQMax - 0.5),
      qMax: Math.max(netQMax, netQMin + 0.5),
      active: this.owners.size > 0,
      hasOverlap: hasActiveOverlap,
      primaryRival,
      flank: primaryFlank,
    };

    return [...this.owners.values()];
  }

  getOwnedCorridor() {
    return this.egoCorridor;
  }

  constrain(candidate, opponents) {
    if (!this.egoCorridor.active) {
      candidate.violatesOwnership = false;
      candidate.corridorFeasible = true;
      return candidate;
    }

    const { qMin, qMax, flank } = this.egoCorridor;
    let outsidePoints = 0;
    const n = candidate.points.length;
    const checkLength = Math.min(n, 8); // check first ~80m

    for (let i = 0; i < checkLength; i++) {
      const offset = candidate.points[i].offset;
      if (flank < 0 && offset > qMax + 0.25) {
        outsidePoints++;
      } else if (flank > 0 && offset < qMin - 0.25) {
        outsidePoints++;
      }
    }

    candidate.violatesOwnership = outsidePoints > 1;
    candidate.corridorFeasible = !candidate.violatesOwnership;
    candidate.ownership = !candidate.violatesOwnership;
    return candidate;
  }
}
