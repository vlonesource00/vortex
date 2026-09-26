import { clamp, wrap } from '../../../sim/math.js';
import { ATTACK_CORRIDOR_MARGIN } from '../interaction/clearance.js';
import { GripRobustness } from './grip-robustness.js';

const smooth = t => { t = clamp(t, 0, 1); return t * t * (3 - 2 * t); };

export class CorridorGenerator {
  constructor(atlas, maxDistance = 220, step = 10) {
    this.atlas = atlas;
    this.track = atlas.track;
    this.maxDistance = maxDistance;
    this.step = step;
    // Free-air line geometry, selected on live grip. Combat corridors are
    // built from the nominal line and are unaffected by this.
    this.robustness = new GripRobustness(atlas, atlas.track);
  }

  generate(ego, opponents, contract = null, corridorOwnership = null) {
    const atlas = this.atlas;
    const trackHalfW = this.track.halfWidth;
    const legalW = trackHalfW - (ego.spec?.halfWidth ?? 0.99) - 0.20;

    const ownedCorridor = corridorOwnership?.getOwnedCorridor?.() ?? { active: false };

    const profiles = [{ id: 'Q0', targetShift: 0, focus: null, flank: 0 }];
    if (opponents.length) {
      profiles.push(...[-3.2, -1.8, -.8, .8, 1.8, 3.2]
        .map((targetShift, i) => ({
          id: `Q${i + 1}`,
          targetShift,
          focus: null,
          flank: Math.sign(targetShift)
        })));
    }

    // Add dedicated owned-corridor candidate when side-by-side or approaching in traffic
    if (ownedCorridor.active) {
      const qMin = ownedCorridor.qMin;
      const qMax = ownedCorridor.qMax;
      const flank = ownedCorridor.flank;
      const safeBuffer = Math.min(0.55, Math.max(0.1, (qMax - qMin) * 0.25));
      const optQ = clamp(atlas.lineOffset(ego.s), qMin + safeBuffer, qMax - safeBuffer);
      profiles.unshift({
        id: `OWNED_OPT`,
        targetLateral: optQ,
        targetShift: optQ - atlas.lineOffset(ego.s),
        focus: ownedCorridor.primaryRival,
        focusStation: 20,
        flank,
        committed: true,
        isOwnedCorridor: true
      });
    }

    for (const rival of opponents) {
      const ds = wrap(rival.s - ego.s + this.track.length / 2, this.track.length) - this.track.length / 2;
      if (ds < -3.0 || ds > 62) continue;

      for (const flank of [-1, 1]) {
        // Multi-car squeeze check (Section 20):
        // If an abreast rival sits in this flank direction, check if the gap between them is physically passable.
        let blocked = false;
        for (const other of opponents) {
          if (other.id === rival.id) continue;
          const otherDs = Math.abs(wrap(other.s - rival.s + this.track.length / 2, this.track.length) - this.track.length / 2);
          if (otherDs < 14.0) {
            const egoW = ego.spec?.halfWidth ?? 0.99;
            const rW = rival.spec?.halfWidth ?? 0.99;
            const oW = other.spec?.halfWidth ?? 0.99;
            if (flank < 0 && other.lateral < rival.lateral) {
              const freeGap = (rival.lateral - rW) - (other.lateral + oW);
              if (freeGap < 2 * egoW + 0.7) {
                blocked = true;
                break;
              }
            } else if (flank > 0 && other.lateral > rival.lateral) {
              const freeGap = (other.lateral - oW) - (rival.lateral + rW);
              if (freeGap < 2 * egoW + 0.7) {
                blocked = true;
                break;
              }
            }
          }
        }
        if (blocked) continue;

        const station = wrap(rival.s + Math.max(-2, Math.min(9, (rival.longitudinalSpeed ?? rival.speed ?? 0) * .16)), this.track.length);
        const base = atlas.lineOffset(station);
        const lateral = clamp(
          rival.lateral + flank * (ego.spec.halfWidth + rival.spec.halfWidth + ATTACK_CORRIDOR_MARGIN),
          -legalW,
          legalW
        );
        const targetShift = clamp(lateral - base, -3.8, 3.8);
        profiles.push({
          id: `E${rival.id}${flank < 0 ? 'L' : 'R'}`,
          targetShift,
          focus: rival.id,
          focusStation: Math.max(12, ds),
          targetLateral: lateral,
          flank
        });
      }
    }

    if (contract) {
      const rival = opponents.find(item => item.id === contract.opponentId);
      if (rival) {
        const dsRival = wrap(rival.s - ego.s + this.track.length / 2, this.track.length) - this.track.length / 2;
        if (dsRival >= -4.5) {
          const base = atlas.lineOffset(wrap(ego.s + 20, this.track.length));
          const lockLateral = clamp(
            contract.targetLateral ?? (rival.lateral + contract.flank * (ego.spec.halfWidth + rival.spec.halfWidth + ATTACK_CORRIDOR_MARGIN)),
            -legalW,
            legalW
          );
          const lockShift = clamp(lockLateral - base, -3.8, 3.8);
          profiles.unshift({
            id: `LOCK${rival.id}`,
            targetShift: lockShift,
            focus: rival.id,
            focusStation: 22,
            targetLateral: lockLateral,
            flank: contract.flank,
            committed: true
          });
        }
      }
    }

    const candidates = [];
    for (const profile of profiles) {
      const points = [];
      const ramp = profile.focusStation ? Math.max(14, Math.min(22, profile.focusStation)) : 18;

      for (let distance = 0; distance <= this.maxDistance; distance += this.step) {
        const s = wrap(ego.s + distance, this.track.length);
        const baseLineQ = atlas.lineOffset(s);

        const isQ0 = profile.id === 'Q0' && !profile.isOwnedCorridor;

        if (isQ0) {
          // The free-air line is the oracle geometry, opened out through
          // whatever the tightest upcoming curvature event is when the live
          // tyre cannot hold the nominal margin. Pure geometry: the speed
          // profile is untouched, so any improvement is the line's own.
          const p = atlas.sample(s, this.robustness.shift(s));
          points.push({
            ...p,
            distance,
            shift: 0,
            offset: p.offset,
            speed: p.speed,
            speedLimit: p.speed,
            lateralLimit: this.track.halfWidth - ego.spec.halfWidth - .16,
            demand: 0
          });
          continue;
        }

        let targetQ;
        if (profile.isOwnedCorridor && ownedCorridor.active) {
          const safeBuffer = Math.min(0.55, Math.max(0.1, (ownedCorridor.qMax - ownedCorridor.qMin) * 0.25));
          targetQ = clamp(baseLineQ, ownedCorridor.qMin + safeBuffer, ownedCorridor.qMax - safeBuffer);
        } else if (Number.isFinite(profile.targetLateral)) {
          targetQ = profile.targetLateral;
        } else {
          targetQ = baseLineQ + profile.targetShift;
        }
        targetQ = clamp(targetQ, -legalW, legalW);

        // Continuous origin transition: at distance 0, start smoothly at ego.lateral
        const transition = smooth(distance / ramp);
        let q = ego.lateral + (targetQ - ego.lateral) * transition;

        if (!profile.isOwnedCorridor && profile.focusStation && distance > profile.focusStation + 20) {
          const exitBlend = smooth((distance - profile.focusStation - 20) / 36);
          // Gently blend toward the optimal track line post-pass if clear, bounded by owned corridor
          const targetExitQ = ownedCorridor.active
            ? clamp(baseLineQ, ownedCorridor.qMin + 0.25, ownedCorridor.qMax - 0.25)
            : clamp(baseLineQ, -legalW, legalW);
          q = q * (1 - exitBlend) + targetExitQ * exitBlend;
        }

        q = clamp(q, -legalW, legalW);
        const shift = q - baseLineQ;
        const p = atlas.sample(s, shift);

        points.push({
          ...p,
          distance,
          shift,
          offset: q,
          speed: p.speed,
          speedLimit: p.speed,
          lateralLimit: this.track.halfWidth - ego.spec.halfWidth - .16,
          demand: 0
        });
      }

      candidates.push({
        id: profile.id,
        points,
        targetId: profile.focus,
        targetLateral: profile.targetLateral ?? points.at(-1)?.offset,
        flank: profile.flank ?? 0,
        committed: Boolean(profile.committed),
        targetShift: profile.targetShift,
        isOwnedCorridor: Boolean(profile.isOwnedCorridor)
      });
    }

    return candidates;
  }
}
