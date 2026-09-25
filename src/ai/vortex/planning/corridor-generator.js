import { clamp, wrap } from '../../../sim/math.js';
import { ATTACK_CORRIDOR_MARGIN } from '../interaction/clearance.js';

const smooth = t => { t = clamp(t, 0, 1); return t * t * (3 - 2 * t); };

export class CorridorGenerator {
  constructor(atlas, maxDistance = 220, step = 10) { this.atlas = atlas; this.track = atlas.track; this.maxDistance = maxDistance; this.step = step; }
  generate(ego, opponents, contract = null) {
    // Corridor shifts are measured against the atlas's own free-air line and
    // ramp from zero. Starting each candidate at the car's current deviation
    // instead would bake that deviation into the plan, making the servo's
    // tracking error identically zero and leaving the car permanently off line.
    const atlas = this.atlas;
    const profiles = [{ id: 'Q0', targetShift: 0, focus: null }];
    if (opponents.length) profiles.push(...[-3.2, -1.8, -.8, .8, 1.8, 3.2]
      .map((targetShift, i) => ({ id: `Q${i + 1}`, targetShift, focus: null })));
    for (const rival of opponents) {
      const ds = wrap(rival.s - ego.s + this.track.length / 2, this.track.length) - this.track.length / 2;
      if (ds < -28 || ds > 62) continue;
      for (const flank of [-1, 1]) {
        const station = wrap(rival.s + Math.max(-2, Math.min(9, rival.longitudinalSpeed * .16)), this.track.length);
        const base = atlas.lineOffset(station);
        // The flank must sit OUTSIDE the interaction boundary, or the attack
        // corridor is traffic-capped by construction. Previously this was +0.42
        // against an interaction threshold of +0.44: the dedicated passing route
        // was 2 cm inside the traffic-conflict boundary. See interaction/clearance.js.
        const lateral = rival.lateral + flank * (ego.spec.halfWidth + rival.spec.halfWidth + ATTACK_CORRIDOR_MARGIN);
        const targetShift = clamp(lateral - base, -3.8, 3.8);
        profiles.push({ id: `E${rival.id}${flank < 0 ? 'L' : 'R'}`, targetShift, focus: rival.id,
          focusStation: Math.max(12, ds), targetLateral: lateral, flank });
      }
    }
    if (contract) {
      const rival = opponents.find(item => item.id === contract.opponentId);
      if (rival) profiles.unshift({ id: `LOCK${rival.id}`,
        targetShift: clamp(contract.flank * 2.35, -3.7, 3.7),
        focus: rival.id, focusStation: 22, targetLateral: rival.lateral + contract.flank * 2.5,
        flank: contract.flank, committed: true });
    }
    const candidates = [];
    for (const profile of profiles) {
      const points = [];
      const ramp = profile.focusStation ? 18 : 16;
      for (let distance = 0; distance <= this.maxDistance; distance += this.step) {
        const transition = smooth(distance / ramp);
        let shift = profile.targetShift * transition;
        if (profile.focusStation && distance > profile.focusStation + 20) {
          const exit = smooth((distance - profile.focusStation - 20) / 32);
          shift = shift * (1 - exit) + profile.targetShift * exit;
        }
        const s = wrap(ego.s + distance, this.track.length), p = this.atlas.sample(s, shift);
        points.push({ ...p, distance, shift, speed: p.speed, speedLimit: p.speed,
          lateralLimit: this.track.halfWidth - ego.spec.halfWidth - .16, demand: 0 });
      }
      candidates.push({ id: profile.id, points, targetId: profile.focus, targetLateral: profile.targetLateral,
        flank: profile.flank ?? 0, committed: Boolean(profile.committed), targetShift: profile.targetShift });
    }
    return candidates;
  }
}
