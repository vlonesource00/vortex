/**
 * Attack commitment semantics around the continuous planner.
 *
 * This is NOT a personality state machine. It is a thin layer that decides
 * whether the current attack flank is credible enough to be held against
 * frame-to-frame scoring noise. The planner keeps producing candidates every
 * solve; this class decides which flank is allowed to win.
 *
 * The previous implementation committed a flank flip whenever
 *
 *     (best.score > 1.8 || edge.overlap === false)
 *
 * where `best.score` is the ABSOLUTE cost of the newly winning candidate.
 * Costs run in the hundreds, so that condition was true on essentially every
 * solve where a different flank won, no matter how small the actual advantage.
 * That is the reported "left/right attack changing inside fractions of a second
 * on the same corner". See tools/audit-clearance.mjs and the block report.
 *
 * The correct quantity is the DIFFERENCE between the two flanks:
 *
 *     dJ = J(alternative) - J(committed)
 *
 * in the same score units the planner already uses (1 unit = 1/13 s of race
 * time, because OpportunityField's trajectory term is `time * 13`).
 */
import { restrictsSpeed, INTERACTION_MARGIN } from './clearance.js';
import { wrap } from '../../../sim/math.js';

/** Solved needed on one flank before commitment is granted (~185 ms at 27 Hz). */
export const COMMITMENT_PERSISTENCE = 5;

/**
 * Score advantage an alternative flank must show to break an existing
 * commitment. Lower scores are better, so the alternative must be this much
 * CHEAPER. 3.0 units is about 0.23 s of race time -- enough to be real, small
 * enough that a genuinely superior route is not held off.
 */
export const FLANK_SWITCH_HYSTERESIS = 3.0;

/**
 * A candidate is only commit-credible if it is not being traffic-capped into a
 * crawl and retains some exit value. Without this, commitment freezes a bad
 * corridor -- the exact failure the earlier hysteresis experiment produced.
 */
export function isCredible(candidate, ego) {
  if (!candidate) return false;
  const exit = candidate.exitSpeed ?? 0;
  const capped = (candidate.trafficLimitedFraction ?? 0) > 0.85;
  return !capped && exit >= ego.speed * 0.45;
}

export class AttackContract {
  constructor() {
    this.active = null;
    this.state = 'SEARCH';
    this.streak = 0;
    this.streakFlank = 0;
    this.switchLog = [];
  }

  /**
   * @param {object} best       currently selected plan
   * @param {object} ego
   * @param {Array}  graph      engagement edges
   * @param {Array}  candidates all scored candidates this solve
   * @param {Array}  opponents  visible opponents
   * @param {object} track      track model
   */
  update(best, ego, graph = [], candidates = [], opponents = [], track = null) {
    const target = best?.targetId ?? null;
    // The generator already assigns an explicit flank to every rival-attack
    // candidate. Deriving it from sign(targetLateral - ego.lateral) is wrong:
    // the sign flips as the ego moves along the corridor, so a committed LEFT
    // attack can report RIGHT while still driving the left line. That is
    // exactly the contract/plan mismatch this block must close.
    const flankOf = (c) => c?.flank || Math.sign((c?.targetLateral ?? 0) - ego.lateral || 1);

    let targetDs = Infinity;
    if (target !== null && track && opponents?.length) {
      const rival = opponents.find(item => item.id === target);
      if (rival) {
        targetDs = wrap(rival.s - ego.s + track.length / 2, track.length) - track.length / 2;
      }
    }

    // Section 11 & Grid-start:
    // If target is physically cleared (ds < -5.1m) or nonexistent, handle completion
    if (!target || targetDs < -5.1) {
      if (this.active && ++this.active.clear > 4) { this.active = null; this.state = 'SEARCH'; }
      else if (this.active) this.state = 'CLEARING';
      this.streak = 0;
      return this.active;
    }

    // Never initiate a new attack against a car that is already behind ego
    if ((!this.active || this.active.opponentId !== target) && targetDs < -2.0) {
      this.streak = 0;
      return this.active;
    }

    const currentEdge = graph.find(edge =>
      (edge.a === ego.id && edge.b === target) || (edge.b === ego.id && edge.a === target));
    const overlapping = currentEdge?.overlap === true;

    // Per-flank best cost this solve. Only candidates that actually describe
    // THIS rival's attack geometry may participate. Generic corridors (Q0, Qn)
    // have targetLateral === null, and `targetLateral ?? 0` used to classify
    // them as a left/right flank purely from the sign of (0 - ego.lateral).
    // That let a generic corridor silently decide a rival attack comparison.
    // LOCK and E candidates both carry targetId === the rival id, so this
    // filter includes exactly the attack family.
    const bestByFlank = new Map();
    for (const c of candidates) {
      if (c.targetId !== target) continue;
      const f = flankOf(c);
      const prev = bestByFlank.get(f);
      if (!prev || c.score < prev.score) bestByFlank.set(f, c);
    }

    const bestFlank = flankOf(best);

    if (!this.active || this.active.opponentId !== target) {
      // First credible sight of a target. Do not commit yet.
      this.active = {
        opponentId: target,
        flank: bestFlank,
        launchS: ego.s,
        completionS: best.points.at(-1)?.s ?? ego.s,
        returnS: (best.points.at(-1)?.s ?? ego.s) + 12,
        probability: 0.5,
        exitValue: best.exitSpeed,
        clear: 0,
        committed: false,
        // §17 The committed attack's PHYSICAL corridor. LOCK must inherit
        // these, not jump to a hardcoded atlas shift.
        targetShift: best.targetShift ?? 0,
        targetLateral: best.targetLateral ?? ego.lateral,
      };
      this.streak = 1;
      this.streakFlank = bestFlank;
      this.state = 'SETUP';
    } else {
      const committedFlank = this.active.flank;
      const sameFlank = bestFlank === committedFlank;

      // §23 Persistence as evidence, not force. Count consecutive solves the
      // same flank wins before granting commitment.
      if (sameFlank) {
        this.streak = this.streakFlank === bestFlank ? this.streak + 1 : 1;
        this.streakFlank = bestFlank;
      } else {
        this.streak = 1;
        this.streakFlank = bestFlank;
      }

      if (!this.active.committed) {
        // SEARCH -> SETUP -> COMMITTED. Only a persistence-confirmed, credible
        // flank may be committed.
        this.state = 'SETUP';
        if (this.streak >= COMMITMENT_PERSISTENCE && isCredible(best, ego)) {
          this.active.committed = true;
          this.active.flank = bestFlank;
          this.state = overlapping ? 'OVERLAP' : 'COMMITTED';
        }
      } else {
        // §20 Hysteresis on the DIFFERENCE between flanks, never on an
        // absolute score.
        const alt = bestByFlank.get(-committedFlank);
        const mine = bestByFlank.get(committedFlank);
        const dJ = (alt && mine) ? (alt.score - mine.score) : (sameFlank ? 0 : best.score);
        const corridorInvalid = !isCredible(mine ?? best, ego);

        if (!sameFlank && (dJ < -FLANK_SWITCH_HYSTERESIS || corridorInvalid)) {
          this.switchLog.push({
            t: globalThis.performance?.now?.() ?? Date.now(),
            from: committedFlank,
            to: bestFlank,
            dJ: Number(dJ.toFixed(2)),
            reason: corridorInvalid ? 'corridor_invalid' : 'alternative_dominates',
          });
          this.active.flank = bestFlank;
          this.active.committed = isCredible(best, ego);
          this.streak = 1;
          this.streakFlank = bestFlank;
        }
        this.state = overlapping ? 'OVERLAP' : 'COMMITTED';
      }

      this.active.exitValue = best.exitSpeed;
      this.active.clear = 0;
    }

    // §17 Keep the committed physical corridor in sync with the winning
    // candidate on the committed flank, so LOCK inherits the actual attack
    // geometry rather than a hardcoded atlas shift.
    const ref = bestByFlank.get(this.active.flank) ?? best;
    this.active.targetShift = ref.targetShift ?? this.active.targetShift ?? 0;
    this.active.targetLateral = ref.targetLateral ?? this.active.targetLateral ?? ego.lateral;

    this.active.probability = Math.max(0, Math.min(1, 0.55 + (best.exitSpeed - ego.speed) * 0.025));
    return this.active;
  }
}
