import { AttackContract, FLANK_SWITCH_HYSTERESIS } from '../interaction/attack-contract.js';
import { BrakeEvents } from './brake-events.js';
import { CorridorGenerator } from './corridor-generator.js';
import { OpportunityField } from '../interaction/opportunity-field.js';
import { ScenarioGame } from './scenario-game.js';
import { TrajectoryRefiner } from './trajectory-refiner.js';
import { wrap } from '../../../sim/math.js';

export class MulticornerPlanner {
  constructor(atlas, track, envelope, ownership) {
    this.atlas = atlas;
    this.track = track;
    this.envelope = envelope;
    this.ownership = ownership;
    this.generator = new CorridorGenerator(atlas);
    this.opportunity = new OpportunityField(track, atlas);
    this.game = new ScenarioGame(this.opportunity, ownership);
    this.refiner = new TrajectoryRefiner(track, envelope, atlas);
    this.attack = new AttackContract();
    this.brakeEvents = new BrakeEvents();
    this.plan = null;
    this.candidates = [];
    this.lastSolveMs = 0;
    this.solveCount = 0;
  }

  update(observation, opponents, occupancy, graph) {
    const start = globalThis.performance?.now?.() ?? Date.now();
    const ego = observation.ego;

    // Multi-car & Grid-start relevance (Section 19 & 20):
    // Any rival within 45m along track matters for spatial occupancy and corridor boundaries.
    const relevant = opponents.filter(rival => {
      const ds = Math.abs(wrap(rival.s - ego.s + this.track.length / 2, this.track.length) - this.track.length / 2);
      return ds < 45.0;
    });

    // Update ownership first so generator has current owned corridor
    this.ownership.update(ego, relevant);

    // Free-air line geometry: latch onto the most demanding upcoming event and
    // choose which of the discrete geometries the live tyre can afford. This
    // runs before candidate generation so the free-air candidate is already the
    // right shape; combat corridors are built from the nominal line and are
    // not touched.
    this.generator.robustness.survey(ego.s);
    this.generator.robustness.select(this.envelope?.planGrip ?? 1);

    const candidates = this.generator.generate(ego, relevant, this.attack.active, this.ownership);
    for (const candidate of candidates) {
      this.refiner.refine(candidate, ego, occupancy, opponents);
    }

    const best = this.game.evaluate(candidates, ego, opponents, occupancy, graph) ?? candidates[0];
    const old = this.plan;
    const prior = old && candidates.find(item => item.id === old.id && !item.hasCollision && !item.violatesOwnership);
    const commitment = this.attack.active ? 1.8 : 2.6;

    if (prior && best.id !== prior.id && best.score > prior.score - commitment) {
      this.plan = prior;
    } else if (old && this.attack.active && best.targetId !== this.attack.active.opponentId) {
      const retained = candidates.find(item => item.targetId === this.attack.active.opponentId && !item.hasCollision && !item.violatesOwnership && item.score <= best.score + 1.8);
      this.plan = retained ?? best;
    } else {
      this.plan = best;
    }

    // Section 12 Enforcement: Generic candidates MUST NOT escape a committed attack or violate ownership.
    const ownedCorridor = this.ownership.getOwnedCorridor();
    const isCommitted = Boolean(this.attack.active?.committed);
    const isOverlapActive = Boolean(ownedCorridor?.active && ownedCorridor.hasOverlap);

    if (isCommitted || isOverlapActive) {
      const target = this.attack.active?.opponentId ?? ownedCorridor.primaryRival;
      const committedFlank = this.attack.active?.flank || ownedCorridor.flank;
      const flankOf = (c) => c?.flank || Math.sign((c?.targetLateral ?? c?.points?.at(-1)?.offset ?? 0) - ego.lateral || 1);

      // Find best candidate on our committed / owned flank that is collision-free and corridor-feasible
      const validCandidates = candidates.filter(c => !c.hasCollision && !c.violatesOwnership);
      const mine = validCandidates.find(c => (c.isOwnedCorridor || c.targetId === target) && flankOf(c) === committedFlank)
        ?? validCandidates.find(c => flankOf(c) === committedFlank)
        ?? validCandidates[0];

      const alt = validCandidates.find(c => c.targetId === target && flankOf(c) === -committedFlank);
      const dJ = (alt && mine) ? (alt.score - mine.score) : 0;
      const maySwitch = alt && mine && dJ < -FLANK_SWITCH_HYSTERESIS;

      // If current plan has collision, violates ownership, or tries to cross opposite to committed flank
      const planViolates = this.plan.hasCollision
        || this.plan.violatesOwnership
        || (!maySwitch && flankOf(this.plan) === -committedFlank && committedFlank !== 0);

      if (planViolates && mine) {
        this.plan = mine;
      }
    } else if (this.plan?.hasCollision || this.plan?.violatesOwnership) {
      // Even in free search, never execute a candidate that has a direct predicted collision
      const safeCandidate = candidates.find(c => !c.hasCollision && !c.violatesOwnership);
      if (safeCandidate) this.plan = safeCandidate;
    }

    this.attack.update(this.plan, ego, graph, candidates, opponents, this.track);
    this.brakeEvents.update(this.plan, ego.s, this.track.length, observation, occupancy);
    this.candidates = candidates;
    this.solveCount++;
    this.lastSolveMs = Math.max(0, (globalThis.performance?.now?.() ?? Date.now()) - start);
    return this.plan;
  }

  at(s) {
    if (!this.plan?.points?.length) return this.atlas.sample(s);
    const points = this.plan.points, track = this.track;
    let distance = ((s - points[0].s + track.length) % track.length);
    if (distance > points.at(-1).distance) return points.at(-1);
    const i = Math.max(0, Math.min(points.length - 2, Math.floor(distance / this.generator.step)));
    const a = points[i], b = points[i + 1], t = Math.max(0, Math.min(1, (distance - a.distance) / Math.max(.01, b.distance - a.distance)));
    const mix = (x, y) => x + (y - x) * t;
    return {
      ...a,
      x: mix(a.x, b.x), z: mix(a.z, b.z),
      offset: mix(a.offset, b.offset),
      speed: mix(a.speed, b.speed),
      speedLimit: mix(a.speedLimit ?? a.speed, b.speedLimit ?? b.speed),
      lateralLimit: mix(a.lateralLimit ?? 20, b.lateralLimit ?? 20),
      curvature: mix(a.curvature, b.curvature),
    };
  }
}
