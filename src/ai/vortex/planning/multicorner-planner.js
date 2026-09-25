import { AttackContract, FLANK_SWITCH_HYSTERESIS } from '../interaction/attack-contract.js';
import { BrakeEvents } from './brake-events.js';
import { CorridorGenerator } from './corridor-generator.js';
import { OpportunityField } from '../interaction/opportunity-field.js';
import { ScenarioGame } from './scenario-game.js';
import { TrajectoryRefiner } from './trajectory-refiner.js';

export class MulticornerPlanner {
  constructor(atlas, track, envelope, ownership) {
    this.atlas = atlas; this.track = track; this.envelope = envelope; this.ownership = ownership;
    this.generator = new CorridorGenerator(atlas); this.opportunity = new OpportunityField(track, atlas);
    this.game = new ScenarioGame(this.opportunity, ownership); this.refiner = new TrajectoryRefiner(track, envelope, atlas);
    this.attack = new AttackContract(); this.brakeEvents = new BrakeEvents();
    this.plan = null; this.candidates = []; this.lastSolveMs = 0; this.solveCount = 0;
  }
  update(observation, opponents, occupancy, graph) {
    const start = globalThis.performance?.now?.() ?? Date.now();
    const ego = observation.ego;
    const relevant = opponents.filter(rival => graph.some(edge => (edge.a === ego.id && edge.b === rival.id
      || edge.b === ego.id && edge.a === rival.id) && (edge.closing > .55 || edge.overlap)));
    const candidates = this.generator.generate(ego, relevant, this.attack.active);
    for (const candidate of candidates) this.refiner.refine(candidate, ego, occupancy, opponents);
    const best = this.game.evaluate(candidates, ego, opponents, occupancy, graph) ?? candidates[0];
    const old = this.plan, prior = old && candidates.find(item => item.id === old.id);
    const commitment = this.attack.active ? 1.8 : 2.6;
    if (prior && best.id !== prior.id && best.score > prior.score - commitment) this.plan = prior;
    else if (old && this.attack.active && best.targetId !== this.attack.active.opponentId) {
      const retained = candidates.find(item => item.targetId === this.attack.active.opponentId && item.score <= best.score + 1.8);
      this.plan = retained ?? best;
    } else this.plan = best;

    // §14 Contract/plan closure. AttackContract is updated AFTER selection, so
    // without this the contract could report COMMITTED LEFT while the solve
    // executed a right-hand or generic geometry. Once committed, the planner
    // may choose any geometry on the committed flank, but may not execute the
    // opposite flank unless the alternative beats it by the ΔJ hysteresis or
    // the committed corridor has become invalid.
    if (this.attack.active?.committed) {
      const committedFlank = this.attack.active.flank;
      const target = this.attack.active.opponentId;
      const flankOf = (c) => c?.flank || Math.sign((c?.targetLateral ?? 0) - ego.lateral || 1);
      const family = candidates.filter(c => c.targetId === target);
      const byFlank = (f) => family.filter(c => flankOf(c) === f).sort((a, b) => a.score - b.score)[0];
      const mine = byFlank(committedFlank);
      const theirs = byFlank(-committedFlank);
      const dJ = (mine && theirs) ? (theirs.score - mine.score) : 0;
      const maySwitch = theirs && mine && dJ < -FLANK_SWITCH_HYSTERESIS;
      // Only a RIVAL ATTACK candidate can violate the commitment. A generic
      // corridor has targetLateral === null and its flank sign is meaningless
      // (it is derived from (0 - ego.lateral)), so forcing it onto the attack
      // flank would be the same contamination this block removed from the
      // contract. Mismatch is defined as an attack trajectory crossing to the
      // opposite rival flank.
      const planIsAttack = this.plan?.targetId === target;
      if (!maySwitch && mine && planIsAttack && flankOf(this.plan) === -committedFlank) this.plan = mine;
    }

    this.attack.update(this.plan, ego, graph, candidates);
    this.brakeEvents.update(this.plan, ego.s, this.track.length, observation, occupancy);
    this.candidates = candidates; this.solveCount++;
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
