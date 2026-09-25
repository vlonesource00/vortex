export class ScenarioGame {
  constructor(opportunity, ownership) { this.opportunity = opportunity; this.ownership = ownership; }
  evaluate(candidates, ego, opponents, occupancy, graph) {
    let best = null;
    const owners = this.ownership.update(ego, opponents);
    for (const candidate of candidates) {
      this.ownership.constrain(candidate, opponents);
      let score = this.opportunity.score(candidate, ego, opponents, occupancy, owners);
      if (candidate.committed) score -= 2.2;
      if (candidate.targetId !== null) {
        const edge = graph.find(item => item.a === ego.id && item.b === candidate.targetId || item.b === ego.id && item.a === candidate.targetId);
        if (edge?.overlap && candidate.flank !== this.ownership.owners.get(candidate.targetId)?.flank) score += 5;
        if (edge && candidate.targetLateral != null) score -= Math.max(0, 1.2 - edge.ttc * .08);
      }
      candidate.score = score;
      if (!best || score < best.score) best = candidate;
    }
    return best;
  }
}
