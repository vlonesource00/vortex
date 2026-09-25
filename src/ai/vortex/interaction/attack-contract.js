export class AttackContract {
  constructor() { this.active = null; }
  update(best, ego, graph) {
    const target = best?.targetId ?? null;
    if (!target) { if (!this.active || this.active.clear++ > 4) this.active = null; return this.active; }
    const currentEdge = graph.find(edge => edge.a === ego.id && edge.b === target || edge.b === ego.id && edge.a === target);
    if (!this.active || this.active.opponentId !== target) this.active = {
      opponentId: target, flank: Math.sign((best.targetLateral ?? 0) - ego.lateral || 1),
      launchS: ego.s, completionS: best.points.at(-1)?.s ?? ego.s, returnS: (best.points.at(-1)?.s ?? ego.s) + 12,
      probability: .5, exitValue: best.exitSpeed, clear: 0,
    };
    else {
      const advantage = (this.active.flank === Math.sign((best.targetLateral ?? 0) - ego.lateral || 1)) ? 0 : best.score;
      if (advantage > 1.8 || currentEdge?.overlap === false) this.active.flank = Math.sign((best.targetLateral ?? 0) - ego.lateral || 1);
      this.active.exitValue = best.exitSpeed; this.active.clear = 0;
    }
    this.active.probability = Math.max(0, Math.min(1, .55 + (best.exitSpeed - ego.speed) * .025));
    return this.active;
  }
}
