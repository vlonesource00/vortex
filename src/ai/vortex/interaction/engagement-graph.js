import { clamp, wrap } from '../../../sim/math.js';

export class EngagementGraph {
  constructor(track) { this.track = track; this.edges = []; this.meaningfulTime = 0; }
  update(ego, opponents, occupancy, dt) {
    const nodes = [ego, ...opponents], edges = [];
    for (let i = 0; i < nodes.length; i++) for (let j = i + 1; j < nodes.length; j++) {
      const a = nodes[i], b = nodes[j], ds = wrap(b.s - a.s + this.track.length / 2, this.track.length) - this.track.length / 2;
      const dv = (a.speed ?? a.longitudinalSpeed) - (b.speed ?? b.longitudinalSpeed);
      const ttc = Math.abs(dv) > .1 ? Math.max(0, Math.abs(ds) / Math.abs(dv)) : Infinity;
      const reachable = Math.abs(ds) < Math.max(48, Math.max(a.speed, b.speed) * 5.5) && (ttc < 7.5 || Math.abs(ds) < 18);
      if (!reachable) continue;
      const overlap = Math.abs(ds) < a.spec.halfLength + b.spec.halfLength + 1.25
        && Math.abs(a.lateral - b.lateral) < a.spec.halfWidth + b.spec.halfWidth + .65;
      const front = ds >= 0;
      edges.push({ a: a.id, b: b.id, ds, dv, ttc, overlap, front,
        leftSpace: this.track.halfWidth - Math.max(a.lateral, b.lateral),
        rightSpace: this.track.halfWidth + Math.min(a.lateral, b.lateral),
        closing: dv * Math.sign(ds || 1), occupancy: occupancy.at(b, clamp(ttc, 0, 5)) });
    }
    this.edges = edges; this.meaningfulTime += edges.some(edge => edge.ttc < 4.5 || edge.overlap) ? dt : 0;
    return edges;
  }
}
