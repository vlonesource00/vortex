import { wrap } from '../../../sim/math.js';

export class LapMemory {
  constructor(track, cells = 120) {
    this.track = track; this.cells = cells; this.samples = Array.from({ length: cells }, () => ({
      count: 0, bestTime: Infinity, residualQ: 0, residualSpeed: 0, brakeShift: 0, exitGain: 0,
    }));
    this.lastProgress = null; this.lapSamples = new Map(); this.learnedLaps = 0;
  }
  index(s) { return Math.floor(wrap(s, this.track.length) / this.track.length * this.cells + 1e-9) % this.cells; }
  residualAt(s) { return this.samples[this.index(s)]; }
  record(s, value, eligible) {
    if (!eligible) { this.lapSamples.clear(); return; }
    const cell = this.index(s), prior = this.lapSamples.get(cell);
    const n = (prior?.n ?? 0) + 1, blend = 1 / n;
    this.lapSamples.set(cell, {
      q: (prior?.q ?? 0) + (Number(value.q ?? 0) - (prior?.q ?? 0)) * blend,
      speedResidual: (prior?.speedResidual ?? 0) + (Number(value.speedResidual ?? 0) - (prior?.speedResidual ?? 0)) * blend,
      brakeShift: (prior?.brakeShift ?? 0) + (Number(value.brakeShift ?? 0) - (prior?.brakeShift ?? 0)) * blend,
      exitGain: (prior?.exitGain ?? 0) + (Number(value.exitGain ?? 0) - (prior?.exitGain ?? 0)) * blend,
      elapsed: (prior?.elapsed ?? 0) + Number(value.elapsed ?? 0), n,
    });
  }
  finishLap(valid) {
    if (!valid || this.lapSamples.size < this.cells * .65) { this.lapSamples.clear(); return { accepted: false, cells: 0 }; }
    let changed = 0;
    for (const [index, sample] of this.lapSamples) {
      const cell = this.samples[index];
      const q = Math.max(-.35, Math.min(.35, Number(sample.q ?? 0)));
      const speed = Math.max(-.6, Math.min(.6, Number(sample.speedResidual ?? 0)));
      cell.residualQ += Math.max(-.035, Math.min(.035, (q - cell.residualQ) * .12));
      cell.residualSpeed += Math.max(-.06, Math.min(.06, (speed - cell.residualSpeed) * .12));
      cell.brakeShift += Math.max(-.2, Math.min(.2, (Number(sample.brakeShift ?? cell.brakeShift) - cell.brakeShift) * .08));
      cell.exitGain += Math.max(-.15, Math.min(.15, (Number(sample.exitGain ?? cell.exitGain) - cell.exitGain) * .08));
      cell.count++; cell.bestTime = Math.min(cell.bestTime, sample.elapsed); changed++;
    }
    this.learnedLaps++; this.lapSamples.clear();
    return { accepted: changed > 0, cells: changed };
  }
  resetSession() { this.lapSamples.clear(); this.lastProgress = null; }
}
