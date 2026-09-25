export class LapResidualLearner {
  constructor(memory, atlas) { this.memory = memory; this.atlas = atlas; this.lapEligible = true; this.lastLap = null; this.telemetry = { acceptedLaps: 0, rejectedLaps: 0, updatedCells: 0 }; }
  update(ego, flags, dt) {
    const lap = ego.race?.lap ?? 0;
    if (this.lastLap !== null && ego.race && lap !== this.lastLap) {
      const result = this.memory.finishLap(Boolean(this.lapEligible && ego.race.valid));
      if (result.accepted) { this.telemetry.acceptedLaps++; this.telemetry.updatedCells += result.cells; }
      else this.telemetry.rejectedLaps++;
      this.lapEligible = Boolean(ego.race.valid);
    }
    const eligible = Boolean(ego.race?.valid) && !flags.trafficLimited && !flags.contact && !flags.offtrack && !flags.recovery && !flags.safety && !flags.overlap;
    this.lapEligible &&= eligible;
    if (eligible) {
      const nominal = this.atlas.sample(ego.s), speedResidual = Math.max(-.45, Math.min(.45, ego.speed - nominal.speed));
      this.memory.record(ego.s, { q: Math.max(-.25, Math.min(.25, ego.lateral - nominal.offset)),
        speedResidual, elapsed: dt, exitGain: Math.max(-.15, Math.min(.15, ego.u - nominal.speed)) }, true);
    }
    this.lastLap = lap;
  }
}
