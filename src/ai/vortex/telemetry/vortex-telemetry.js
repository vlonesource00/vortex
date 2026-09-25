export class VortexTelemetry {
  constructor() { this.reset(); }
  reset() {
    Object.assign(this, { steps: 0, solverTimeMs: 0, solverMaxMs: 0, safetyInterventions: 0,
      controllerFallbacks: 0, interactionBrakeSteps: 0, cornerBrakeSteps: 0, meaningfulInteractionTime: 0,
      retainedPasses: 0, defensiveEpisodes: 0, cleanLearnerCells: 0, paceLoss: { line: 0, brake: 0,
        minimumSpeed: 0, exit: 0, traffic: 0, control: 0 } });
  }
  update({ dt, planner, safety, optimizer, brakeReason, engaged, retainedPass, defensive, learner }) {
    this.steps++; this.solverTimeMs += planner.lastSolveMs; this.solverMaxMs = Math.max(this.solverMaxMs, planner.lastSolveMs);
    this.safetyInterventions = safety.interventions; this.controllerFallbacks = optimizer.fallbacks;
    if (brakeReason === 'interaction') this.interactionBrakeSteps++;
    if (brakeReason === 'corner') this.cornerBrakeSteps++;
    if (engaged) this.meaningfulInteractionTime += dt;
    if (retainedPass) this.retainedPasses++;
    if (defensive) this.defensiveEpisodes++;
    this.cleanLearnerCells = learner.telemetry.updatedCells;
  }
  snapshot() { return { ...this, paceLoss: { ...this.paceLoss }, meanSolverMs: this.steps ? this.solverTimeMs / this.steps : 0 }; }
}
