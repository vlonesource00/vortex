// Clear-air calibration, blended with the established traffic envelope.
// Accelerations are m/s²; slipTarget is radians; lookahead is seconds.
// These are driver demands, not changes to vehicle or tyre forces.
export const PACE={
  corner:12.5,lineBrake:10,brake:10,brakeMax:12,lateralReserve:.9,
  throttleGain:.6,throttleBase:.35,brakeGain:.5,
  rotation:2.5,slipTarget:.12,lookahead:.3,slipCompensation:.85,thermalHorizon:20,
};
