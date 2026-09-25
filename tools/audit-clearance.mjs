/**
 * Deterministic geometry-only clearance audit. No simulation, no timing.
 *
 * Places ego and rival at the same longitudinal station with parallel headings
 * and sweeps the LATERAL BODY CLEARANCE (the gap between the two car bodies,
 * not centre-to-centre). For each clearance it asks every combat subsystem what
 * it thinks the situation is. The point is to find out whether they agree.
 *
 *   node tools/audit-clearance.mjs
 */
import { OpportunityField } from '../src/ai/vortex/interaction/opportunity-field.js';
import { CorridorOwnership } from '../src/ai/vortex/interaction/corridor-ownership.js';
import { BrakeEvents } from '../src/ai/vortex/planning/brake-events.js';
import { classifyConflict, restrictsSpeed, CLEARANCE } from '../src/ai/vortex/interaction/clearance.js';
import { CAR_CLASSES } from '../src/sim/car-specs.js';

const spec = CAR_CLASSES.gt;
const ego = { s: 0, lateral: 0, speed: 35, spec };
const BODY_LATERAL = spec.halfWidth * 2;
const BODY_LONGITUDINAL = spec.halfLength * 2;
const track = { halfWidth: 7.5, length: 2704.62 };
const CLEARANCES = [0.20, 0.30, 0.34, 0.40, 0.42, 0.44, 0.50, 0.60, 0.72, 0.80, 1.00];

console.log('=== CLEARANCE CONSTANT AUDIT ===');
console.log(`GT body: halfWidth=${spec.halfWidth}  halfLength=${spec.halfLength}`);
console.log(`body-lateral sum = ${BODY_LATERAL}   body-longitudinal sum = ${BODY_LONGITUDINAL}\n`);
console.log('concept'.padEnd(34) + 'margin  axis');
for (const [k, v] of Object.entries(CLEARANCE)) {
  const axis = /LONGITUDINAL/.test(k) ? 'longitudinal' : 'lateral';
  console.log(k.padEnd(34) + String(v).padEnd(8) + axis);
}
console.log(`\n>>> flank ${CLEARANCE.ATTACK_CORRIDOR_MARGIN} vs interaction ${CLEARANCE.INTERACTION_MARGIN}`
  + ` => flank sits ${(CLEARANCE.ATTACK_CORRIDOR_MARGIN - CLEARANCE.INTERACTION_MARGIN).toFixed(2)} m OUTSIDE the traffic-conflict boundary`);

console.log('\n=== SIDE-BY-SIDE SWEEP (same station, parallel, constant speed) ===');
console.log('clear | gen-flank | refiner | state               | contact | ownership | brake');
console.log('------+-----------+---------+---------------------+---------+-----------+------');

for (const c of CLEARANCES) {
  const rivalLateral = 0;
  const egoLateral = BODY_LATERAL + c;
  const rival = { id: 'R', s: 0, lateral: rivalLateral, speed: 20, spec };

  const path = {
    id: `FLANK_${c}`,
    points: [{ s: 0, distance: 0, offset: egoLateral, speed: 35, speedLimit: 35, freeSpeedLimit: 35, curvature: 0, demand: 0, lateralLimit: 1 }],
  };
  const occupancy = { at: () => ({ s: rival.s, lateral: rival.lateral, speed: rival.speed, halfLength: spec.halfLength, halfWidth: spec.halfWidth }) };
  new OpportunityField(track).score(path, ego, [rival], occupancy, []);

  const ownership = new CorridorOwnership(track);
  ownership.update(ego, [{ ...rival, spec }]);
  const owned = ownership.owners.size > 0;

  const brake = new BrakeEvents();
  brake.update(
    { points: [{ s: 0, distance: 0, offset: egoLateral, speed: 35, speedLimit: 35 }, { s: 20, distance: 20, offset: egoLateral, speed: 35, speedLimit: 35 }] },
    0, track.length, { ego, rivals: [{ ...rival, spec }] }, occupancy,
  );

  const latGap = c;                    // signed body clearance
  const alongGap = -BODY_LONGITUDINAL; // same station => full longitudinal overlap
  const state = classifyConflict(latGap, alongGap, 15, 0);
  const capped = restrictsSpeed(state);
  const requestedFlank = Math.abs(c - CLEARANCE.ATTACK_CORRIDOR_MARGIN) < 1e-9;

  console.log(
    String(c.toFixed(2)).padEnd(7) +
    (requestedFlank ? 'YES     ' : 'no      ') +
    (capped ? 'CAP   ' : 'ok    ') + ' ' +
    state.padEnd(21) + ' ' +
    path.breakdown.contact.toFixed(1).padStart(6) + '  ' +
    (owned ? 'OWNED   ' : 'clear   ') + '  ' +
    brake.reason,
  );
}

console.log('\n=== INTERPRETATION ===');
console.log('The generated attack flank is at CLEARANCE.ATTACK_CORRIDOR_MARGIN.');
console.log('It must be: not interaction-capped, near-zero contact cost, and usable.');
console.log('Contact cost must fall to near zero once the bodies are separated,');
console.log('and must remain extremely expensive on overlap or crossing.');
