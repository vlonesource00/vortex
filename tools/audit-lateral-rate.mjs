/**
 * Controlled lateral-rate sweep (brief section 7).
 *
 * Builds candidate segments with KNOWN lateral translations and reports the
 * lateral velocity that the old pseudo-tick estimate produced, the physical
 * estimate now used, the conflict classification, and the contact score.
 *
 * TrajectoryRefiner and OpportunityField both call
 * clearance.candidateLateralVelocity, so they cannot drift apart; this tool
 * verifies the shared estimate against the analytic expectation.
 *
 *   node tools/audit-lateral-rate.mjs
 */
import { candidateLateralVelocity, classifyConflict, restrictsSpeed } from '../src/ai/vortex/interaction/clearance.js';
import { OpportunityField } from '../src/ai/vortex/interaction/opportunity-field.js';
import { CAR_CLASSES } from '../src/sim/car-specs.js';

const spec = CAR_CLASSES.gt;
const track = { halfWidth: 7.5, length: 2704.62 };
const ego = { s: 0, lateral: 0, speed: 40, spec };

// (label, lateral translation m, path length m)
const TRANSITIONS = [
  ['1 m over 10 m', 1, 10],
  ['2 m over 10 m', 2, 10],
  ['3 m over 20 m', 3, 20],
  ['3 m over 40 m', 3, 40],
];
const SPEEDS = [20, 30, 40, 50, 60];

console.log('=== LATERAL-RATE UNITS AUDIT ===');
console.log('old pseudo-tick estimate: dq * 27   (candidate points are 10 m apart, not 1 tick)');
console.log('corrected physical:       (dq / ds) * v\n');

console.log('transition    v  |  expected   old-tick   corrected  |  conflict            cap | contact');
console.log('----------------+-----------------------------------+-----------------------+--------');

let worstErr = 0;
for (const [label, dq, ds] of TRANSITIONS) {
  for (const v of SPEEDS) {
    // Analytic expectation from the actual path geometry.
    const expected = (dq / ds) * v;
    const oldTick = dq * 27;
    const corrected = candidateLateralVelocity(dq, ds, v);
    worstErr = Math.max(worstErr, Math.abs(corrected - expected));

    // Rival sits ahead on the same line; candidate swings across.
    const rival = { id: 'R', s: 25, lateral: 0, speed: 20, spec };
    const egoLateral = 0.5;
    const path = {
      id: label,
      points: [
        { s: 0, distance: 0, offset: egoLateral, speed: v, speedLimit: v, freeSpeedLimit: v, curvature: 0, demand: 0, lateralLimit: 1 },
        { s: ds, distance: ds, offset: egoLateral + dq, speed: v, speedLimit: v, freeSpeedLimit: v, curvature: 0, demand: 0, lateralLimit: 1 },
      ],
    };
    const occupancy = {
      at: (r, t) => ({ s: r.s + r.speed * (t ?? 0), lateral: r.lateral, speed: r.speed, halfLength: spec.halfLength, halfWidth: spec.halfWidth }),
    };
    new OpportunityField(track).score(path, { ...ego, speed: v }, [rival], occupancy, []);

    // Closing rate in the same convention the refiner uses.
    const dt = 1 / 27;
    const gapNow = Math.abs(rival.lateral - egoLateral);
    const gapFuture = Math.abs(rival.lateral - (egoLateral + corrected * dt));
    const relSpeedLat = Math.max(0, (gapNow - gapFuture) / dt);
    const relSpeedLong = Math.abs(v - rival.speed);
    const latGap = Math.abs(rival.lateral - egoLateral) - (spec.halfWidth * 2);
    const alongGap = Math.abs(rival.s) - (spec.halfLength * 2);
    const state = classifyConflict(latGap, alongGap, relSpeedLong, relSpeedLat, true);

    console.log(
      label.padEnd(14) + String(v).padEnd(4) + '| ' +
      expected.toFixed(3).padStart(8) + oldTick.toFixed(3).padStart(11) + corrected.toFixed(3).padStart(11) + '  | ' +
      state.padEnd(21) + (restrictsSpeed(state) ? 'CAP ' : 'ok  ') + '| ' +
      path.breakdown.contact.toFixed(1),
    );
  }
}

console.log(`\nmax |corrected - analytic| = ${worstErr.toExponential(2)}  (shared helper, both systems)`);
console.log('\n=== INTERPRETATION ===');
console.log('The old estimate is speed-independent and always dq*27, so a 2 m');
console.log('transition over 10 m reported 54 m/s of lateral speed at every');
console.log('longitudinal speed. The physical estimate scales with v and matches');
console.log('the analytic value exactly.');
