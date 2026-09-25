/**
 * Racecraft laboratory (brief sections 9-12, 39-40).
 *
 * Deterministic matrix of pace delta x initial gap x rival behaviour. Records
 * the section 40 scorecard for every scenario so racecraft decisions are made
 * from campaign results, not from reading code.
 *
 *   node tools/racecraft-lab.mjs
 *   node tools/racecraft-lab.mjs --quick
 */
import { Track } from '../src/sim/track.js';
import { wrap } from '../src/sim/math.js';
import { VortexSession } from '../src/vortex-session.js';

const argv = process.argv.slice(2);
const QUICK = argv.includes('--quick');
const SECONDS = 22;

const track = new Track('harbor-ring');
const HALF_L = 2.3, HALF_W = 0.99;

// Section 10/11/12 matrices.
const PACE_DELTAS = QUICK ? [-5, 0, 3] : [-8, -5, -3, -1.5, 0, 1.5, 3];
const GAPS = QUICK ? [15, 40] : [8, 15, 25, 40, 60];
const BEHAVIOURS = QUICK ? ['hold', 'defend-inside'] : ['hold', 'defend-inside', 'defend-outside', 'brake-early', 'yield'];

function place(car, s, lateral, speed) {
  const p = track.at(wrap(s, track.length), lateral);
  car.s = wrap(s, track.length);
  car.x = p.x; car.z = p.z; car.yaw = p.heading;
  car.lateral = lateral;
  car.u = speed; car.v = 0; car.speed = speed;
  car.controls = { throttle: 0, brake: 0, steer: 0 };
}

/** Free-air reference: what the same rolling start does with no rival. */
function freeAirReference(startS, speed) {
  const solo = new VortexSession(track, { classId: 'gt' });
  solo.mode = 'practice'; solo.field = 1; solo.laps = 1; solo.autopilot = true;
  solo.start({ freshTrack: true });
  solo.phase = 'racing'; solo.countdown = 0;
  place(solo.player, startS, 0, speed);
  const zero = { steer: 0, throttle: 0, brake: 0 };
  const map = new Map();
  let t = 0;
  while (t < SECONDS + 8) {
    solo.step(1 / 120, zero);
    t += 1 / 120;
    if (solo.phase !== 'racing') continue;
    const key = Math.round(solo.player.s);
    if (!map.has(key)) map.set(key, { time: t, speed: solo.player.speed });
  }
  return map;
}

function scenario({ delta, gap, behaviour }) {
  const egoSpeed = 42;
  const rivalSpeed = egoSpeed - delta;
  const startS = 200;
  const freeAir = freeAirReference(startS, egoSpeed);

  const session = new VortexSession(track, { classId: 'gt' });
  session.mode = 'race'; session.field = 2; session.laps = 1; session.autopilot = true;
  session.start({ freshTrack: true });
  session.phase = 'racing'; session.countdown = 0;

  const ego = session.cars[0], rival = session.cars[1];
  place(ego, startS, 0, egoSpeed);
  place(rival, startS + gap, 0, rivalSpeed);
  // Rival defence offset: which side of the track it protects.
  const rivalHome = behaviour === 'defend-inside' ? -2.2 : behaviour === 'defend-outside' ? 2.2 : 0;

  const zero = { steer: 0, throttle: 0, brake: 0 };
  const driver = session.drivers[0];
  let t = 0, overlap = false, noseAhead = false, fullClear = false;
  let firstOverlapT = null, firstClearT = null, retained = true;
  let minClearance = 99, contacts = 0, offtrack = 0, safetyInterventions = 0, lastImpact = 0;
  let throttleLifts = 0, lastThrottle = 0, brakeEvents = 0, lastBrake = 0;
  let routeReversals = 0, lastFlankSign = 0, lastTargetQ = 0;
  let targetSum = 0, targetN = 0, speedSum = 0, speedN = 0;
  let wakeSum = 0, wakeN = 0;
  let timeLost = 0;
  let attackAttempted = false, committedT = null;

  while (t < SECONDS) {
    // Rival behaviour model. Physically driven, not pinned to a world velocity.
    const gapNow = wrap(rival.s - ego.s + track.length / 2, track.length) - track.length / 2;
    if (behaviour === 'hold' || behaviour === 'defend-inside' || behaviour === 'defend-outside') {
      const err = rivalSpeed - rival.speed;
      rival.controls = { throttle: err > 0.2 ? 1 : err < -0.2 ? 0 : 0.6, brake: err < -1.2 ? 0.5 : 0, steer: 0 };
      // Steer toward the defensive line when the attacker is close.
      const want = rivalHome;
      rival.controls.steer = Math.max(-0.12, Math.min(0.12, (want - rival.lateral) * 0.05));
    } else if (behaviour === 'brake-early') {
      const brakeS = 420;
      const d = wrap(brakeS - rival.s + track.length / 2, track.length) - track.length / 2;
      rival.controls = d < 60 ? { throttle: 0, brake: 0.45, steer: 0 } : { throttle: rivalSpeed > rival.speed ? 1 : 0.4, brake: 0, steer: 0 };
    } else if (behaviour === 'yield') {
      if (Math.abs(gapNow) < 12) {
        rival.controls = { throttle: 0, brake: 0.15, steer: 0 };
        rival.controls.steer = Math.max(-0.1, Math.min(0.1, -Math.sign(ego.lateral - rival.lateral || 1) * 0.06));
      } else {
        rival.controls = { throttle: rivalSpeed > rival.speed ? 1 : 0.5, brake: 0, steer: 0 };
      }
    }

    session.step(1 / 120, zero);
    t += 1 / 120;
    if (session.phase !== 'racing') break;

    const car = session.player;
    const ds = wrap(rival.s - car.s + track.length / 2, track.length) - track.length / 2;
    const latGap = Math.abs(rival.lateral - car.lateral) - HALF_W * 2;
    const longGap = Math.abs(ds) - HALF_L * 2;
    const clearance = Math.hypot(Math.max(0, latGap), Math.max(0, longGap));
    minClearance = Math.min(minClearance, clearance);

    // Pass lifecycle (section 25).
    if (!overlap && longGap < 0 && latGap < 1.5) { overlap = true; firstOverlapT = t; attackAttempted = true; }
    if (!noseAhead && ds < -0.6) { noseAhead = true; }
    if (!fullClear && overlap && clearance > 2.2) {
      fullClear = true; firstClearT = t;
    }
    if (fullClear && clearance < 1.2) retained = false;

    if (car.impact > 0.02 && lastImpact <= 0.02) contacts++;
    lastImpact = car.impact;
    if (Math.abs(car.lateral) > track.halfWidth - HALF_W) offtrack++;
    if (driver?.recover || driver?.safety?.active) safetyInterventions++;

    // Throttle lifts and brake events without a physical reason.
    const th = car.controls.throttle, br = car.controls.brake;
    if (lastThrottle > 0.7 && th < 0.25 && br < 0.1) throttleLifts++;
    if (br > 0.15 && lastBrake <= 0.15) brakeEvents++;
    lastThrottle = th; lastBrake = br;

    // Route churn geometrically (section 19), not by candidate id.
    const tq = driver?.planner?.at(car.s + 25)?.offset ?? 0;
    const flankSign = Math.sign(tq - car.lateral || 0);
    if (lastFlankSign && flankSign && flankSign !== lastFlankSign && driver?.planner?.attack?.active?.committed) routeReversals++;
    if (flankSign) lastFlankSign = flankSign;
    lastTargetQ = tq;

    // Engagement tax (section 20): speed given away with no real conflict.
    const ref = freeAir.get(Math.round(car.s));
    if (ref && !overlap) {
      const freeTarget = ref.speed;
      const planned = Number.isFinite(driver?.planSpeed) ? driver.planSpeed : freeTarget;
      targetSum += Math.max(0, freeTarget - planned); targetN++;
      speedSum += Math.max(0, freeTarget - car.speed); speedN++;
    }
    wakeSum += car.aero.wake; wakeN++;
    if (ref) timeLost += Math.max(0, (t - ref.time) * 0);
  }

  const meanTargetDeficit = targetN ? targetSum / targetN : 0;
  const meanSpeedDeficit = speedN ? speedSum / speedN : 0;
  return {
    delta, gap, behaviour,
    attackAttempted, overlap, noseAhead, fullClear, retained: fullClear && retained,
    timeToOverlap: firstOverlapT, timeToClear: firstClearT,
    minClearance: Number(minClearance.toFixed(2)),
    contacts, offtrack, safetyInterventions,
    throttleLifts, brakeEvents,
    routeReversals,
    meanTargetDeficit: Number(meanTargetDeficit.toFixed(2)),
    meanSpeedDeficit: Number(meanSpeedDeficit.toFixed(2)),
    meanWake: Number((wakeSum / Math.max(1, wakeN)).toFixed(3)),
  };
}

console.log('=== RACECRAFT LABORATORY ===');
console.log(`scenarios: ${PACE_DELTAS.length} pace x ${GAPS.length} gaps x ${BEHAVIOURS.length} behaviours = `
  + `${PACE_DELTAS.length * GAPS.length * BEHAVIOURS.length}   (${QUICK ? 'quick' : 'full'} matrix)\n`);

const rows = [];
for (const delta of PACE_DELTAS) {
  for (const gap of GAPS) {
    for (const behaviour of BEHAVIOURS) {
      const r = scenario({ delta, gap, behaviour });
      rows.push(r);
      console.log(
        `d=${String(delta).padStart(5)}  gap=${String(gap).padStart(2)}  ${behaviour.padEnd(14)} | ` +
        `att=${r.attackAttempted ? 'Y' : 'n'} ovl=${r.overlap ? 'Y' : 'n'} nose=${r.noseAhead ? 'Y' : 'n'} ` +
        `clr=${r.fullClear ? 'Y' : 'n'} ret=${r.retained ? 'Y' : 'n'} | ` +
        `clrMin=${String(r.minClearance).padStart(5)} ctc=${r.contacts} off=${r.offtrack} ` +
        `lift=${String(r.throttleLifts).padStart(3)} rev=${String(r.routeReversals).padStart(2)} | ` +
        `tDef=${String(r.meanTargetDeficit).padStart(5)} vDef=${String(r.meanSpeedDeficit).padStart(5)} wake=${r.meanWake}`,
      );
    }
  }
}

// Section 41-43 aggregation.
const band = (d) => d <= -3 ? 'slower' : d >= 1.5 ? 'faster' : 'equal';
console.log('\n=== SCORECARD BY RIVAL CLASS ===');
console.log('class   n  attempt overlap nose  clear retained | meanTDef meanVDef contacts routeRev throttleLifts');
for (const b of ['slower', 'equal', 'faster']) {
  const g = rows.filter(r => band(r.delta) === b);
  if (!g.length) continue;
  const f = (k) => g.filter(r => r[k]).length;
  const avg = (k) => g.reduce((a, r) => a + r[k], 0) / g.length;
  console.log(
    b.padEnd(7) + String(g.length).padStart(2) +
    String(f('attackAttempted')).padStart(8) + String(f('overlap')).padStart(7) +
    String(f('noseAhead')).padStart(6) + String(f('fullClear')).padStart(6) +
    String(f('retained')).padStart(9) + ' | ' +
    avg('meanTargetDeficit').toFixed(2).padStart(7) + avg('meanSpeedDeficit').toFixed(2).padStart(9) +
    String(g.reduce((a, r) => a + r.contacts, 0)).padStart(9) +
    String(g.reduce((a, r) => a + r.routeReversals, 0)).padStart(9) +
    String(g.reduce((a, r) => a + r.throttleLifts, 0)).padStart(13),
  );
}

console.log('\n=== SECTION 22 SIDE-BY-SIDE THROTTLE BEHAVIOUR ===');
const sbs = rows.filter(r => r.overlap);
console.log(`scenarios with overlap: ${sbs.length}`);
console.log(`  total unnecessary throttle lifts during pursuit: ${rows.reduce((a, r) => a + r.throttleLifts, 0)}`);
console.log(`  total route reversals after commitment: ${rows.reduce((a, r) => a + r.routeReversals, 0)}`);
console.log(`  total contacts: ${rows.reduce((a, r) => a + r.contacts, 0)}`);
