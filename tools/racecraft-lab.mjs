/**
 * Racecraft laboratory (brief sections 9-12, 39-40; repaired per section 13).
 *
 * Deterministic matrix of pace delta x initial gap x rival behaviour. Records
 * the scorecard for every scenario so racecraft decisions are made
 * from campaign results, not intuition.
 *
 *   node tools/racecraft-lab.mjs
 *   node tools/racecraft-lab.mjs --quick
 */
import { Track } from '../src/sim/track.js';
import { wrap, clamp } from '../src/sim/math.js';
import { VortexSession } from '../src/vortex-session.js';

const argv = process.argv.slice(2);
const QUICK = argv.includes('--quick');
const SECONDS = 22;

const track = new Track('harbor-ring');
const HALF_L = 2.3, HALF_W = 0.99;

// Section 10/11/12 matrices.
// Pace delta: positive means rival is SLOWER than ego (egoSpeed - rivalSpeed).
const PACE_DELTAS = QUICK ? [3, 0, -3] : [8, 5, 3, 1.5, 0, -1.5, -3];
const GAPS = QUICK ? [15, 40] : [8, 15, 25, 40, 60];
const BEHAVIOURS = QUICK ? ['hold', 'defend-inside'] : ['hold', 'defend-inside', 'defend-outside', 'brake-early', 'yield'];

/**
 * Place a car rolling at speed in a physically consistent gear, engine RPM, and race state.
 * Repaired per Section 13 item 2.
 */
function placeRolling(car, s, lateral, speed) {
  car.place(track, s, lateral, speed);
  const SPEC = car.spec;
  let best = 1, bestErr = Infinity;
  for (let g = 1; g < SPEC.gears.length; g++) {
    const rpm = Math.abs(speed) / SPEC.radius * SPEC.gears[g] * SPEC.finalDrive * 60 / (2 * Math.PI);
    const err = Math.abs(rpm - 5800);
    if (err < bestErr) { bestErr = err; best = g; }
  }
  car.gear = best;
  car.rpm = Math.abs(speed) / SPEC.radius * SPEC.gears[best] * SPEC.finalDrive * 60 / (2 * Math.PI);
  car.race = {
    progress: car.s,
    previousS: car.s,
    lap: 1,
    lastLap: null,
    bestLap: null,
    lapStart: 0,
    sector: 0,
    valid: true,
    sectors: [],
    finishTime: null,
    offtrack: 0,
  };
  return car;
}

/** Free-air reference: what the same rolling start does with no rival. */
function freeAirReference(startS, speed) {
  const solo = new VortexSession(track, { classId: 'gt' });
  solo.mode = 'practice'; solo.field = 1; solo.laps = 1; solo.autopilot = true;
  solo.start({ freshTrack: true });
  solo.phase = 'racing'; solo.countdown = 0;
  placeRolling(solo.player, startS, 0, speed);
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

/**
 * Scripted rival driver that runs inside session.step() and cannot be overwritten.
 * Repaired per Section 13 item 3.
 */
class ScriptedRivalDriver {
  constructor(behaviour, targetSpeed, homeOffset) {
    this.behaviour = behaviour;
    this.targetSpeed = targetSpeed;
    this.homeOffset = homeOffset;
  }
  update(car, cars, dt, context) {
    const ego = cars.find(c => c.id !== car.id);
    const gapToEgo = ego ? wrap(car.s - ego.s + track.length / 2, track.length) - track.length / 2 : 50;

    let wantSpeed = this.targetSpeed;
    let wantLateral = this.homeOffset;

    if (this.behaviour === 'brake-early') {
      const brakeS = 420;
      const distToBrake = wrap(brakeS - car.s + track.length / 2, track.length) - track.length / 2;
      if (distToBrake > 0 && distToBrake < 65) {
        wantSpeed = this.targetSpeed * 0.52;
      }
    } else if (this.behaviour === 'yield') {
      if (Math.abs(gapToEgo) < 14) {
        wantSpeed = this.targetSpeed * 0.70;
        const egoSide = ego ? Math.sign(ego.lateral - car.lateral || 1) : 1;
        wantLateral = -egoSide * 2.8;
      }
    }

    // Speed control
    const vErr = wantSpeed - car.speed;
    let throttle = 0, brake = 0;
    if (vErr > 0.25) {
      throttle = clamp(vErr * 0.5 + 0.35, 0.4, 1.0);
      brake = 0;
    } else if (vErr < -0.8) {
      throttle = 0;
      brake = clamp(-vErr * 0.35, 0.2, 0.85);
    } else {
      throttle = 0.35;
      brake = 0;
    }

    // Steering control towards wantLateral
    const latErr = wantLateral - car.lateral;
    const steer = clamp(latErr * 0.08, -0.22, 0.22);

    car.controls = { throttle, brake, steer, reverse: false };
  }
}

function scenario({ delta, gap, behaviour }) {
  const egoSpeed = 42;
  // delta is egoSpeed - rivalSpeed: positive means rival is SLOWER
  const rivalSpeed = egoSpeed - delta;
  const startS = 200;
  const freeAir = freeAirReference(startS, egoSpeed);

  const session = new VortexSession(track, { classId: 'gt' });
  session.mode = 'race'; session.field = 2; session.laps = 1; session.autopilot = true;
  session.start({ freshTrack: true });
  session.phase = 'racing'; session.countdown = 0;

  const ego = session.cars[0], rival = session.cars[1];
  placeRolling(ego, startS, 0, egoSpeed);
  placeRolling(rival, startS + gap, 0, rivalSpeed);

  // Rival defence offset: which side of the track it protects.
  const rivalHome = behaviour === 'defend-inside' ? -2.2 : behaviour === 'defend-outside' ? 2.2 : 0;
  session.drivers[1] = new ScriptedRivalDriver(behaviour, rivalSpeed, rivalHome);

  const zero = { steer: 0, throttle: 0, brake: 0 };
  const driver = session.drivers[0];
  let t = 0, overlap = false, noseAhead = false, fullClear = false;
  let firstOverlapT = null, firstClearT = null, retained = true;
  let minClearance = 99, contacts = 0, lightTouches = 0, hardCollisions = 0, offtrack = 0, safetyInterventions = 0, lastImpact = 0;
  let throttleLifts = 0, lastThrottle = 0, brakeEvents = 0, lastBrake = 0;
  let routeReversals = 0, lastFlankSign = 0;
  let targetSum = 0, targetN = 0, speedSum = 0, speedN = 0;
  let wakeSum = 0, wakeN = 0;
  let maxTimeLost = 0;
  let attackAttempted = false;

  while (t < SECONDS) {
    session.step(1 / 120, zero);
    t += 1 / 120;
    if (session.phase !== 'racing') break;

    const car = session.player;
    const ds = wrap(rival.s - car.s + track.length / 2, track.length) - track.length / 2;
    const latGap = Math.abs(rival.lateral - car.lateral) - HALF_W * 2;
    const longGap = Math.abs(ds) - HALF_L * 2;
    const clearance = Math.hypot(Math.max(0, latGap), Math.max(0, longGap));
    minClearance = Math.min(minClearance, clearance);

    // Pass lifecycle (repaired per Section 13 item 7).
    if (!overlap && longGap < 0 && latGap < 2.0) {
      overlap = true;
      firstOverlapT = t;
      attackAttempted = true;
    }
    if (!noseAhead && ds < -0.2) {
      noseAhead = true;
    }
    // Full clear requires ego rear ahead of rival front + margin (Section 13 item 7)
    const clearThreshold = -(2 * HALF_L + 0.5); // ~ -5.1m
    if (!fullClear && overlap && ds < clearThreshold) {
      fullClear = true;
      firstClearT = t;
    }
    // Retained clear check: once cleared, did rival regain overlap?
    if (fullClear && ds > -HALF_L) {
      retained = false;
    }

    // Contact severity (repaired per Section 13 item 8).
    if (car.impact > 0.02 && lastImpact <= 0.02) {
      contacts++;
      if (car.impact < 0.15) lightTouches++;
      else hardCollisions++;
    }
    lastImpact = car.impact;

    if (Math.abs(car.lateral) > track.halfWidth - HALF_W) offtrack++;
    if (driver?.recover || driver?.safety?.active) safetyInterventions++;

    // Throttle lifts and brake events without a physical reason.
    const th = car.controls.throttle, br = car.controls.brake;
    if (lastThrottle > 0.7 && th < 0.25 && br < 0.1) throttleLifts++;
    if (br > 0.15 && lastBrake <= 0.15) brakeEvents++;
    lastThrottle = th; lastBrake = br;

    // Route reversals relative to rival (repaired per Section 13 item 6).
    const planOffset = driver?.planner?.at(car.s + 20)?.offset ?? 0;
    const flankSign = Math.sign(planOffset - rival.lateral);
    if (lastFlankSign !== 0 && flankSign !== 0 && flankSign !== lastFlankSign && (driver?.planner?.attack?.active?.committed || overlap)) {
      routeReversals++;
    }
    if (flankSign !== 0) lastFlankSign = flankSign;

    // Engagement tax: target-deficit calculation (repaired per Section 13 item 4).
    const ref = freeAir.get(Math.round(car.s));
    if (ref && !overlap) {
      const freeTarget = ref.speed;
      const planned = typeof driver?.planSpeed === 'function' ? driver.planSpeed(car.s) : freeTarget;
      targetSum += Math.max(0, freeTarget - planned);
      targetN++;
      speedSum += Math.max(0, freeTarget - car.speed);
      speedN++;
    }
    wakeSum += car.aero.wake;
    wakeN++;

    // Station-matched time loss (repaired per Section 13 item 5).
    if (ref) {
      const currentDelta = t - ref.time;
      if (currentDelta > maxTimeLost) maxTimeLost = currentDelta;
    }
  }

  const meanTargetDeficit = targetN ? targetSum / targetN : 0;
  const meanSpeedDeficit = speedN ? speedSum / speedN : 0;
  return {
    delta, gap, behaviour,
    attackAttempted, overlap, noseAhead, fullClear, retained: fullClear && retained,
    timeToOverlap: firstOverlapT, timeToClear: firstClearT,
    minClearance: Number(minClearance.toFixed(2)),
    contacts, lightTouches, hardCollisions, offtrack, safetyInterventions,
    throttleLifts, brakeEvents,
    routeReversals,
    meanTargetDeficit: Number(meanTargetDeficit.toFixed(2)),
    meanSpeedDeficit: Number(meanSpeedDeficit.toFixed(2)),
    meanWake: Number((wakeSum / Math.max(1, wakeN)).toFixed(3)),
    maxTimeLost: Number(maxTimeLost.toFixed(3)),
  };
}

console.log('=== RACECRAFT LABORATORY (REPAIRED) ===');
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
        `clrMin=${String(r.minClearance).padStart(5)} ctc=${r.contacts}(${r.lightTouches}r/${r.hardCollisions}h) off=${r.offtrack} ` +
        `lift=${String(r.throttleLifts).padStart(3)} rev=${String(r.routeReversals).padStart(2)} | ` +
        `tDef=${String(r.meanTargetDeficit).padStart(5)} vDef=${String(r.meanSpeedDeficit).padStart(5)} loss=${r.maxTimeLost}s wake=${r.meanWake}`,
      );
    }
  }
}

// Section 13 item 1: Pace labeling aggregation.
// Positive delta means rival is SLOWER than ego.
const band = (d) => d > 0.5 ? 'slower' : d < -0.5 ? 'faster' : 'equal';
console.log('\n=== SCORECARD BY RIVAL CLASS (CORRECTED PACE BANDS) ===');
console.log('class   n  attempt overlap nose  clear retained | meanTDef meanVDef contacts(r/h) routeRev throttleLifts meanLoss');
for (const b of ['slower', 'equal', 'faster']) {
  const g = rows.filter(r => band(r.delta) === b);
  if (!g.length) continue;
  const f = (k) => g.filter(r => r[k]).length;
  const avg = (k) => g.reduce((a, r) => a + r[k], 0) / g.length;
  const totalRub = g.reduce((a, r) => a + r.lightTouches, 0);
  const totalHard = g.reduce((a, r) => a + r.hardCollisions, 0);
  console.log(
    b.padEnd(7) + String(g.length).padStart(2) +
    String(f('attackAttempted')).padStart(8) + String(f('overlap')).padStart(7) +
    String(f('noseAhead')).padStart(6) + String(f('fullClear')).padStart(6) +
    String(f('retained')).padStart(9) + ' | ' +
    avg('meanTargetDeficit').toFixed(2).padStart(7) + avg('meanSpeedDeficit').toFixed(2).padStart(9) +
    `  ${totalRub}r/${totalHard}h`.padStart(13) +
    String(g.reduce((a, r) => a + r.routeReversals, 0)).padStart(9) +
    String(g.reduce((a, r) => a + r.throttleLifts, 0)).padStart(13) +
    (avg('maxTimeLost').toFixed(3) + 's').padStart(10),
  );
}

console.log('\n=== SECTION 22 SIDE-BY-SIDE THROTTLE BEHAVIOUR ===');
const sbs = rows.filter(r => r.overlap);
console.log(`scenarios with overlap: ${sbs.length}`);
console.log(`  total unnecessary throttle lifts during pursuit: ${rows.reduce((a, r) => a + r.throttleLifts, 0)}`);
console.log(`  total route reversals after commitment: ${rows.reduce((a, r) => a + r.routeReversals, 0)}`);
console.log(`  total contacts: ${rows.reduce((a, r) => a + r.contacts, 0)} (${rows.reduce((a, r) => a + r.lightTouches, 0)} rub / ${rows.reduce((a, r) => a + r.hardCollisions, 0)} hard)`);
