/**
 * Geometry x torque ablation.
 *
 * The two candidate fixes for the degraded-stint departure live at different
 * levels: the actuator map decides how hard the car is driven, and the free-air
 * line decides what geometry it is driven along. This measures whether they
 * interact, by running every combination over the same four-lap stint.
 *
 *   A  physical allocator + nominal Q0        (3fd9bc8 default)
 *   B  cheap allocator    + nominal Q0        (historical fast / unstable)
 *   C  physical allocator + robust trajectory
 *   D  cheap allocator    + robust trajectory
 *   E  state-adaptive trajectory + cheap allocator
 *   F  state-adaptive trajectory + physical allocator
 *
 *   node tools/ab-geometry.mjs
 */
import { Track } from '../src/sim/track.js';
import { VortexSession } from '../src/vortex-session.js';
import { VortexDriver } from '../src/ai/vortex/vortex-driver.js';
import { ActuatorAllocator } from '../src/ai/vortex/control/actuator-allocator.js';

const LAPS = 4;

function run(label, { mode, forcedAmp, enabled }) {
  const track = new Track('harbor-ring');
  const session = new VortexSession(track, { classId: 'gt' });
  session.mode = 'practice'; session.field = 1; session.laps = LAPS; session.autopilot = true;
  session.start({ freshTrack: true });
  const car = session.cars[0];
  const driver = new VortexDriver(0, session.lineFor(car), { aggression: session.aggression });
  driver.allocator = new ActuatorAllocator({ mode });
  driver.planner.generator.robustness.enabled = Boolean(enabled);
  driver.planner.generator.robustness.forcedAmp = forcedAmp;
  session.drivers[0] = driver;

  const zero = { steer: 0, throttle: 0, brake: 0 };
  const laps = [];
  const perLap = [];
  let prevLap = car.race.lap;
  let peakBeta = 0, peakYaw = 0, peakKappa = 0, peakSlipPower = 0, spun = false;
  let lapPeak = { beta: 0, yaw: 0, kappa: 0, slipPower: 0 };
  let entrySpeed = null, minSpeed = 1e9, exitSpeed = 0, inCorner = false;

  for (let step = 0; step < 120 * 60 * 12; step++) {
    session.step(1 / 120, zero);
    if (session.phase !== 'racing') continue;

    if (car.race.lap !== prevLap) {
      if (car.race.lastLap !== null) {
        laps.push(Number(car.race.lastLap.toFixed(3)));
        perLap.push({
          lap: prevLap,
          time: Number(car.race.lastLap.toFixed(3)),
          core: Number((car.wheels.reduce((a, w) => a + w.tyre.core, 0) / 4).toFixed(1)),
          surface: Number((car.wheels.reduce((a, w) => a + w.tyre.surface, 0) / 4).toFixed(1)),
          pressure: Number((car.wheels.reduce((a, w) => a + w.tyre.pressure, 0) / 4).toFixed(3)),
          wear: Number((car.wheels.reduce((a, w) => a + w.tyre.wear, 0) / 4).toFixed(5)),
          slipPower: Number(lapPeak.slipPower.toFixed(0)),
          beta: Number(lapPeak.beta.toFixed(3)),
          yaw: Number(lapPeak.yaw.toFixed(3)),
          kappa: Number(lapPeak.kappa.toFixed(3)),
        });
        lapPeak = { beta: 0, yaw: 0, kappa: 0, slipPower: 0 };
      }
      prevLap = car.race.lap;
      if (car.race.lap > LAPS) break;
    }

    const beta = Math.abs(Math.atan2(car.v, Math.max(4, car.u)));
    peakBeta = Math.max(peakBeta, beta); lapPeak.beta = Math.max(lapPeak.beta, beta);
    peakYaw = Math.max(peakYaw, Math.abs(car.yawRate)); lapPeak.yaw = Math.max(lapPeak.yaw, Math.abs(car.yawRate));
    const k = Math.max(Math.abs(car.wheels[2].tyre.kappa), Math.abs(car.wheels[3].tyre.kappa));
    peakKappa = Math.max(peakKappa, k); lapPeak.kappa = Math.max(lapPeak.kappa, k);
    const sp = (car.wheels[2].tyre.slipPower + car.wheels[3].tyre.slipPower) / 2;
    peakSlipPower = Math.max(peakSlipPower, sp); lapPeak.slipPower = Math.max(lapPeak.slipPower, sp);
    if (beta > 0.9 || Math.abs(car.yawRate) > 2.0) spun = true;

    // Final-corner telemetry: the measured tight event sits at s≈2630.
    if (car.s > 2560 && car.s < 2700) {
      if (!inCorner) { inCorner = true; entrySpeed = car.speed; minSpeed = 1e9; }
      minSpeed = Math.min(minSpeed, car.speed);
      exitSpeed = car.speed;
    } else if (inCorner && car.s > 2700) {
      inCorner = false;
    }
  }

  return {
    label, laps, spun,
    best: laps.length ? Number(Math.min(...laps).toFixed(3)) : null,
    total: Number(laps.reduce((a, b) => a + b, 0).toFixed(3)),
    valid: laps.length,
    offtrack: Number(session.player.race.offtrack.toFixed(2)),
    contacts: session.contacts,
    severe: session.collisionStats.severeContacts,
    peakBeta: Number(peakBeta.toFixed(3)), peakYaw: Number(peakYaw.toFixed(3)),
    peakKappa: Number(peakKappa.toFixed(3)), peakSlipPower: Number(peakSlipPower.toFixed(0)),
    entrySpeed: entrySpeed ? Number(entrySpeed.toFixed(2)) : null,
    minSpeed: Number(minSpeed.toFixed(2)), exitSpeed: Number(exitSpeed.toFixed(2)),
    perLap,
  };
}

const CASES = [
  { label: 'A physical+nominal', mode: 'physical', forcedAmp: 0 },
  { label: 'B cheap+nominal', mode: 'cheap', forcedAmp: 0 },
  { label: 'C physical+robust1.5', mode: 'physical', forcedAmp: 1.5 },
  { label: 'D cheap+robust1.5', mode: 'cheap', forcedAmp: 1.5 },
  { label: 'E adaptive+cheap', mode: 'cheap', forcedAmp: null, enabled: true },
  { label: 'F adaptive+physical', mode: 'physical', forcedAmp: null, enabled: true },
  { label: 'G physical+robust2.5', mode: 'physical', forcedAmp: 2.5 },
  { label: 'H cheap+robust2.5', mode: 'cheap', forcedAmp: 2.5 },
];

const results = [];
for (const c of CASES) {
  const r = run(c.label, c);
  results.push(r);
  console.log(
    r.label.padEnd(24) + r.laps.map(t => String(t).padStart(8)).join('') +
    ' total ' + String(r.total).padStart(7) + ' best ' + String(r.best).padStart(7) +
    ' valid ' + r.valid + ' off ' + String(r.offtrack).padStart(5) +
    ' spin ' + (r.spun ? 'Y' : 'n') +
    ' beta ' + String(r.peakBeta).padStart(5) + ' yaw ' + String(r.peakYaw).padStart(5) +
    ' ctc ' + r.contacts + '/' + r.severe
  );
}

console.log('\n=== LAP BOUNDARY THERMAL + FINAL-CORNER SPEED ===');
console.log('case                     lap    time  core  surf  press   wear  slipPwr   beta   yaw  kappa | entry   min   exit');
for (const r of results) {
  for (const p of r.perLap) {
    console.log(
      r.label.padEnd(24) + String(p.lap).padStart(4) + String(p.time).padStart(8) +
      String(p.core).padStart(6) + String(p.surface).padStart(6) + String(p.pressure).padStart(7) +
      String(p.wear).padStart(8) + String(p.slipPower).padStart(9) +
      String(p.beta).padStart(7) + String(p.yaw).padStart(6) + String(p.kappa).padStart(7) +
      ' | ' + String(r.entrySpeed).padStart(5) + String(r.minSpeed).padStart(6) + String(r.exitSpeed).padStart(6)
    );
  }
}
