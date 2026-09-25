/**
 * Ground-truth probe for the VORTEX envelope model.
 *
 * Drives the frozen canonical Vehicle through three physical experiments on the
 * Harbor Ring main straight and compares the measured accelerations with the
 * analytic EnvelopeModel. Nothing in src/sim is modified; the probe only
 * issues ordinary {steer, throttle, brake} commands.
 *
 *   node tools/calibrate-envelope.mjs
 */
import { Track } from '../src/sim/track.js';
import { Vehicle } from '../src/sim/vehicle.js';
import { angle, clamp } from '../src/sim/math.js';
import { EnvelopeModel, referenceTyre } from '../src/ai/vortex/atlas/envelope.js';

const track = new Track('harbor-ring');
const dt = 1 / 120;
const STRAIGHT_S = 320;

// The main straight runs from the start line through the first sector.
{
  const probe = [];
  for (let s = 0; s < track.length; s += 25) probe.push({ s, k: Math.abs(track.at(s).curvature) });
  probe.sort((a, b) => a.k - b.k);
  if (probe[0].s < 100 || probe[0].s > 700) throw new Error(`Unexpected straightest station ${probe[0].s}`);
}

function fresh(speed, core = 85) {
  const car = new Vehicle(0, 'PROBE', '#fff', 'gt');
  car.place(track, STRAIGHT_S, 0, speed);
  for (const w of car.wheels) {
    w.tyre.core = core; w.tyre.surface = core;
    w.tyre.pressure = (w.tyre.coldPressure + 1.01325) * ((core + 273.15) / (24 + 273.15)) - 1.01325;
    w.omega = speed / car.spec.radius;
  }
  car.u = speed; car.speed = speed;
  return car;
}

/** Keep the car on the centreline of the straight without adding lateral demand. */
function laneHold(car) {
  const p = track.nearest(car.x, car.z);
  const lookahead = clamp(6 + car.speed * 0.4, 8, 30);
  const target = track.at(p.s + lookahead);
  const dx = target.x - car.x, dz = target.z - car.z;
  const localX = dx * Math.cos(car.yaw) - dz * Math.sin(car.yaw);
  const pursuit = Math.atan2(2 * car.spec.wheelbase * localX, Math.max(20, dx * dx + dz * dz));
  const slip = Math.atan2(car.v, Math.max(4, car.u));
  return clamp((pursuit + slip * 0.85) / car.spec.steeringLock, -0.4, 0.4);
}

function holdSpeed(car, target) {
  const error = target - car.speed;
  car.controls.throttle = error > 0 ? Math.min(1, error * 0.6 + 0.35) : 0;
  car.controls.brake = error < 0 ? Math.min(1, -error * 0.5) : 0;
  if (car.controls.brake > 0.015) car.controls.throttle = 0;
}

function onAsphalt(car) {
  return Math.abs(car.lateral) < track.halfWidth - 0.2 && car.zone === 'asphalt';
}

// --- experiment 1: pure braking from 78 m/s ---------------------------------
const brakePoints = [];
{
  const car = fresh(78, 85);
  for (let i = 0; i < 120 * 12; i++) {
    car.controls = { steer: laneHold(car), throttle: 0, brake: 1 };
    car.step(dt, track, 0);
    if (onAsphalt(car) && Math.abs(car.ax) > 2 && car.speed > 4 && car.speed < 77) {
      brakePoints.push({ v: car.speed, a: -car.ax });
    }
    if (car.speed < 3) break;
  }
}

// --- experiment 2: full-throttle acceleration --------------------------------
const drivePoints = [];
{
  const car = fresh(14, 85);
  for (let i = 0; i < 120 * 30; i++) {
    car.controls = { steer: laneHold(car), throttle: 1, brake: 0 };
    car.step(dt, track, 0);
    if (onAsphalt(car) && car.speed > 16 && car.speed < 77 && Math.abs(car.ay) < 1.2) {
      drivePoints.push({ v: car.speed, a: car.ax });
    }
    if (car.speed > 76.5) break;
  }
}

// --- experiment 3: saturated lateral at fixed speed -------------------------
// Step the steering on the straight and read the peak |ay| before the car
// drifts off the asphalt. Tyre relaxation is far shorter than the window, so
// the plateau is the plant's real lateral limit.
function lateralPeak(speed, steerCommand, core = 85) {
  const car = fresh(speed, core);
  let best = 0;
  for (let i = 0; i < 200; i++) {
    car.controls.steer = steerCommand;
    holdSpeed(car, speed);
    car.step(dt, track, 0);
    if (!onAsphalt(car)) continue;
    if (Math.abs(car.ay) < 1) continue;
    best = Math.max(best, Math.abs(car.ay));
  }
  return best;
}

function lateralLimit(speed, core = 85) {
  let best = 0;
  for (const steer of [0.1, 0.16, 0.24, 0.34, 0.46, 0.6, 0.76, 0.92, 1]) {
    best = Math.max(best, lateralPeak(speed, steer, core));
  }
  return best;
}

const lateralPoints = [12, 20, 30, 40, 50, 60, 72].map(v => ({ v, a: lateralLimit(v, 85) }));

// --- comparison against the analytic envelope -------------------------------
const model = new EnvelopeModel({ classId: 'gt', fuel: 35, wing: 6, tyre: referenceTyre({ core: 85 }) });

const bin = (points, speed, width = 2) => {
  const near = points.filter(p => Math.abs(p.v - speed) <= width);
  return near.length ? Math.max(...near.map(p => p.a)) : null;
};

const row = (v, measured, predicted) => ({
  v,
  measured: measured == null ? null : Number(measured.toFixed(3)),
  predicted: Number(predicted.toFixed(3)),
  ratio: measured == null ? null : Number((measured / predicted).toFixed(4)),
});

const report = {
  track: 'harbor-ring',
  straightS: STRAIGHT_S,
  length: Number(track.length.toFixed(3)),
  halfWidth: track.halfWidth,
  model: model.snapshot(40),
  braking: [72, 60, 50, 40, 30].map(v => row(v, bin(brakePoints, v), model.brakeAccel(v))),
  driving: [20, 30, 40, 50, 60, 70].map(v => row(v, bin(drivePoints, v), model.driveAccel(v))),
  lateral: lateralPoints.map(p => row(p.v, p.a, model.lateralAccel(p.v))),
  samples: { brake: brakePoints.length, drive: drivePoints.length },
};

console.log(JSON.stringify(report, null, 2));
