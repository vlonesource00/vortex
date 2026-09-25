import { writeFile } from 'node:fs/promises';
import { Track } from '../src/sim/track.js';
import { Vehicle } from '../src/sim/vehicle.js';

// Measures the canonical Vehicle envelope directly. Nothing here changes
// physics; it only records what the simulated car can actually do, so the
// planner is limited by the car instead of a hand-tuned constant.
const track = new Track('harbor-ring');
const dt = 1 / 120;

function sample(car, every = 15) {
  return { v: Number(car.speed.toFixed(2)), ax: Number(car.ax.toFixed(2)), ay: Number(car.ay.toFixed(2)),
    gear: car.gear, rpm: Number(car.rpm.toFixed(0)) };
}

function braking() {
  const rows = [];
  for (const entry of [60, 50, 40, 30]) {
    const car = new Vehicle(0, 'P', '#fff', 'gt');
    car.place(track, 400, 0, entry);
    let peak = 0;
    for (let i = 0; i < dt * 6 * 120 && car.speed > 8; i++) {
      car.controls = { steer: 0, throttle: 0, brake: 1 };
      car.step(dt, track, 0);
      peak = Math.max(peak, -car.ax);
    }
    rows.push({ entry, peakDecel: Number(peak.toFixed(2)) });
  }
  return rows;
}

function cornering() {
  const rows = [];
  for (const v of [20, 30, 40, 50, 60]) {
    const car = new Vehicle(0, 'P', '#fff', 'gt');
    car.place(track, 400, 0, v);
    let peak = 0;
    for (let i = 0; i < dt * 3 * 120; i++) {
      car.controls = { steer: 0.22, throttle: 0.35, brake: 0 };
      car.step(dt, track, 0);
      peak = Math.max(peak, Math.abs(car.ay));
    }
    rows.push({ v, peakLateral: Number(peak.toFixed(2)) });
  }
  return rows;
}

function acceleration() {
  const car = new Vehicle(0, 'P', '#fff', 'gt');
  car.place(track, 400, 0, 20);
  const rows = [];
  let previous = 20;
  for (let i = 0; i < dt * 20 * 120 && car.speed < 72; i++) {
    car.controls = { steer: 0, throttle: 1, brake: 0 };
    car.step(dt, track, 0);
    if (car.speed - previous >= 2) {
      rows.push(sample(car));
      previous = car.speed;
    }
  }
  return rows;
}

const output = {
  source: 'canonical Vehicle.step, harbor-ring, GT class, fresh tyres',
  physicsHz: 120,
  braking: braking(),
  cornering: cornering(),
  acceleration: acceleration(),
};
const path = process.argv.find(value => value.startsWith('--out='))?.slice(6) ?? 'vortex-envelope.json';
await writeFile(path, JSON.stringify(output, null, 2));
console.log(JSON.stringify(output, null, 2));
