import fs from 'node:fs';
import { Track } from '../src/sim/track.js';
import { VortexSession } from '../src/vortex-session.js';
import { VortexDriver as OldDriver } from '../../vortex-old/src/ai/vortex/vortex-driver.js';
import { VortexDriver as NewDriver } from '../src/ai/vortex/vortex-driver.js';
import { wrap } from '../src/sim/math.js';

function runSoloLap(DriverClass, label) {
  const track = new Track('harbor-ring');
  const session = new VortexSession(track, { classId: 'gt' });
  session.mode = 'practice';
  session.field = 1;
  session.laps = 3;
  session.autopilot = true;
  session.start({ freshTrack: true });

  const car = session.cars[0];
  const driver = new DriverClass(0, session.lineFor(car), { aggression: session.aggression });
  session.drivers[0] = driver;

  const zero = { steer: 0, throttle: 0, brake: 0 };
  let lap2Samples = [];
  let lapTimes = [];
  let prevLap = car.race.lap;
  let lapStartTime = 0;

  for (let step = 0; step < 120 * 400; step++) {
    session.step(1 / 120, zero);
    if (session.phase !== 'racing') continue;

    if (car.race.lap !== prevLap) {
      lapTimes.push({ lap: prevLap, time: car.race.lastLap });
      prevLap = car.race.lap;
      if (car.race.lap === 2) {
        lapStartTime = session.time;
      }
      if (car.race.lap > 2) break;
    }

    if (car.race.lap === 2) {
      const limiter = driver.limiter || {};
      const env = driver.envelope?.at?.(car, car.speed, car.curvature, car.lateral) || {};
      const here = driver.planner?.at?.(car.s) || {};
      lap2Samples.push({
        t: session.time - lapStartTime,
        s: car.s,
        qTarget: here.offset ?? 0,
        actualQ: car.lateral,
        curvature: here.curvature ?? car.curvature,
        profileSpeed: driver.atlas?.profileSpeed?.(car.s) ?? 0,
        planSpeed: driver.planSpeed?.(car.s) ?? 0,
        speedTarget: driver.targetSpeed ?? 0,
        actualSpeed: car.speed,
        throttle: car.controls.throttle,
        brake: car.controls.brake,
        steering: car.controls.steer,
        beta: limiter.beta ?? Math.atan2(car.v, Math.max(4, car.u)),
        yawRate: car.yawRate,
        gripUtil: limiter.gripUtil ?? (Math.abs(car.ay) / Math.max(1, env.lateral ?? 10)),
        ax: car.ax,
        ay: car.ay,
      });
    }
  }

  return { label, lapTimes, lap2Samples };
}

console.log('Running Old Driver...');
const oldResult = runSoloLap(OldDriver, 'OLD (ba18856)');
console.log('Old Lap Times:', oldResult.lapTimes);

console.log('Running New Driver...');
const newResult = runSoloLap(NewDriver, 'NEW (HEAD 1f3c329)');
console.log('New Lap Times:', newResult.lapTimes);

const trackLength = 2704.62;
const stationStep = 20;
const stationCount = Math.floor(trackLength / stationStep);

function interpolateAtS(samples, targetS) {
  let best = samples[0];
  let minDiff = Infinity;
  for (let i = 0; i < samples.length; i++) {
    const diff = Math.abs(samples[i].s - targetS);
    if (diff < minDiff) {
      minDiff = diff;
      best = samples[i];
    }
  }
  return best;
}

const table = [];
for (let i = 0; i <= stationCount; i++) {
  const s = i * stationStep;
  const o = interpolateAtS(oldResult.lap2Samples, s);
  const n = interpolateAtS(newResult.lap2Samples, s);
  const deltaT = n.t - o.t;

  table.push({
    station: s,
    deltaT,
    old: o,
    new: n,
  });
}

fs.writeFileSync('tools/ab-telemetry.json', JSON.stringify(table, null, 2));

console.log('\n--- TELEMETRY SUMMARY EVERY 100m ---');
console.log('Station |  ΔT(s)  | v_old -> v_new (km/h) | tgt_old -> tgt_new | q_old -> q_new | thr_old/new | brk_old/new | grip_old/new');
for (let i = 0; i <= stationCount; i += 5) {
  const item = table[i];
  const s = item.station;
  const o = item.old;
  const n = item.new;
  const deltaT = item.deltaT;
  const vOldKmh = (o.actualSpeed * 3.6).toFixed(1);
  const vNewKmh = (n.actualSpeed * 3.6).toFixed(1);
  const tgtOldKmh = (o.speedTarget * 3.6).toFixed(1);
  const tgtNewKmh = (n.speedTarget * 3.6).toFixed(1);
  const qOld = o.actualQ.toFixed(2);
  const qNew = n.actualQ.toFixed(2);
  const thrOld = o.throttle.toFixed(2);
  const thrNew = n.throttle.toFixed(2);
  const brkOld = o.brake.toFixed(2);
  const brkNew = n.brake.toFixed(2);
  const gripOld = o.gripUtil.toFixed(2);
  const gripNew = n.gripUtil.toFixed(2);

  console.log(
    `${String(s).padStart(5)}m | +${deltaT.toFixed(3)}s | ` +
    `${vOldKmh.padStart(5)} -> ${vNewKmh.padStart(5)} | ` +
    `${tgtOldKmh.padStart(5)} -> ${tgtNewKmh.padStart(5)} | ` +
    `${qOld.padStart(5)} -> ${qNew.padStart(5)} | ` +
    `${thrOld}/${thrNew} | ${brkOld}/${brkNew} | ${gripOld}/${gripNew}`
  );
}

// Microsector loss analysis: find every station where ΔT jumps by >0.10s
console.log('\n--- FIRST SIGNIFICANT DIVERGENCES (where ΔT jumps > 0.10s per 20m) ---');
for (let i = 1; i < table.length; i++) {
  const dDelta = table[i].deltaT - table[i - 1].deltaT;
  if (dDelta > 0.10) {
    const s = table[i].station;
    console.log(`Station s=${s}m (+${dDelta.toFixed(3)}s jump, total ΔT=${table[i].deltaT.toFixed(3)}s):`);
    console.log(`  OLD: v=${(table[i].old.actualSpeed * 3.6).toFixed(1)} km/h, tgt=${(table[i].old.speedTarget * 3.6).toFixed(1)} km/h, plan=${(table[i].old.planSpeed * 3.6).toFixed(1)} km/h, q=${table[i].old.actualQ.toFixed(2)}, brk=${table[i].old.brake.toFixed(2)}, thr=${table[i].old.throttle.toFixed(2)}`);
    console.log(`  NEW: v=${(table[i].new.actualSpeed * 3.6).toFixed(1)} km/h, tgt=${(table[i].new.speedTarget * 3.6).toFixed(1)} km/h, plan=${(table[i].new.planSpeed * 3.6).toFixed(1)} km/h, q=${table[i].new.actualQ.toFixed(2)}, brk=${table[i].new.brake.toFixed(2)}, thr=${table[i].new.throttle.toFixed(2)}`);
  }
}
