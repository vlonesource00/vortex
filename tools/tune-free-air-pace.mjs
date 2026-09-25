/**
 * Free-air pace tuner.
 *
 * The offline oracle is quasi-steady-state and therefore optimistic: a plan
 * solved at full grip cannot be driven, and one solved too conservatively
 * leaves measured pace on the table. The only honest way to choose the plan's
 * grip margin is to solve an oracle at each candidate level and then run it in
 * the frozen 120 Hz plant and read the lap time.
 *
 *   node tools/tune-free-air-pace.mjs
 *   node tools/tune-free-air-pace.mjs --grips 0.86,0.88,0.9 --laps 4 --write
 */
import { writeFileSync, mkdirSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

import { Track } from '../src/sim/track.js';
import { RacingLine } from '../src/sim/ai.js';
import { SPEC } from '../src/sim/vehicle.js';
import { EnvelopeModel, referenceTyre } from '../src/ai/vortex/atlas/envelope.js';
import { TrackOracle } from '../src/ai/vortex/atlas/oracle.js';
import { VortexSession } from '../src/vortex-session.js';

const here = dirname(fileURLToPath(import.meta.url));
const argv = process.argv.slice(2);
const flag = name => argv.includes(name);
const number = (name, fallback) => {
  const i = argv.indexOf(name);
  return i >= 0 && Number.isFinite(Number(argv[i + 1])) ? Number(argv[i + 1]) : fallback;
};
const list = (name, fallback) => {
  const i = argv.indexOf(name);
  return i >= 0 ? argv[i + 1].split(',').map(Number) : fallback;
};

const track = new Track('harbor-ring');
const line = new RacingLine(track, SPEC);
const laps = Math.max(1, Math.min(8, number('--laps', 3)));
const grips = list('--grips', [0.80, 0.84, 0.88, 0.92, 0.96, 1.0]);
const qMax = number('--qmax', track.halfWidth - 0.99 - 0.12);

function solveOracle(gripScale) {
  const envelope = new EnvelopeModel({
    classId: 'gt',
    fuel: 35,
    wing: 6,
    gripScale,
    tyre: referenceTyre({ core: number('--core', 85) }),
  });
  const oracle = new TrackOracle(track, {
    envelope,
    spacing: number('--spacing', 2.2),
    qMax,
    qSlopeMax: number('--q-slope', Infinity),
  });
  const initial = new Float64Array(oracle.n);
  for (let i = 0; i < oracle.n; i++) initial[i] = line.offsetAt(oracle.s[i]);
  const result = oracle.optimize({
    initial,
    iterations: number('--iterations', 80),
    step: number('--step', 1.4),
    smoothing: number('--smoothing', 12),
  });
  return {
    prediction: result.time,
    record: TrackOracle.toJSON(result, {
      track: 'harbor-ring',
      trackLength: track.length,
      legalOffset: oracle.qMax,
      gripScale: envelope.gripScale,
      tyreCore: envelope.tyre.core,
    }),
  };
}

function measure(record) {
  const session = new VortexSession(track, { classId: 'gt', vortex: { oracle: record } });
  session.mode = 'practice';
  session.field = 1;
  session.laps = laps;
  session.autopilot = true;
  session.start({ freshTrack: true });

  const zero = { steer: 0, throttle: 0, brake: 0 };
  let lastLap = session.player.race.lap;
  let lapValid = true;
  let steps = 0;
  let priorOfftrack = 0;
  const recorded = [];
  while (steps < 120 * 60 * 6 && session.phase !== 'finished') {
    const priorLap = session.player.race.lap;
    const priorValid = session.player.race.valid;
    session.step(1 / 120, zero);
    steps++;
    if (session.player.race.lap !== lastLap) {
      lastLap = session.player.race.lap;
      if (session.player.race.lastLap !== null) {
        const cumulative = session.player.race.offtrack;
        recorded.push({
          time: session.player.race.lastLap,
          valid: lapValid && priorValid,
          offtrack: cumulative - priorOfftrack,
        });
        priorOfftrack = cumulative;
      }
      lapValid = session.player.race.valid;
      if (recorded.filter(item => item.valid).length >= laps) break;
    } else if (!session.player.race.valid) {
      lapValid = false;
    }
  }
  const valid = recorded.filter(item => item.valid).map(item => item.time).sort((a, b) => a - b);
  return {
    best: valid.length ? valid[0] : null,
    median: valid.length ? valid[Math.floor(valid.length / 2)] : null,
    validCount: valid.length,
    attempts: recorded.length,
    offtrack: session.player.race.offtrack,
    contacts: session.contacts,
    severe: session.collisionStats.severeContacts,
    // Per-lap detail: the cumulative off-track figure cannot tell a clean
    // flying lap from a lap that lost it on the third tour.
    perLap: recorded.map((item, index) => ({
      lap: index + 1,
      time: Number(item.time.toFixed(3)),
      valid: item.valid,
      offtrack: Number(item.offtrack.toFixed(2)),
    })),
  };
}

const rows = [];
for (const gripScale of grips) {
  const started = Date.now();
  const { prediction, record } = solveOracle(gripScale);
  const measured = measure(record);
  const clean = measured.severe === 0 && measured.offtrack < 0.25 && measured.validCount >= Math.min(2, laps);
  rows.push({
    gripScale,
    prediction: Number(prediction.toFixed(3)),
    best: measured.best === null ? null : Number(measured.best.toFixed(3)),
    median: measured.median === null ? null : Number(measured.median.toFixed(3)),
    validLaps: `${measured.validCount}/${measured.attempts}`,
    offtrack: Number(measured.offtrack.toFixed(2)),
    contacts: measured.contacts,
    severe: measured.severe,
    clean,
    overPrediction: measured.best === null ? null : Number((measured.best - prediction).toFixed(3)),
    perLap: measured.perLap,
    wallSeconds: Number(((Date.now() - started) / 1000).toFixed(1)),
    record,
  });
}

const cleanRows = rows.filter(row => row.clean && row.best !== null);
const best = cleanRows.length
  ? cleanRows.reduce((fastest, row) => (row.best < fastest.best ? row : fastest))
  : null;

console.log(JSON.stringify({
  track: 'harbor-ring',
  laps,
  qMax: Number(qMax.toFixed(3)),
  table: rows.map(({ record: _record, ...row }) => row),
  recommendation: best
    ? { gripScale: best.gripScale, best: best.best, prediction: best.prediction, overPrediction: best.overPrediction }
    : null,
}, null, 2));

if (flag('--write') && best) {
  const jsonPath = resolve(here, '../src/ai/vortex/atlas/oracle-data.json');
  const jsPath = resolve(here, '../src/ai/vortex/atlas/oracle-data.js');
  mkdirSync(dirname(jsonPath), { recursive: true });
  writeFileSync(jsonPath, JSON.stringify(best.record) + '\n');
  writeFileSync(jsPath, `// Generated by tools/tune-free-air-pace.mjs. Do not edit by hand.\nexport default ${JSON.stringify(best.record)};\n`);
  console.error(`wrote gripScale ${best.gripScale} -> ${jsonPath}`);
}

if (!best) {
  console.error('no clean grip level found');
  process.exitCode = 1;
}
