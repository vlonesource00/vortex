import { writeFile } from 'node:fs/promises';
import { VortexSession } from '../src/vortex-session.js';
import { Track } from '../src/sim/track.js';
import { RacingLine } from '../src/sim/ai.js';
import { TrackAtlas } from '../src/ai/vortex/atlas/track-atlas.js';
import { PHYSICAL } from '../src/ai/vortex/atlas/physical-profile.js';

// Sweeps the free-air pace margin against the canonical simulator and keeps
// only configurations that stay legal. Off-track time and safety interventions
// are hard failures here: they are never a signal to push the margin further.
const arg = (name, fallback) => { const i = process.argv.indexOf(name); return i >= 0 ? Number(process.argv[i + 1]) : fallback; };
const laps = Math.max(2, Math.min(6, arg('--laps', 3)));
const grid = (process.argv.find(v => v.startsWith('--grid='))?.slice(7) ?? '1.30,1.24,1.18,1.12,1.06,1.00')
  .split(',').map(Number).filter(Number.isFinite);

function measure(physical, laps) {
  const session = new VortexSession(new Track('harbor-ring'), { classId: 'gt', physical });
  session.mode = 'practice'; session.field = 1; session.laps = laps; session.autopilot = true;
  session.start({ freshTrack: true });
  const zero = { steer: 0, throttle: 0, brake: 0 };
  let lastLap = session.player.race.lap, lapValid = true;
  const recorded = [];
  let steps = 0;
  while (steps < 120 * 60 * 6 && session.phase !== 'finished') {
    const priorValid = session.player.race.valid;
    session.step(1 / 120, zero); steps++;
    if (session.player.race.lap !== lastLap) {
      lastLap = session.player.race.lap;
      recorded.push({ time: session.player.race.lastLap, valid: lapValid && priorValid });
      lapValid = session.player.race.valid;
    } else if (!session.player.race.valid) lapValid = false;
    if (recorded.filter(l => l.valid).length >= laps) break;
  }
  const valid = recorded.filter(l => l.valid && Number.isFinite(l.time)).map(l => l.time).sort((a, b) => a - b);
  const driver = session.drivers[0];
  return {
    valid: valid.map(v => Number(v.toFixed(3))),
    best: valid.length ? Number(valid[0].toFixed(3)) : null,
    median: valid.length ? Number(valid[Math.floor(valid.length / 2)].toFixed(3)) : null,
    offtrackSeconds: Number(session.player.race.offtrack.toFixed(2)),
    safetyInterventions: driver.telemetry.safetyInterventions,
    atlasLapTime: Number(driver.atlas.lapTime.toFixed(3)),
  };
}

const rows = [];
for (const muLat of grid) {
  const physical = { ...PHYSICAL, muLat };
  const result = measure(physical, laps);
  const legal = result.offtrackSeconds === 0 && result.safetyInterventions === 0 && result.valid.length >= laps;
  rows.push({ muLat, ...result, legal });
  console.log(JSON.stringify({ muLat, ...result, legal }));
}

const legal = rows.filter(row => row.legal && row.best !== null).sort((a, b) => a.best - b.best);
const chosen = legal[0] ?? rows.filter(row => row.best !== null).sort((a, b) => a.offtrackSeconds - b.best)[0];
const track = new Track('harbor-ring'), line = new RacingLine(track);
const atlas = new TrackAtlas(line, null, { physical: { ...PHYSICAL, muLat: chosen.muLat } });
const output = {
  trackId: track.id, schema: 2, source: 'closed-loop physical profile on the canonical atlas geometry',
  physicsHz: 120, trackLength: track.length, chosen, sweep: rows,
  stations: atlas.oracle.map(point => ({ s: Number(point.s.toFixed(3)), q: Number(point.q.toFixed(4)),
    speed: Number(point.speed.toFixed(3)), curvature: Number(point.curvature.toFixed(6)) })),
};
const path = process.argv.find(v => v.startsWith('--out='))?.slice(6) ?? 'vortex-oracle.json';
await writeFile(path, JSON.stringify(output, null, 2));
console.log(JSON.stringify({ path, chosen, atlasLapTime: output.chosen.atlasLapTime }, null, 2));
