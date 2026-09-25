import test from 'node:test';
import assert from 'node:assert/strict';
import { Track } from '../src/sim/track.js';
import { RacingLine } from '../src/sim/ai.js';
import { Vehicle } from '../src/sim/vehicle.js';
import { makeVortexObservation } from '../src/ai/vortex/observation.js';
import { VortexDriver } from '../src/ai/vortex/vortex-driver.js';
import { LapMemory } from '../src/ai/vortex/atlas/lap-memory.js';
import { EngagementGraph } from '../src/ai/vortex/interaction/engagement-graph.js';
import { CorridorOwnership } from '../src/ai/vortex/interaction/corridor-ownership.js';

const track = new Track('harbor-ring');
const line = new RacingLine(track);

test('driver exports deterministic bounded controls from a canonical observation', () => {
  const ego = new Vehicle(0, 'VORTEX', '#df482d', 'gt');
  ego.place(track, track.gridS, 0, 5);
  const obs = makeVortexObservation(ego, [ego], track, { time: 1, mode: 'practice', projections: new Map([[0, track.nearest(ego.x, ego.z)]]) });
  const a = new VortexDriver(0, line), b = new VortexDriver(0, line);
  for (let i = 0; i < 12; i++) {
    const ca = a.step(obs, 1 / 120), cb = b.step(obs, 1 / 120);
    assert.deepEqual(ca, cb);
    assert.ok(Number.isFinite(ca.steer) && ca.steer >= -1 && ca.steer <= 1);
    assert.ok(ca.throttle >= 0 && ca.throttle <= 1 && ca.brake >= 0 && ca.brake <= 1);
  }
});

test('nearby non-intersecting traffic does not trigger a brake command', () => {
  const ego = new Vehicle(0, 'VORTEX', '#df482d', 'gt');
  const rival = new Vehicle(1, 'RIVAL', '#356653', 'gt');
  const s = track.nodes.find(point => line.at(point.s).speed > 48)?.s ?? track.gridS;
  ego.place(track, s, 0, 5); rival.place(track, s + 18, 7.5, 18);
  const cars = [ego, rival], projections = new Map(cars.map(car => [car.id, track.nearest(car.x, car.z)]));
  const obs = makeVortexObservation(ego, cars, track, { time: 1, mode: 'practice', projections });
  const driver = new VortexDriver(0, line);
  const controls = driver.step(obs, 1 / 120);
  assert.equal(controls.brake, 0);
  assert.equal(driver.diagnostics.brakeReason, 'clear');
});

test('clean lap learning updates are bounded and invalid laps are rejected', () => {
  const memory = new LapMemory(track, 12);
  for (let i = 0; i < 12; i++) memory.record(i / 12 * track.length, { q: 3, speedResidual: 5, elapsed: .5 }, true);
  const first = memory.finishLap(true);
  assert.equal(first.accepted, true); assert.equal(first.cells, 12);
  assert.ok(memory.samples.every(cell => Math.abs(cell.residualQ) <= .035 && Math.abs(cell.residualSpeed) <= .06));
  for (let i = 0; i < 12; i++) memory.record(i / 12 * track.length, { q: -3, speedResidual: -5, elapsed: .5 }, true);
  assert.equal(memory.finishLap(false).accepted, false);
  assert.ok(memory.samples.every(cell => Math.abs(cell.residualQ) <= .035 && Math.abs(cell.residualSpeed) <= .06));
});

test('joint engagement graph sees overlap and corridor ownership persists through brief clearance', () => {
  const ego = new Vehicle(0, 'VORTEX', '#df482d', 'gt');
  const rival = new Vehicle(1, 'RIVAL', '#356653', 'gt');
  ego.place(track, 200, 0, 30); rival.place(track, 204, 1.8, 29);
  const e = track.nearest(ego.x, ego.z), r = track.nearest(rival.x, rival.z);
  const egoState = { id: ego.id, ...ego, s: e.s, lateral: e.lateral };
  const rivalState = { id: rival.id, ...rival, s: r.s, lateral: r.lateral, longitudinalSpeed: 29 };
  const graph = new EngagementGraph(track), edges = graph.update(egoState, [rivalState], { at: () => ({}) }, 1 / 30);
  assert.equal(edges.length, 1); assert.equal(edges[0].overlap, true);
  const owner = new CorridorOwnership(track);
  assert.equal(owner.update(egoState, [rivalState]).length, 1);
  rivalState.s = egoState.s + 8; rivalState.lateral = 6;
  for (let i = 0; i < 3; i++) assert.equal(owner.update(egoState, [rivalState]).length, 1);
  assert.equal(owner.update(egoState, [rivalState]).length, 0);
});
