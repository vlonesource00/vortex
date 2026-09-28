import test from 'node:test';
import assert from 'node:assert/strict';
import { VehicleEnvelope } from '../src/ai/vortex/estimation/vehicle-envelope.js';
import { ActuatorAllocator } from '../src/ai/vortex/control/actuator-allocator.js';

function wheel({ load, fx, fy, utilisation, core = 85, pressure = 2.15, wear = 0 }) {
  return {
    load,
    tyre: { fx, fy, utilisation, core, surface: core, pressure, wear, kappa: 0, alpha: 0, slipPower: 0 },
  };
}

test('axle diagnostics separate rear capability from the four-tyre mean', () => {
  const env = new VehicleEnvelope();
  const ego = {
    ay: 0, lateral: 0, impact: 0, zone: 'asphalt', speed: 30, ax: 0,
    controls: { brake: 0, throttle: 0 },
    wheels: [
      wheel({ load: 3300, fx: 0, fy: 1500, utilisation: 0.40 }),
      wheel({ load: 3300, fx: 0, fy: 1500, utilisation: 0.40 }),
      wheel({ load: 3300, fx: 0, fy: 3000, utilisation: 0.90 }),
      wheel({ load: 3300, fx: 0, fy: 3000, utilisation: 0.90 }),
    ],
  };
  env.update(ego, 1 / 120);
  // Front is at 40 % of its peak, rear at 90 %. The mean cannot show that; the
  // axle split must.
  assert.ok(env.frontReserve > env.rearReserve, 'front must show more reserve than rear');
  assert.ok(env.rearReserve < 0.2, 'rear is nearly saturated');
  assert.ok(env.rearUtilisation > 0.85, 'rear combined utilisation is high');
  assert.ok(env.yawSettlementReserve < env.yawGenerationReserve,
    'rear settling reserve must be the tighter of the two');
  assert.equal(env.muScale === undefined, false);
});

test('axle diagnostics are inert when both axles match', () => {
  const env = new VehicleEnvelope();
  const w = wheel({ load: 3300, fx: 0, fy: 1500, utilisation: 0.50 });
  env.update({ ay: 0, lateral: 0, impact: 0, zone: 'asphalt', speed: 30, ax: 0,
    controls: { brake: 0, throttle: 0 }, wheels: [w, w, w, w] }, 1 / 120);
  assert.ok(Math.abs(env.frontReserve - env.rearReserve) < 1e-9);
  assert.ok(Math.abs(env.rearToFrontGrip - 1) < 1e-9);
});

test('reset clears the axle state', () => {
  const env = new VehicleEnvelope();
  env.rearReserve = 0.1;
  env.reset();
  assert.equal(env.rearReserve, 1);
  assert.equal(env.yawSettlementReserve, 1);
});

test('rear cap is reported through allocate and leaves controls alone when inert', () => {
  const alloc = new ActuatorAllocator({ mode: 'physical' });
  const spec = { radius: 0.33, steeringLock: 0.55, wheelbase: 2.9, halfWidth: 0.99 };
  // Healthy rear: plenty of longitudinal authority, so no reduction.
  const ego = {
    speed: 40, spec,
    wheels: [
      wheel({ load: 3300, fx: 0, fy: 500, utilisation: 0.20 }),
      wheel({ load: 3300, fx: 0, fy: 500, utilisation: 0.20 }),
      wheel({ load: 3300, fx: 0, fy: 500, utilisation: 0.20 }),
      wheel({ load: 3300, fx: 0, fy: 500, utilisation: 0.20 }),
    ],
  };
  const envelope = { lateral: 25, brake: 20, drive: 8, drag: 0.5, driveTorque: 3000 };
  const r = alloc.allocate(3, ego, envelope, 0.01);
  assert.equal(typeof r.capActive, 'boolean');
  assert.equal(r.capActive, false, 'healthy rear must not be capped');
  assert.ok(r.throttle > 0 && r.throttle <= 1);
});

test('rear cap engages by default and reports it', () => {
  const alloc = new ActuatorAllocator({ mode: 'physical' });
  const spec = { radius: 0.33, steeringLock: 0.55, wheelbase: 2.9, halfWidth: 0.99 };
  // Rear heavily laterally loaded - the promoted law must intervene here.
  const ego = {
    speed: 40, spec,
    wheels: [
      wheel({ load: 3300, fx: 0, fy: 500, utilisation: 0.20 }),
      wheel({ load: 3300, fx: 0, fy: 500, utilisation: 0.20 }),
      wheel({ load: 3300, fx: 0, fy: 3000, utilisation: 0.95 }),
      wheel({ load: 3300, fx: 0, fy: 3000, utilisation: 0.95 }),
    ],
  };
  const envelope = { lateral: 25, brake: 20, drive: 8, drag: 0.5, driveTorque: 3000 };
  const r = alloc.allocate(3, ego, envelope, 0.01);
  assert.equal(r.capActive, true, 'the promoted law is active by default');
  assert.ok(r.capReduce > 0, 'it must actually reduce the throttle ask');
});
