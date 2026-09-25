import { wrap } from '../../sim/math.js';

function view(car, track) {
  const p = track.nearest(car.x, car.z);
  return {
    id: car.id, name: car.name, classId: car.classId, x: car.x, z: car.z,
    yaw: car.yaw, vx: car.vx, vz: car.vz, u: car.u, v: car.v,
    speed: car.speed, yawRate: car.yawRate, steering: car.steering,
    ax: car.ax, ay: car.ay, pitch: car.pitch, heave: car.heave, s: p.s, lateral: p.lateral, heading: p.heading,
    tx: p.tx, tz: p.tz, nx: p.nx, nz: p.nz, curvature: p.curvature,
    zone: car.zone, impact: car.impact, damage: car.damage, fuel: car.fuel,
    trackWetness: track.wetness, trackHalfWidth: track.halfWidth,
    controls: { ...car.controls },
    aero: { ...car.aero }, setup: { ...car.setup }, spec: car.spec,
    wheels: car.wheels.map(w => ({
      load: w.load, steer: w.steer, omega: w.omega, brakeTemp: w.brakeTemp,
      tyre: { ...w.tyre },
    })),
    race: car.race ? { lap: car.race.lap, progress: car.race.progress, valid: car.race.valid,
      offtrack: car.race.offtrack, lastLap: car.race.lastLap, bestLap: car.race.bestLap } : null,
  };
}

/** Pure, canonical snapshot shared by the standalone host and benchmark bridge. */
export function makeVortexObservation(car, cars, track, context = null) {
  const ego = view(car, track);
  const rivals = cars.filter(other => other.id !== car.id).map(other => view(other, track));
  const projections = context?.projections;
  return {
    ego, rivals, track,
    time: Number(context?.time ?? 0), mode: context?.mode ?? 'race',
    totalLaps: Number(context?.totalLaps ?? 0), paceObjective: context?.paceObjective ?? 'race',
    order: context?.order?.map(item => item.id) ?? [car, ...cars.filter(item => item !== car)].map(item => item.id),
    projection: projections?.get(car.id) ?? { s: ego.s, lateral: ego.lateral, heading: ego.heading,
      tx: ego.tx, tz: ego.tz, nx: ego.nx, nz: ego.nz, curvature: ego.curvature },
    contactCount: Number(context?.contactCount ?? 0),
  };
}

export function relativeStation(a, b, length) {
  return wrap(b.s - a.s + length / 2, length) - length / 2;
}
