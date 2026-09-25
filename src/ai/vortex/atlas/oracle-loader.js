import { wrap } from '../../../sim/math.js';

/**
 * Loads the offline oracle record into a TrackAtlas.
 *
 * The record is produced by tools/build-vortex-oracle.mjs and is pure data: a
 * lateral line, a speed profile and a curvature field on a uniform station
 * grid. Installing it grants no authority over physics -- geometry, legality
 * and the simulated car remain the source of truth -- it only tells the runtime
 * the fastest trajectory the canonical car can actually achieve.
 *
 * Both the compact grid form ({n, spacing, q[], v[], kappa[]}) and the
 * per-station form ({stations: [{s, speed, q, ...}]}) are accepted so an older
 * or hand-edited record still installs.
 */
export function installOracle(atlas, payload) {
  if (!payload) return false;

  if (Number.isFinite(payload.n) && Array.isArray(payload.q) && Array.isArray(payload.v)) {
    return atlas.installOracle(payload);
  }

  if (payload.trackId && payload.trackId !== atlas.track.id) return false;
  const stations = (payload.stations ?? [])
    .map(point => ({
      s: wrap(Number(point.s), atlas.track.length),
      speed: Number(point.speed),
      q: Number(point.q ?? 0),
      kappa: Number(point.kappa ?? point.curvature ?? 0),
    }))
    .filter(point => Number.isFinite(point.s) && Number.isFinite(point.speed))
    .sort((a, b) => a.s - b.s);
  if (stations.length < 8) return false;

  const n = stations.length;
  return atlas.installOracle({
    n,
    spacing: atlas.track.length / n,
    lapTime: Number(payload.lapTime) || 0,
    legalOffset: Number(payload.legalOffset) || undefined,
    q: stations.map(point => point.q),
    v: stations.map(point => point.speed),
    kappa: stations.map(point => point.kappa),
  });
}

export default installOracle;
