import { clamp, wrap } from '../../../sim/math.js';
import { classifyConflict, restrictsSpeed, INTERACTION_MARGIN } from '../interaction/clearance.js';

// How aggressively a demanded lateral move is charged against the grip budget.
// Tuned against the s=1200-1350 departure: the plan was promising a ~4 m
// excursion through a bend already at 70-80% grip utilisation.
const LATERAL_RATE_COST = 1.5;

/**
 * Turns a corridor candidate into a physically feasible speed plan.
 *
 * The plan speed (`speedLimit`) is the speed the car may actually hold at each
 * station: seeded from the optimised free-air profile, capped by this
 * candidate's own driven-path curvature, reduced by any predicted interaction,
 * then made braking-feasible by a backward sweep. The forward sweep produces
 * `speed` and `time` for scoring only -- they are deliberately not used as
 * control targets, because a forward-reachable speed profile makes a controller
 * brake to corner speed from its lookahead distance out.
 */
export class TrajectoryRefiner {
  constructor(track, envelope, atlas = null) {
    this.track = track;
    this.envelope = envelope;
    this.atlas = atlas;
    this.brakeSteps = 2;
  }

  refine(path, ego, occupancy, rivals) {
    const points = path.points, n = points.length;
    if (!n) return path;

    // Driven-path curvature of this candidate, not the centreline's.
    for (let i = 1; i < n - 1; i++) {
      const a = points[i - 1], b = points[i], c = points[i + 1];
      const h0 = Math.atan2(b.x - a.x, b.z - a.z), h1 = Math.atan2(c.x - b.x, c.z - b.z);
      const ds = Math.max(0.5, (c.distance - a.distance) * 0.5);
      b.curvature = Math.atan2(Math.sin(h1 - h0), Math.cos(h1 - h0)) / ds;
    }
    if (n > 1) { points[0].curvature = points[1].curvature; points[n - 1].curvature = points[n - 2].curvature; }

    // Free-air anchor: the optimised profile is the plan whenever this
    // candidate follows the optimised geometry. Displaced corridors are capped
    // by their own curvature so a wide line never promises impossible speed.
    let speed = Math.max(6, ego.speed);
    for (let i = 0; i < n; i++) {
      const p = points[i];
      const anchor = this.atlas ? this.atlas.profileSpeed(p.s) : Infinity;
      const env = this.envelope.at(ego, speed, p.curvature, p.offset);
      const geometry = env.speedLimit;
      p.lateralLimit = env.lateral;
      // Reachability. Moving the car sideways costs grip that is then not
      // available for cornering: over a transition of length L covering dq the
      // extra demand is of order v^2 * dq / L^2. A plan that ignores this asks
      // for a multi-metre excursion through a fast bend, the car tracks a few
      // metres off it, and once the tyres are a few percent down it runs out of
      // road entirely. The horizon is the distance the car covers in roughly
      // the servo's own response time.
      const j = Math.min(n - 1, i + 6);
      const L = Math.max(10, points[j].distance - p.distance);
      const dq = points[j].offset - p.offset;
      const lateralRate = LATERAL_RATE_COST * Math.abs(dq) / (L * L);
      const reachable = Math.sqrt(Math.max(1, env.lateral / Math.max(1e-5, Math.abs(p.curvature) + lateralRate)));
      p.speedLimit = clamp(Math.min(anchor, geometry, reachable), 5, 82);
      // Free-air limit before any traffic cap. The scorer needs this to charge
      // a candidate for the time a rival is actually costing it.
      p.freeSpeedLimit = p.speedLimit;
      p.demand = 0;
    }

    // Interaction limits only where station, time and lane genuinely intersect
    // in SWEPT BODY GEOMETRY, and only when the conflict state actually
    // warrants a speed restriction. Previously this was a binary lateral-gap
    // test that treated any side-by-side as traffic, which forced the generated
    // attack corridor to inherit the rival's speed. See interaction/clearance.js.
    let interaction = false;
    const interactionOrigins = [];
    for (let i = 0; i < n; i++) {
      const p = points[i];
      const prior = points[i - 1];
      const horizon = (points[i].distance ?? 0) / Math.max(8, ego.speed);
      const dt = 1 / 27;
      for (const rival of rivals) {
        const predicted = occupancy.at(rival, horizon);
        const future = occupancy.at(rival, horizon + dt);
        const ds = wrap(predicted.s - p.s + this.track.length / 2, this.track.length) - this.track.length / 2;
        // Signed body clearances. >0 means the bodies are separated.
        const latGap = Math.abs(predicted.lateral - p.offset)
          - (ego.spec.halfWidth + predicted.halfWidth);
        const alongGap = Math.abs(ds) - (ego.spec.halfLength + predicted.halfLength);
        // Closing rates: true relative speed along track, and the lateral gap
        // SHRINKING RATE. The latter must not use the candidate's raw lateral
        // velocity: a corridor that swings away from the rival is escaping, not
        // crossing. Measuring raw motion classified an escaping flank as
        // CROSSING_CONFLICT and cut the target speed by 34 m/s at the ego's own
        // station for a rival 18 m ahead. See tools/fixture-combat.mjs.
        const relSpeedLong = Math.abs(ego.speed - predicted.speed);
        const gapNow = Math.abs(predicted.lateral - p.offset);
        const pOffsetAhead = prior ? p.offset + (p.offset - prior.offset) : p.offset;
        const gapFuture = Math.abs(future.lateral - pOffsetAhead);
        const relSpeedLat = Math.max(0, (gapNow - gapFuture) / dt);

        const state = classifyConflict(latGap, alongGap, relSpeedLong, relSpeedLat, ds > -1);
        if (!restrictsSpeed(state)) continue;

        // Only restrict where the candidate and rival actually occupy
        // overlapping LONGITUDINAL body space -- the station where they would
        // meet. Everything upstream of that is the backward sweep's job, and it
        // is the physically correct way to arrive at that speed.
        //
        // Previously the cap was applied at every point out to the full horizon
        // with a shallow (ds-4)*0.32 ramp. Measured consequence: a rival 21 m
        // ahead cut the target speed at the ego's own station from 55.12 to
        // 26.68 m/s before the car had done anything wrong. That is the
        // reported "braking before the actual fight".
        if (alongGap > 0) continue;

        // Match the rival's pace, do not trail below it.
        const capped = Math.max(0, predicted.speed);
        if (capped >= p.speedLimit) continue;
        if (!interaction) interactionOrigins.push({
          point: i,
          s: Number(p.s.toFixed(2)),
          distanceAhead: Number((p.distance ?? 0).toFixed(2)),
          timeAhead: Number(horizon.toFixed(2)),
          rivalId: rival.id ?? 'R',
          rivalS: Number(predicted.s.toFixed(2)),
          rivalQ: Number(predicted.lateral.toFixed(2)),
          candidateS: Number(p.s.toFixed(2)),
          candidateQ: Number(p.offset.toFixed(2)),
          longitudinalBodyGap: Number(alongGap.toFixed(2)),
          lateralBodyGap: Number(latGap.toFixed(2)),
          classification: state,
          freeSpeedLimit: Number((p.freeSpeedLimit ?? p.speedLimit).toFixed(2)),
          speedLimitBefore: Number(p.speedLimit.toFixed(2)),
          speedLimitAfter: Number(capped.toFixed(2)),
        });
        p.speedLimit = Math.min(p.speedLimit, capped);
        interaction = true;
      }
    }
    path.interactionOrigins = interactionOrigins;

    // The free-air profile is already braking-feasible, so the backward sweep
    // only runs when an interaction limit has to be propagated back up the
    // braking zone. Braking authority here is the full straight-line capacity:
    // the car finishes braking before the corner, and using the corner's
    // reduced friction reserve would force apex speed hundreds of metres early.
    // §8 Backward propagation instrumentation. A future traffic cap must not
    // silently reach 150 m back toward the current station and present itself
    // as braking now. Snapshot before, compare after, report the reach.
    let backwardPropagation = null;
    if (interaction) {
      const before = points.map(p => p.speedLimit);
      for (let pass = 0; pass < this.brakeSteps; pass++) {
        for (let i = n - 2; i >= 0; i--) {
          const a = points[i], b = points[i + 1];
          const ds = Math.max(0.1, b.distance - a.distance);
          const env = this.envelope.at(ego, b.speedLimit, b.curvature, b.offset);
          a.speedLimit = Math.min(a.speedLimit, Math.sqrt(b.speedLimit * b.speedLimit + 2 * env.brake * ds));
        }
      }
      let earliest = -1, latest = -1, maxDrop = 0;
      for (let i = 0; i < n; i++) {
        const drop = before[i] - points[i].speedLimit;
        if (drop > 0.05) {
          if (earliest < 0) earliest = i;
          latest = i;
          maxDrop = Math.max(maxDrop, drop);
        }
      }
      const origin = interactionOrigins[0];
      backwardPropagation = {
        triggered: true,
        originStation: origin?.s ?? null,
        originCappedSpeed: origin?.speedLimitAfter ?? null,
        earliestModifiedStation: earliest >= 0 ? Number(points[earliest].s.toFixed(2)) : null,
        distancePropagatedBackward: earliest >= 0 && origin
          ? Number((origin.s - points[earliest].s).toFixed(2)) : 0,
        targetSpeedBeforeAtEgo: Number((before[0] ?? 0).toFixed(2)),
        targetSpeedAfterAtEgo: Number((points[0].speedLimit ?? 0).toFixed(2)),
        totalDeltaVAtEgo: Number(((before[0] ?? 0) - (points[0].speedLimit ?? 0)).toFixed(2)),
        maxLocalDrop: Number(maxDrop.toFixed(2)),
        modifiedPoints: earliest >= 0 ? (latest - earliest + 1) : 0,
      };
    }
    path.backwardPropagation = backwardPropagation;

    // Forward sweep: reachable speed and elapsed time, for scoring only.
    for (let i = 0; i < n; i++) {
      const p = points[i];
      const ds = i === 0 ? 0 : Math.max(0, p.distance - points[i - 1].distance);
      const env = this.envelope.at(ego, speed, p.curvature, p.offset);
      const acceleration = env.drive * env.reserve;
      speed = Math.min(p.speedLimit, Math.sqrt(speed * speed + 2 * acceleration * ds));
      speed = Math.max(speed, Math.sqrt(Math.max(1, speed * speed - 2 * env.brake * ds)));
      p.speed = speed;
      p.time = i === 0 ? 0 : points[i - 1].time + ds / Math.max(3, (speed + points[i - 1].speed) * 0.5);
      p.demand = clamp(speed * speed * Math.abs(p.curvature) / Math.max(1, p.lateralLimit), 0, 1.4);
    }

    path.exitSpeed = points.at(-1)?.speedLimit ?? ego.speed;
    path.planEntrySpeed = points[0]?.speedLimit ?? ego.speed;
    return path;
  }
}
