import { angle, clamp, wrap } from '../../sim/math.js';
import { makeVortexObservation } from './observation.js';
import { TrackAtlas } from './atlas/track-atlas.js';
import { installOracle } from './atlas/oracle-loader.js';
import { LapMemory } from './atlas/lap-memory.js';
import { LapResidualLearner } from './atlas/residual-learner.js';
import { VehicleEnvelope } from './estimation/vehicle-envelope.js';
import { OpponentFilter } from './estimation/opponent-filter.js';
import { OccupancyPredictor } from './estimation/occupancy-predictor.js';
import { EngagementGraph } from './interaction/engagement-graph.js';
import { CorridorOwnership } from './interaction/corridor-ownership.js';
import { MulticornerPlanner } from './planning/multicorner-planner.js';
import { VehicleServo } from './control/servo.js';
import { TrajectoryOptimizer } from './control/trajectory-optimizer.js';
import { ActuatorAllocator } from './control/actuator-allocator.js';
import { SafetyKernel } from './safety/safety-kernel.js';
import { VortexTelemetry } from './telemetry/vortex-telemetry.js';

const SERVO_HZ = 120, TRANSIENT_HZ = 60, WORLD_HZ = 30, PLAN_HZ = 27;

/**
 * Deterministic Harbor driver. A single candidate game serves clear air,
 * passing, defence, overlap and three-wide situations; labels are telemetry.
 */
export class VortexDriver {
  constructor(id, line, options = {}) {
    this.id = id; this.line = line; this.track = line.track; this.skill = options.skill ?? 1;
    this.strategy = { aggression: clamp(options.aggression ?? .78, 0, 1), pressure: 0, reason: 'space-time value' };
    this.options = { ...options };
    this.initialize();
  }
  initialize() {
    this.memory = new LapMemory(this.track, this.options.microsectors ?? 120);
    this.atlas = new TrackAtlas(this.line, this.memory, this.options);
    if (this.options.oracle) installOracle(this.atlas, this.options.oracle);
    // One physical envelope, shared by the atlas and the online estimator, so
    // the plan and the controller can never disagree about what the car can do.
    this.envelope = new VehicleEnvelope({ ...this.options, model: this.atlas.envelope });
    this.opponents = new OpponentFilter(this.track);
    this.occupancy = new OccupancyPredictor(this.track);
    this.graphBuilder = new EngagementGraph(this.track);
    this.ownership = new CorridorOwnership(this.track);
    this.planner = new MulticornerPlanner(this.atlas, this.track, this.envelope, this.ownership);
    this.servo = new VehicleServo();
    this.optimizer = new TrajectoryOptimizer(this.track, this.envelope);
    this.allocator = new ActuatorAllocator();
    this.safety = new SafetyKernel(this.track);
    this.learner = new LapResidualLearner(this.memory, this.atlas);
    this.telemetry = new VortexTelemetry();
    this.worldClock = 0; this.planClock = 0; this.optimizerClock = 0; this.envelopeClock = 0;
    this.lastGraph = []; this.lastOpponents = []; this.plan = null; this.selectedTrajectory = null;
    this.targetSpeed = 0; this.steer = 0; this.state = 'FREE AIR'; this.recovery = 0;
    this.speedIntegral = 0;
    this.wasRecovering = false; this.reverseTimer = 0; this.wasDefending = false;
    this.recoveryOutside = false; this.recovery = 0;
    this.passState = new Map(); this.debug = { candidates: [], selected: 0, rollouts: [] };
    this.lastSolve = null; this.aim = null; this.totalTime = 0; this.stepCount = 0; this.limiter = null;
  }
  reset() { this.initialize(); }
  update(car, cars, dt, context = null) {
    const observation = makeVortexObservation(car, cars, this.track, context);
    const controls = this.step(observation, dt);
    car.controls = controls;
    return controls;
  }
  step(observation, dt) {
    dt = clamp(dt, 1 / 1000, .1);
    const ego = observation.ego, projection = observation.projection;
    // Preserve the canonical projection for shared benchmark observations.
    ego.s = projection.s; ego.lateral = projection.lateral; ego.heading = projection.heading;
    ego.tx = projection.tx; ego.tz = projection.tz; ego.nx = projection.nx; ego.nz = projection.nz;
    ego.curvature = projection.curvature;
    this.totalTime += dt; this.stepCount++;
    this.worldClock += dt; this.envelopeClock += dt;
    while (this.envelopeClock >= 1 / WORLD_HZ) {
      const h = this.envelopeClock >= 1 / WORLD_HZ ? 1 / WORLD_HZ : this.envelopeClock;
      this.envelope.update(ego, h); this.envelopeClock -= h;
    }
    this.planClock += dt; this.optimizerClock += dt;
    const worldUpdate = this.worldClock >= 1 / WORLD_HZ || this.lastOpponents.length !== observation.rivals.length;
    if (worldUpdate) {
      const elapsed = this.worldClock; this.worldClock = 0;
      this.lastOpponents = this.opponents.update(observation, Math.max(1 / WORLD_HZ, elapsed));
      this.lastGraph = this.graphBuilder.update(ego, this.lastOpponents, this.occupancy, Math.max(1 / WORLD_HZ, elapsed));
    }
    const engaged = this.lastGraph.some(edge => (edge.a === ego.id || edge.b === ego.id)
      && (edge.ttc < 5 || edge.overlap));
    const recovering = this.needsRecovery(ego);
    if (recovering) {
      this.wasRecovering = true;
      const controls = this.recover(ego, observation.rivals, dt);
      this.finishStep(controls, observation, engaged, true);
      return controls;
    }
    if (this.wasRecovering) this.servo.reset();
    this.wasRecovering = false;
    if (!this.planner.plan || this.planClock >= 1 / PLAN_HZ) {
      this.planClock = 0;
      this.plan = this.planner.update(observation, this.lastOpponents, this.occupancy, this.lastGraph);
      this.selectedTrajectory = this.plan ? { id: this.plan.id, points: this.plan.points.map(point => ({
        x: point.x, z: point.z, y: .15, s: point.s, distance: point.distance, offset: point.offset,
        speed: point.speed, speedLimit: point.speedLimit, curvature: point.curvature,
        nx: point.nx, nz: point.nz, tx: point.tx, tz: point.tz, time: point.time,
      })) } : null;
      this.debug.candidates = this.planner.candidates.map(candidate => ({ id: candidate.id, score: candidate.score,
        points: candidate.points }));
    }
    const path = { at: s => this.planner.at(s) };
    this.optimizerClock += 0;
    let nominalSteer = this.steer;
    const firstSteer = this.servo.steerTo(ego, path, this.track, this.optimizer.steerCorrection);
    nominalSteer = firstSteer.steer; this.aim = firstSteer.target;
    const here = path.at(ego.s);
    // The plan is a physically reachable speed profile in station coordinates,
    // so the target is that profile at the car's own station. Targeting the
    // speed of a point a lookahead away instead brakes to corner speed from
    // that distance out, which is the phantom-early-brake failure.
    const targetSpeed = this.speedTarget(ego);
    const cap = this.envelope.at(ego, Math.max(ego.speed, targetSpeed), here.curvature, here.offset).speedLimit;
    const limiting = Math.min(targetSpeed, cap);
    const current = this.track.nearest(ego.x, ego.z), headingError = angle(current.heading - ego.yaw);
    const slip = Math.atan2(ego.v, Math.max(4, ego.u));
    if (this.optimizerClock >= 1 / TRANSIENT_HZ) {
      const h = this.optimizerClock; this.optimizerClock = 0;
      this.lastSolve = this.optimizer.solve(ego, path, nominalSteer, limiting, h);
    }
    if (this.lastSolve) {
      const corrected = this.servo.steerTo(ego, path, this.track, this.lastSolve.steerCorrection);
      nominalSteer = corrected.steer; this.aim = corrected.target;
    }
    const env = this.envelope.at(ego, ego.speed, here.curvature, ego.lateral);
    // Longitudinal demand is a required acceleration in m/s^2 derived from the
    // station profile, not an instantaneous speed error. Corner braking comes
    // from geometry and interaction braking from predicted occupancy; the
    // presence of a nearby car alone never lowers the target (brief section 15).
    const feedforward = this.longitudinalDemand(ego, dt);
    const required = feedforward + (this.lastSolve?.accelerationBias ?? 0);
    const actuation = this.allocator.allocate(required, ego, env, here.curvature);
    let controls = { steer: nominalSteer, throttle: actuation.throttle, brake: actuation.brake, reverse: false };
    const safety = this.safety.evaluate(ego, observation.rivals, dt);
    if (safety) { controls = { steer: safety.steer, throttle: safety.throttle, brake: safety.brake, reverse: safety.reverse }; this.state = 'SAFETY'; }
    else this.state = engaged ? (this.plan?.targetId === null ? 'ENGAGED / CLEAR CORRIDOR' : 'ENGAGED / OPPORTUNITY') : 'FREE AIR';
    this.limiter = { planned: this.planSpeed(ego.s), cap, targetSpeed: limiting, slip, lateral: current.lateral,
      feedforward, required, drive: actuation.drive, braking: actuation.braking,
      lateralDemand: actuation.lateralDemand, lookahead: firstSteer.lookahead,
      // Full control chain, for the conditional execution-loss diagnosis. The
      // point is to be able to tell a throttle that is being withheld from a
      // tyre that has nothing left, which whole-lap averages cannot separate.
      speedIntegral: this.speedIntegral,
      accelBias: this.lastSolve?.accelerationBias ?? 0,
      reserve: actuation.reserve,
      accel: actuation.acceleration,
      throttle: actuation.throttle,
      brakeCmd: actuation.brake,
      envDrive: env.drive,
      envBrake: env.brake,
      envDrag: env.drag,
      envMu: env.mu,
      gripUtil: env.utilisation,
      steer: nominalSteer,
      yawRate: ego.yawRate,
      yawError: angle(current.heading - ego.yaw),
      beta: slip,
      ax: ego.ax,
      ay: ego.ay,
      servo: this.servo.terms };
    this.steer = controls.steer; this.targetSpeed = targetSpeed;
    this.strategy.pressure = engaged ? clamp(this.strategy.pressure + dt * .25, 0, 1) : clamp(this.strategy.pressure - dt * .12, 0, 1);
    this.strategy.reason = this.plan?.targetId === null ? 'multi-corner progress value' : `space-time opportunity against ${this.plan.targetId}`;
    this.finishStep(controls, observation, engaged, false);
    return controls;
  }
  /** Plan speed at a station: the free-air profile, reduced by any plan limit. */
  planSpeed(station) {
    let speed = this.atlas.profileSpeed(station);
    const plan = this.planner.plan;
    if (plan?.points?.length) {
      const travelled = wrap(station - plan.points[0].s, this.track.length);
      if (travelled <= plan.points.at(-1).distance) {
        const limited = this.planner.at(station).speedLimit;
        if (Number.isFinite(limited)) speed = Math.min(speed, limited);
      }
    }
    return speed;
  }

  /**
   * Braking authority used for the reachability clamp. The stored profile is
   * already friction-ellipse coupled, so this uses the full straight-line
   * braking capacity: braking is completed before the corner, and using the
   * corner's reduced reserve here would force the car to arrive at apex speed
   * hundreds of metres early.
   */
  brakeAuthority(ego, station, speed) {
    // Braking is completed before the corner, so the authority is the
    // straight-line capacity. Evaluating it at the target station's own
    // curvature applies the apex's reduced friction-ellipse grip over the
    // whole braking distance and makes the car slow down far too early: at the
    // Dock Hairpin that alone cost 4.7 m/s of mid-corner speed.
    const offset = this.planner.at(station).offset;
    return this.envelope.at(ego, speed, 0, offset).brake;
  }

  /**
   * Fastest speed the car may hold right now and still reach every upcoming
   * plan station under braking. This is the braking-point law written in
   * station coordinates; once a brake event is behind the car it simply stops
   * constraining anything, so a stale target cannot survive into the next
   * corner the way a timer-based one can.
   */
  speedTarget(ego) {
    let target = Infinity;
    for (const distance of [0, 2, 4, 7, 12, 20, 32, 50, 75, 110, 160]) {
      const speed = this.planSpeed(ego.s + distance);
      const reach = distance <= 0
        ? speed
        : Math.sqrt(Math.max(0, speed * speed)
          + 2 * this.brakeAuthority(ego, ego.s + distance, speed) * Math.max(0, distance - 1.5));
      if (reach < target) target = reach;
    }
    return Number.isFinite(target) ? target : this.atlas.profileSpeed(ego.s);
  }

  /**
   * Required net longitudinal acceleration in m/s^2 to get onto the plan
   * profile and stay there.
   *
   * The target is the braking-feasible profile speed at the car's own station,
   * which already encodes every upcoming brake event in station coordinates.
   * Gains are set so full throttle is reached within about 2 m/s of error and
   * full braking within about 2.3 m/s: a gentler mapping leaves the car unable
   * to shed speed before a hairpin and it arrives dozens of km/h over the plan.
   */
  longitudinalDemand(ego, dt = 1 / 120) {
    const target = this.speedTarget(ego);
    const error = target - Math.max(0, ego.speed);
    // Pure proportional control against a moving profile lags by roughly the
    // profile's own acceleration divided by the gain, which is where the
    // standing ~1.2 m/s deficit comes from. Feed forward the profile's
    // required acceleration so the proportional term only has to absorb
    // disturbances, and integrate the residual that is left.
    const ds = Math.max(5, ego.speed * 0.3);
    const here = this.planSpeed(ego.s);
    const forward = this.planSpeed(ego.s + ds);
    const feedForward = clamp((forward * forward - here * here) / (2 * ds), -22, 22);
    this.speedIntegral = clamp(this.speedIntegral + error * dt * 16, -4, 4);
    if (error * this.speedIntegral < 0) this.speedIntegral *= 0.88;
    return clamp(error, -40, 40) * (error > 0 ? 3.6 : 6.5) + feedForward + this.speedIntegral;
  }

  /**
   * Rejoin after a genuine departure. Ported from the canonical host's proven
   * routine: a timed reverse manoeuvre with hysteresis, not a persistent state,
   * so forward and reverse never cancel each other while a crawling car
   * straddles the asphalt/runoff boundary.
   */
  needsRecovery(ego) {
    const edge = this.track.halfWidth;
    const headingError = angle(ego.heading - ego.yaw);
    // The hysteresis threshold must match the host's own off-track definition
    // (halfCarInside is |lateral| <= halfWidth). The ported gate held recovery
    // at edge - 0.7, which sits *inside* the legal racing surface: Astra's
    // racing line never reaches it, but the oracle line legitimately runs out
    // to the track edge, so every wide corner looked like a departure and the
    // car dropped to its 8 m/s rejoin target mid-lap.
    return (Math.abs(ego.lateral) > edge + 0.7 && ego.speed < 16)
      || Math.abs(headingError) > 1.3
      || this.reverseTimer > 0
      || (this.wasRecovering && (Math.abs(ego.lateral) > edge || Math.abs(headingError) > 0.6));
  }

  recover(ego, rivals, dt) {
    const edge = this.track.halfWidth;
    this.speedIntegral = 0;
    const headingError = angle(ego.heading - ego.yaw);
    const outside = this.recoveryOutside = Math.abs(ego.lateral) > edge + 0.3
      || (Boolean(this.recoveryOutside) && Math.abs(ego.lateral) > edge - 0.4);
    const target = this.track.at(ego.s + (outside ? 6 : 12), clamp(ego.lateral * 0.35, -3, 3));
    const dx = target.x - ego.x, dz = target.z - ego.z;
    const lx = dx * Math.cos(ego.yaw) - dz * Math.sin(ego.yaw);
    const lz = dx * Math.sin(ego.yaw) + dz * Math.cos(ego.yaw);

    this.reverseTimer = Math.max(0, this.reverseTimer - dt);
    this.recovery = ego.speed < 0.7 ? this.recovery + dt : 0;
    if ((lz < 0 || this.recovery > 1.7) && ego.speed < 2 && this.reverseTimer === 0) {
      this.reverseTimer = 2.0; this.recovery = 0;
    }
    if (this.reverseTimer > 0 && Math.abs(headingError) < 0.65) this.reverseTimer = 0;
    // Never reverse further into runoff when forward motion leads back in.
    const forwardOutward = (Math.sin(ego.yaw) * ego.nx + Math.cos(ego.yaw) * ego.nz) * Math.sign(ego.lateral);
    if (outside && forwardOutward < -0.15) this.reverseTimer = 0;

    const reverse = this.reverseTimer > 0;
    const steer = reverse
      ? -clamp(headingError / 1.2, -0.85, 0.85)
      : clamp(Math.atan2(2 * ego.spec.wheelbase * lx, Math.max(12, dx * dx + dz * dz)) / ego.spec.steeringLock, -0.85, 0.85);
    const recoverySpeed = reverse ? 2.8 : outside ? 4.5 : 8;
    const controls = {
      steer,
      throttle: ego.speed < recoverySpeed ? (outside ? 0.25 : 0.34) : 0,
      brake: ego.speed > recoverySpeed + 1 ? 0.65 : 0,
      reverse,
    };
    this.targetSpeed = recoverySpeed;
    this.state = reverse ? 'REVERSE RECOVERY' : 'PHYSICAL REJOIN';

    const safety = this.safety.evaluate(ego, rivals, dt);
    if (safety) return { steer: safety.steer, throttle: safety.throttle, brake: safety.brake, reverse: false };
    return controls;
  }
  finishStep(controls, observation, engaged, recovering) {
    const ego = observation.ego, safety = this.safety;
    const flags = {
      trafficLimited: this.planner.brakeEvents.reason === 'interaction',
      contact: ego.impact > .02 || ego.damage > 0, offtrack: !ego.race?.valid || Math.abs(ego.lateral) > this.track.halfWidth - ego.spec.halfWidth,
      recovery: recovering, safety: safety.reason !== 'clear',
      overlap: this.ownership.owners.size > 0,
    };
    this.learner.update(ego, flags, 1 / SERVO_HZ);
    const threatened = this.lastGraph.some(edge => edge.closing > 0 && edge.ds < 34 && edge.ds > -3);
    const retainedPass = this.updatePassEpisodes(ego, observation.rivals);
    const defensive = threatened && !this.wasDefending;
    this.wasDefending = threatened;
    this.telemetry.update({ dt: 1 / SERVO_HZ, planner: this.planner, safety, optimizer: this.optimizer,
      brakeReason: this.planner.brakeEvents.reason, engaged, retainedPass, defensive, learner: this.learner });
    this.recovery = this.recovery || (ego.speed < 1 && controls.throttle > .5 ? 1 / SERVO_HZ : 0);
  }
  updatePassEpisodes(ego, rivals) {
    let retained = false;
    for (const rival of rivals) {
      const ds = wrap(rival.s - ego.s + this.track.length / 2, this.track.length) - this.track.length / 2;
      const prior = this.passState.get(rival.id) ?? { wasBehind: ds > 0, since: 0, counted: false };
      if (ds < -2 && prior.wasBehind && !prior.counted) { prior.since += 1 / SERVO_HZ; if (prior.since > .35) { retained = true; prior.counted = true; } }
      else if (ds > 4) { prior.wasBehind = true; prior.since = 0; prior.counted = false; }
      this.passState.set(rival.id, prior);
    }
    return retained;
  }
  get diagnostics() {
    return { state: this.state, selected: this.plan?.id ?? null, targetSpeed: this.targetSpeed,
      brakeReason: this.planner.brakeEvents.reason, brakeEvents: this.planner.brakeEvents.events,
      telemetry: this.telemetry.snapshot(), limiter: this.limiter,
      atlas: { usingOracle: this.atlas.usingOracle, lapTime: this.atlas.oracle?.lapTime ?? null,
        qMax: this.atlas.qMax ?? null },
      envelope: this.envelope.state, candidates: this.planner.candidates.length };
  }
}

export { makeVortexObservation };
