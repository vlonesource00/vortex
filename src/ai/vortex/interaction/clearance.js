/**
 * The single source of truth for what counts as contact, proximity, ownership
 * and traffic conflict in VORTEX's combat layer.
 *
 * Every combat subsystem previously carried its own inline literal (+0.34,
 * +0.42, +0.44, +0.50, +0.65, +0.85, +2.0 ...). Several of those numbers
 * disagreed with each other in ways that made the generated attack corridor
 * simultaneously a "valid side-by-side route" and a "traffic obstruction",
 * and made a legal flank cost almost as much as a real impact. See
 * tools/audit-clearance.mjs for the measured table that forced this module.
 *
 * All margins are measured as LATERAL BODY CLEARANCE: the free gap between the
 * two car bodies, not centre-to-centre distance. A margin of 0 means the two
 * body edges touch. This keeps every number physically interpretable.
 */

/** Two bodies physically touching. Contact begins here. */
export const BODY_CONTACT_CLEARANCE = 0.0;

/**
 * Legal close racing at low relative energy. A car may run here wheel-to-wheel
 * with no tactical penalty. Used by the contact model to bound the cost of
 * parallel low-energy side-by-side.
 */
export const LOW_ENERGY_RACE_CLEARANCE = 0.30;

/**
 * Below this lateral body clearance the plan is traffic-restricted: the ego
 * cannot simply hold free-air speed through the rival's space.
 *
 * This is the number that was previously 0.44 in trajectory-refiner while the
 * generator placed the attack flank at 0.42 -- so the dedicated passing
 * corridor was traffic-capped by construction.
 */
export const INTERACTION_MARGIN = 0.55;

/**
 * Clearance at which the planner is willing to DECLARE an attack corridor. It
 * must sit comfortably OUTSIDE INTERACTION_MARGIN so an established flank is
 * not immediately reclassified as traffic. The extra band is the physical
 * requirement: the car is moving, and a flank that leaves zero tolerance for
 * prediction error is not a usable pass route.
 */
export const ATTACK_CORRIDOR_MARGIN = 0.72;

/** Longitudinal half-gap beyond the two bodies at which contact cost starts. */
export const CONTACT_LONGITUDINAL_MARGIN = 0.65;

/**
 * Lateral gap over which proximity cost decays. Scales how quickly a car stops
 * being a contact threat as it pulls away. Kept well under a car width so a
 * genuinely separate route is not taxed.
 */
export const PROXIMITY_LAT_DECAY = 1.0;

/** Longitudinal gap over which proximity cost decays. */
export const PROXIMITY_LONG_DECAY = 3.0;

/** Ownership (corridor ownership) activates inside this lateral clearance. */
export const OWNERSHIP_MARGIN = 0.85;

/** Ownership activates inside this longitudinal clearance. */
export const OWNERSHIP_LONGITUDINAL_MARGIN = 0.80;

/** Ownership releases only once this clear distance is reached. */
export const OWNERSHIP_RELEASE_LONGITUDINAL = 1.40;

/** An interaction brake EVENT is reported inside this lateral clearance. */
export const BRAKE_EVENT_MARGIN = 0.50;

/** An interaction brake EVENT is reported inside this longitudinal clearance. */
export const BRAKE_EVENT_LONGITUDINAL_MARGIN = 2.00;

/** Engagement overlap inside this lateral clearance. */
export const ENGAGEMENT_MARGIN = 0.65;

/** Engagement overlap inside this longitudinal clearance. */
export const ENGAGEMENT_LONGITUDINAL_MARGIN = 1.25;

/** Safety kernel emergency intervention inside this lateral clearance. */
export const SAFETY_MARGIN = 0.15;

/** Safety kernel emergency intervention inside this longitudinal clearance. */
export const SAFETY_LONGITUDINAL_MARGIN = 1.10;

/**
 * Physical meaning, for the audit trail:
 *
 *   BODY_CONTACT_CLEARANCE      -- bodies touch. Physical.
 *   SAFETY_MARGIN               -- last-resort intervention, deliberately tight.
 *   LOW_ENERGY_RACE_CLEARANCE   -- legal wheel-to-wheel. Tactical, not physical.
 *   INTERACTION_MARGIN          -- below this the plan may be speed-restricted.
 *   ATTACK_CORRIDOR_MARGIN      -- a flank is only a flank beyond this.
 *   OWNERSHIP_MARGIN            -- who is entitled to the corridor.
 *   BRAKE_EVENT_MARGIN          -- telemetry attribution, not a physical limit.
 *   ENGAGEMENT_MARGIN           -- planning relevance, deliberately generous.
 *
 * Correct numerical ordering (the previous comment claimed
 * SAFETY <= BODY_CONTACT, which is false: 0.15 > 0.00):
 *
 *   BODY_CONTACT (0.00) < SAFETY (0.15) < LOW_ENERGY_RACE (0.30)
 *     < INTERACTION (0.55) < ATTACK_CORRIDOR (0.72) < OWNERSHIP (0.85)
 *
 * SAFETY sits ABOVE body contact because a last-resort intervention must fire
 * before the bodies actually touch, not after. BRAKE_EVENT (0.50) and
 * ENGAGEMENT (0.65) sit between INTERACTION and OWNERSHIP because they are
 * reporting and planning horizons, not physical limits.
 */
export const CLEARANCE = Object.freeze({
  BODY_CONTACT_CLEARANCE,
  LOW_ENERGY_RACE_CLEARANCE,
  INTERACTION_MARGIN,
  ATTACK_CORRIDOR_MARGIN,
  CONTACT_LONGITUDINAL_MARGIN,
  PROXIMITY_LAT_DECAY,
  PROXIMITY_LONG_DECAY,
  OWNERSHIP_MARGIN,
  OWNERSHIP_LONGITUDINAL_MARGIN,
  OWNERSHIP_RELEASE_LONGITUDINAL,
  BRAKE_EVENT_MARGIN,
  BRAKE_EVENT_LONGITUDINAL_MARGIN,
  ENGAGEMENT_MARGIN,
  ENGAGEMENT_LONGITUDINAL_MARGIN,
  SAFETY_MARGIN,
  SAFETY_LONGITUDINAL_MARGIN,
});

/**
 * Classify a side-by-side situation into the states the brief requires.
 * Returns one of PROXIMITY | SIDE_BY_SIDE_CLEAR | FOLLOW_BLOCKED |
 * CONVERGING_CONFLICT | CROSSING_CONFLICT.
 *
 * The bodies are rectangles: they intersect only when they overlap on BOTH
 * axes. Longitudinal overlap alone is simply side-by-side racing, which must
 * not be reported as a conflict. A rival BEHIND the candidate is never a
 * reason for the candidate to slow down.
 *
 * @param {number} latGap      lateral body clearance (m, >0 means separated)
 * @param {number} alongGap    longitudinal body clearance (m, >0 means separated)
 * @param {number} relSpeedLong closing speed along track (m/s)
 * @param {number} relSpeedLat  closing speed laterally (m/s)
 * @param {boolean} rivalAhead  rival is ahead of or overlapping the candidate
 */
export function classifyConflict(latGap, alongGap, relSpeedLong, relSpeedLat, rivalAhead = true) {
  const overLat = Math.max(0, -latGap);
  const overLong = Math.max(0, -alongGap);
  const intersecting = overLat > 0 && overLong > 0;
  const relSpeed = Math.hypot(relSpeedLong, relSpeedLat);

  if (intersecting) return relSpeed > 8 ? 'CROSSING_CONFLICT' : 'CONVERGING_CONFLICT';

  // A rival behind the candidate cannot restrict its speed.
  if (!rivalAhead) return 'PROXIMITY';

  // Separated laterally beyond the interaction boundary: legal parallel racing
  // or clean air. This is the state a real attack flank must land in.
  if (latGap >= INTERACTION_MARGIN) {
    return alongGap < 0
      ? 'SIDE_BY_SIDE_CLEAR'
      : (relSpeedLong > 4 ? 'FOLLOW_BLOCKED' : 'PROXIMITY');
  }

  // Inside the interaction boundary but not touching.
  if (relSpeedLat > 1.5) return 'CROSSING_CONFLICT';
  if (alongGap < 0) {
    // Side-by-side inside the interaction band. Legal close racing at low
    // relative energy; a genuine squeeze at high energy.
    return relSpeed > 12 ? 'CONVERGING_CONFLICT' : 'SIDE_BY_SIDE_CLEAR';
  }
  if (relSpeedLong > 4) return 'FOLLOW_BLOCKED';
  return 'PROXIMITY';
}

/** Which conflict states are allowed to restrict longitudinal target speed. */
export function restrictsSpeed(state) {
  return state === 'FOLLOW_BLOCKED'
    || state === 'CONVERGING_CONFLICT'
    || state === 'CROSSING_CONFLICT';
}

/**
 * Physical lateral velocity of a candidate at a path point.
 *
 * Candidate points are SPATIALLY separated by the corridor step (10 m), not by
 * one planner tick. Estimating v_q as `dq * PLAN_HZ` therefore invents lateral
 * speeds up to 54 m/s for a routine 2 m transition over 10 m. The correct
 * relation is
 *
 *     v_q = (dq / ds) * v
 *
 * i.e. the local path slope times the predicted speed. Both TrajectoryRefiner
 * and OpportunityField MUST use this helper so they cannot drift apart.
 *
 * @param {number} offsetDelta  q(n) - q(n-1) in metres
 * @param {number} dsStep       distance between the two points in metres
 * @param {number} speed        predicted vehicle speed in m/s
 */
export function candidateLateralVelocity(offsetDelta, dsStep, speed) {
  return (offsetDelta / Math.max(0.1, dsStep)) * speed;
}
