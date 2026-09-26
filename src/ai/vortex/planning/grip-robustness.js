import { clamp, wrap } from '../../../sim/math.js';

/**
 * Grip-adaptive free-air line.
 *
 * The oracle geometry is time-optimal at the grip it was solved for. At lower
 * grip the same geometry is not merely slower, it is dynamically fragile: on
 * the measured final-corner state the nominal line runs out of rear stability
 * margin entirely and departs, while the same corner taken a metre and a half
 * further from the turn centre still has margin left. In the controlled
 * robustness laboratory that wider line costs 0.034 s on a healthy tyre and
 * removes the departure on a degraded one, which is exactly the trade worth
 * taking.
 *
 * Two things were measured about how that has to be applied. The offset has to
 * cover the approach as well as the apex - the nominal line's approach is where
 * the yaw transient is spent and the margin is gone before the corner arrives -
 * so it is a wide window around the event, not a local bump on it. And it has
 * to be centred on one event at a time: a deformation spread over every
 * demanding corner of the lap costs more at the others than it saves here.
 *
 * So the line selects between discrete free-air geometries on measured physical
 * capability, and applies the chosen geometry to the single most demanding
 * event ahead. Nothing here names a corner, a station or a lap: the event is
 * whatever the tightest upcoming demand happens to be, and the same search
 * anywhere on any circuit returns that.
 */

// Three free-air geometries, and the remaining-grip bands that select them.
// The amplitudes are measured: 1.5 m clears the departure at 0.034 s of fresh
// cost, 2.5 m clears it with about 2% more grip margin at 0.067 s.
const BANDS = [
  { minGrip: 0.968, amp: 0 },
  { minGrip: 0.938, amp: 1.5 },
  { minGrip: -Infinity, amp: 2.5 },
];

const WINDOW = 400;
const HORIZON = 380;

export class GripRobustness {
  constructor(atlas, track) {
    this.atlas = atlas;
    this.track = track;
    this.amplitude = 0;
    this.requested = 0;
    this.apexS = null;
    this.apexSign = -1;
    this.apexDemand = 0;
    // Off by default. The controlled laboratory shows the wider line is worth
    // having - 0.034 s on a healthy tyre against a whole corner on a degraded
    // one - but applying it over a full stint measured worse than the nominal
    // line in every configuration tried, including the state-adaptive one:
    // the lap has several events at the lateral limit and opening all of them
    // costs more than the one it saves. The machinery stays because the
    // measurement is about how it is applied, not about whether the line is
    // better, and `forcedAmp` lets the ablations measure each case on its own.
    this.enabled = false;
    // Set by ablation tools to pin a geometry and stop the state-adaptive
    // selection, so each configuration can be measured on its own.
    this.forcedAmp = null;
  }

  /** Planned lateral utilisation at a station: demand, not curvature. */
  demand(s) {
    const v = Math.max(6, this.atlas.profileSpeed(s));
    const k = Math.abs(this.atlas.lineCurvature(s));
    const capacity = this.atlas.envelope ? this.atlas.envelope.lateralAt(v) : 13;
    return v * v * k / Math.max(1, capacity);
  }

  /**
   * Latch onto the most demanding event ahead and stay with it until it is
   * behind the car. Without the latch the window slides as the search moves,
   * and a line that slides sideways under the car is a disturbance rather than
   * a line change - which is what the first production attempt measured.
   */
  survey(station) {
    const len = this.track.length;
    const ahead = d => wrap(station + d, len);
    // Keep the current event while it is still ahead of the car and not yet
    // clearly bettered by something further on.
    if (this.apexS !== null) {
      const d = wrap(this.apexS - station + len / 2, len) - len / 2;
      if (d > -40 && d < HORIZON && this.apexDemand >= this.bestDemand(station) * 0.92) return this.apexS;
    }
    let best = -1, bestS = station;
    for (let d = 0; d <= HORIZON; d += 12) {
      const s = ahead(d);
      const dem = this.demand(s);
      if (dem > best) { best = dem; bestS = s; }
    }
    this.apexS = bestS;
    this.apexDemand = best;
    this.apexSign = this.atlas.lineCurvature(bestS) < 0 ? -1 : 1;
    return bestS;
  }

  bestDemand(station) {
    let best = -1;
    for (let d = 0; d <= HORIZON; d += 24) {
      best = Math.max(best, this.demand(wrap(station + d, this.track.length)));
    }
    return best;
  }

  /** Discrete geometry choice on measured grip remaining. */
  select(gripRemaining) {
    this.requested = this.forcedAmp !== null
      ? this.forcedAmp
      : (this.enabled ? (BANDS.find(b => gripRemaining >= b.minGrip)?.amp ?? 0) : 0);
    // Damp the applied amplitude so the line does not step when the grip band
    // changes mid-lap; a slowly arriving line change is a line change, a
    // sudden one is a disturbance. Called once per plan update, not per point.
    this.amplitude += (this.requested - this.amplitude) * 0.05;
    return this.requested;
  }

  /**
   * Lateral offset to add to the oracle line at a station: a wide smooth
   * window centred on the latched event, directed away from its turn centre.
   * Negative curvature turns left, so the outside of that corner is positive
   * lateral.
   */
  shift(s) {
    if (this.apexS === null || Math.abs(this.amplitude) < 1e-4) return 0;
    const len = this.track.length;
    const d = wrap(s - this.apexS + len / 2, len) - len / 2;
    const x = (d + WINDOW / 2) / WINDOW;
    if (x <= 0 || x >= 1) return 0;
    const w = 0.5 * (1 - Math.cos(2 * Math.PI * x));
    return -this.apexSign * this.amplitude * w;
  }

  get active() { return Math.abs(this.amplitude) > 0.05; }
}
