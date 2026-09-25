import { angle, clamp, wrap } from '../../../sim/math.js';
import { OpponentProfile } from './opponent-profile.js';

export class OpponentFilter {
  constructor(track) { this.track = track; this.states = new Map(); this.profiles = new Map(); this.lastUpdate = null; }
  update(observation, dt) {
    const length = this.track.length;
    for (const rival of observation.rivals) {
      let state = this.states.get(rival.id);
      if (!state) {
        state = { ...rival, longitudinalSpeed: rival.vx * rival.tx + rival.vz * rival.tz,
          lateralSpeed: 0, acceleration: 0, hypotheses: { hold: .68, cover: .12, lateBrake: .08, earlyBrake: .06, yield: .06 },
          previousBrake: rival.controls?.brake ?? 0, overlap: false };
        this.states.set(rival.id, state); this.profiles.set(rival.id, new OpponentProfile(rival.id));
      } else {
        const ds = wrap(rival.s - state.s + length / 2, length) - length / 2;
        const measuredSpeed = ds / Math.max(.001, dt), measuredQ = (rival.lateral - state.lateral) / Math.max(.001, dt);
        const speed = rival.vx * rival.tx + rival.vz * rival.tz;
        const alpha = clamp(dt * 5, .03, .28);
        const profile = this.profiles.get(rival.id); profile.update(state, rival, dt);
        Object.assign(state, rival, { longitudinalSpeed: speed * .72 + measuredSpeed * .28,
          lateralSpeed: state.lateralSpeed * (1 - alpha) + measuredQ * alpha,
          acceleration: clamp(state.acceleration * .72 + ((speed - state.longitudinalSpeed) / Math.max(.02, dt)) * .28, -12, 8),
          overlap: false });
        const h = state.hypotheses, brake = rival.controls?.brake ?? 0;
        h.lateBrake = clamp(h.lateBrake + (brake > .65 && rival.speed > 15 ? .035 : -.009), .02, .7);
        h.cover = clamp(h.cover + (Math.abs(state.lateralSpeed) > .6 ? .045 : -.012), .02, .65);
        h.yield = clamp(h.yield + (brake > .6 && Math.abs(state.lateralSpeed) < .2 ? .012 : -.004), .01, .4);
        h.hold = Math.max(.1, 1 - h.lateBrake - h.cover - h.yield - .08);
        const total = Object.values(h).reduce((sum, value) => sum + value, 0);
        for (const key of Object.keys(h)) h[key] /= total;
        state.previousBrake = brake;
      }
    }
    for (const id of this.states.keys()) if (!observation.rivals.some(rival => rival.id === id)) { this.states.delete(id); this.profiles.delete(id); }
    this.lastUpdate = observation.time;
    return [...this.states.values()];
  }
  get(id) { return this.states.get(id); }
}
