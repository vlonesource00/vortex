export class BrakeEvents {
  constructor() { this.events = []; this.reason = 'clear'; }
  update(path, currentS, trackLength, observation, occupancy) {
    const events = [];
    let active = null;
    for (let i = 1; i < path.points.length; i++) {
      const a = path.points[i - 1], b = path.points[i];
      const interaction = observation.rivals.some(rival => {
        const predicted = occupancy.at(rival, b.time ?? b.distance / Math.max(6, observation.ego.speed));
        const ds = Math.abs(((predicted.s - b.s + trackLength * .5) % trackLength + trackLength) % trackLength - trackLength * .5);
        return ds < observation.ego.spec.halfLength + predicted.halfLength + 2
          && Math.abs(predicted.lateral - b.offset) < observation.ego.spec.halfWidth + predicted.halfWidth + .5;
      });
      // Corner braking is a drop in the planned speed profile. Interaction
      // braking is a separately attributed event. The two are never merged.
      const decel = (a.speedLimit ?? a.speed) - (b.speedLimit ?? b.speed);
      if (decel > .35 || interaction) {
        if (!active) active = { start: a.s, peak: b.s, release: b.s, entrySpeed: a.speedLimit ?? a.speed,
          targetSpeed: b.speedLimit ?? b.speed, reason: interaction ? 'interaction' : 'corner', acceleration: 0 };
        active.peak = b.s; active.release = b.s;
        active.targetSpeed = Math.min(active.targetSpeed, b.speedLimit ?? b.speed);
      } else if (active) { events.push(active); active = null; }
    }
    if (active) events.push(active);
    this.events = events.filter(event => event.start >= 0 && event.peak >= 0);
    this.reason = this.events.some(event => event.reason === 'interaction') ? 'interaction' : this.events.length ? 'corner' : 'clear';
    return this.events;
  }
  upcoming(s, length) { return this.events.find(event => ((event.start - s + length) % length) < 180) ?? null; }
}
