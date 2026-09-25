import { Vehicle, wakes, collisions } from './vehicle.js';
import { RacingLine } from './ai.js';
import { AdaptiveDriver as Driver } from './controller.js';
import { wrap, clamp } from './math.js';
import { raceInterval } from './interval.js';
import { halfCarInside } from './racecraft-policy.js';
import { CLASS_IDS, carSpecFor } from './car-specs.js';

const GRID = [ ['YOU', '#df482d'], ['M. ROSSI', '#d4dbd9'], ['J. MOREAU', '#356653'], ['A. WEBER', '#d7a32e'], ['K. SATO', '#2e515f'], ['L. COSTA', '#a0a399'], ['O. REED', '#4058a0'], ['S. LAURENT', '#d5c6a8'] ];
export class Session {
  constructor(track,{classId='gt',mixed=false}={}) {
    this.track = track; this.line = new RacingLine(track);
    this.classId=carSpecFor(classId).key;this.mixed=mixed;
    this.lines=new Map([['gt',this.line]]);
    const lineFor=id=>{if(!this.lines.has(id))this.lines.set(id,new RacingLine(track,carSpecFor(id)));return this.lines.get(id);};
    this.cars = GRID.map(([name,color],id) => new Vehicle(id,name,color,mixed?CLASS_IDS[(id+CLASS_IDS.indexOf(this.classId))%CLASS_IDS.length]:this.classId));
    this.lineFor=car=>lineFor(car.classId);
    this.line=this.lineFor(this.cars[0]);
    this.aggression=.72;this.paceObjective='race';
    this.drivers = this.cars.map((c,i) => new Driver(i,this.lineFor(c),0.952 + (i % 4) * 0.008,this.aggression));
    this.player = this.cars[0]; this.mode = 'race'; this.laps = 3; this.field = 6;
    this.phase = 'menu'; this.time = 0; this.countdown = 0; this.contacts = 0; this.autopilot = false;
    this.reset();
  }
  get activeCars() { return this.cars.slice(0,this.mode === 'practice' ? 1 : this.field); }
  reset() {
    this.time = 0; this.contacts = 0; this.results = null; this.collisionStats = {peakClosing:0,severeContacts:0};
    this.cars.forEach((c,i) => {
      const start=this.track.scenario?.start,rowSpacing=start?.rowSpacingM??9.5,lane=start?.laneOffsetM??2.3;
      c.place(this.track, this.track.gridS - Math.floor(i / 2) * rowSpacing, i % 2 ? -lane : lane);
      this.drivers[i] = new Driver(i,this.lineFor(c),0.952 + (i % 4) * 0.008,this.aggression);
      const gridToFinish=wrap(this.track.finishS-this.track.gridS,this.track.length);
      c.race = { progress: -gridToFinish-Math.floor(i / 2) * rowSpacing, previousS: c.s, lap: 1, lastLap: null, bestLap: null, lapStart: 0, sector: 0, valid: true, sectors: [], finishTime: null, offtrack: 0 };
    });
    this.timingHistory=this.cars.map(c=>[{progress:c.race.progress,time:0}]);this.nextTimingAt=0;
  }
  start({freshTrack=false}={}) { if(freshTrack)this.track.rubber.fill(0);this.reset(); this.phase = 'countdown'; this.countdown = 4; }
  step(dt, playerControls) {
    if (!['racing','countdown'].includes(this.phase)) return;
    if (this.phase === 'countdown') { this.countdown -= dt; if (this.countdown <= 0) this.phase = 'racing'; return; }
    this.time += dt;
    const cars = this.activeCars;
    const projections=new Map(cars.map(c=>[c.id,this.track.nearest(c.x,c.z)]));
    const order=[...cars].sort((a,b)=>(a.race.finishTime??Infinity)-(b.race.finishTime??Infinity)||b.race.progress-a.race.progress);
    const context={projections,order,totalLaps:this.laps,mode:this.mode,time:this.time,paceObjective:this.paceObjective};
    cars.forEach((c,i) => {
      if (i === 0 && !this.autopilot) c.controls = playerControls;
      else this.drivers[i].update(c,cars,dt,context);
      // Keep finished cars moving under AI steering instead of parking them on
      // the racing line in front of competitors still completing their lap.
      if(c.race.finishTime!==null)c.controls={...c.controls,throttle:Math.min(.35,c.controls.throttle),brake:Math.max(c.controls.brake,c.speed>25?.2:0)};
    });
    const airflow = wakes(cars);
    cars.forEach((c,i) => c.step(dt,this.track,airflow[i]));
    this.contacts += collisions(cars,this.collisionStats);
    for (const c of cars) {
      const r = c.race;
      const previousProgress=r.progress;
      const delta = wrap(c.s - r.previousS + this.track.length / 2,this.track.length) - this.track.length / 2;
      r.previousS = c.s;
      if (Math.abs(delta) < 20) r.progress += delta;
      if(this.track.scenario&&previousProgress<0&&r.progress>=0){r.lapStart=this.time;r.valid=true;}
      if (!halfCarInside(c.lateral,this.track.halfWidth)) { r.valid = false; r.offtrack += dt; }
      const totalSectors = Math.floor(Math.max(0,r.progress) / (this.track.length / 3));
      if (totalSectors > r.sector) {
        r.sectors.push(this.time); r.sector = totalSectors;
        if (totalSectors % 3 === 0) {
          r.lastLap = this.time - r.lapStart;
          if (r.valid && (r.bestLap === null || r.lastLap < r.bestLap)) r.bestLap = r.lastLap;
          r.lapStart = this.time; r.lap++; r.valid = true;
          if (this.mode !== 'practice' && r.lap > this.laps && r.finishTime === null) r.finishTime = this.time;
        }
      }
    }
    if(this.time>=this.nextTimingAt){
      this.nextTimingAt=this.time+.25;
      for(const c of cars){const h=this.timingHistory[c.id];if(c.race.progress>h.at(-1).progress){h.push({progress:c.race.progress,time:this.time});if(h.length>2400)h.shift();}}
    }
    if (this.player.race.finishTime !== null) { this.phase = 'finished'; this.results = this.standings(); }
  }
  interval(car,leader=this.standings()[0]){
    if(car===leader)return 0;
    if(car.race.finishTime!==null&&leader.race.finishTime!==null)return car.race.finishTime-leader.race.finishTime;
    return raceInterval(this.timingHistory[leader.id],car.race.progress,this.time,leader.race.progress);
  }
  standings() { return [...this.activeCars].sort((a,b) => (a.race.finishTime ?? Infinity) - (b.race.finishTime ?? Infinity) || b.race.progress - a.race.progress); }
  recover() {
    const c = this.player, saved = { ...c.race }, fuel = c.fuel, damage = c.damage;
    // Recovery never advances race distance and incurs a five-second penalty.
    c.place(this.track,c.s,0,0); c.race = saved; c.race.previousS = c.s; c.race.valid = false;
    c.fuel = fuel; c.damage = damage; this.time += 5;
  }
}
