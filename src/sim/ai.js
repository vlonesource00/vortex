import { clamp, damp, angle, wrap, lerp } from './math.js';
import { SPEC } from './vehicle.js';
import { PACE } from './pace.js';
import { GlobalPace } from './global-pace.js';
import { HarborEntry } from './harbor-entry.js';
import { pathCurvature } from './path-geometry.js';
import { RaceContinuation } from './race-continuation.js';

export class RacingLine {
  constructor(track,spec=SPEC) {
    this.track = track;
    this.spec=spec;this.lineLimit=Math.min(3.9,track.halfWidth-2.6);
    const nodes = track.nodes, n = nodes.length;
    this.offset = new Float64Array(n);
    // Elastic-band optimization constrained to the usable track envelope.
    // A broad stencil reduces curvature without following sampling noise.
    for (let pass = 0; pass < 120; pass++) {
      const next = this.offset.slice();
      for (let i = 0; i < n; i++) {
        const ia = wrap(i - 6, n), ib = (i + 6) % n;
        const a = nodes[ia], b = nodes[ib], p = nodes[i];
        const mx = (a.x + a.nx * this.offset[ia] + b.x + b.nx * this.offset[ib]) / 2;
        const mz = (a.z + a.nz * this.offset[ia] + b.z + b.nz * this.offset[ib]) / 2;
        next[i] = clamp(lerp(this.offset[i], (mx - p.x) * p.nx + (mz - p.z) * p.nz, 0.12), -3.9, 3.9);
      }
      this.offset = next;
    }
    this.rebuildSpeeds();
    this.optimizeTime();
    if(track.id==='harbor-ring'){this.entry=new HarborEntry(track);this.rebuildSpeeds();}
    this.globalPace=new GlobalPace(this,spec);
    if(track.id==='harbor-ring')this.continuation=new RaceContinuation(this);
    if(spec.key!=='gt'){
      // Class power and cornering envelopes constrain both pace branches.
      for(let i=0;i<n;i++){
        const scale=Math.sqrt(spec.tyreGrip);
        this.speeds[i]=Math.min(this.speeds[i]*scale,this.globalPace.speed[i]);
        this.conservativeSpeeds[i]=Math.min(this.conservativeSpeeds[i]*scale,this.globalPace.speed[i]);
      }
    }
  }
  rebuildSpeeds() {
    const nodes=this.track.nodes,n=nodes.length,track=this.track;
    this.speeds = new Float64Array(n);
    this.conservativeSpeeds = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      const p = this.at(nodes[i].s), a = this.at(nodes[i].s - 6), b = this.at(nodes[i].s + 6);
      const aa = Math.atan2(p.x - a.x, p.z - a.z), bb = Math.atan2(b.x - p.x, b.z - p.z);
      const k = this.track.id==='harbor-ring'?Math.abs(pathCurvature(a,p,b)):Math.abs(angle(bb - aa)) / 6;
      this.speeds[i] = clamp(Math.sqrt(PACE.corner / Math.max(0.001, k)), 15, 78);
      this.conservativeSpeeds[i] = clamp(Math.sqrt(10 / Math.max(0.001, k)), 15, 78);
    }
    for (let pass = 0; pass < 4; pass++) {
      for (let i = n - 1; i >= 0; i--) { const j = (i + 1) % n, ds = wrap(nodes[j].s - nodes[i].s, track.length); this.speeds[i] = Math.min(this.speeds[i], Math.sqrt(this.speeds[j] ** 2 + 2 * PACE.lineBrake * ds)); }
      for (let i = 0; i < n; i++) { const j = wrap(i - 1, n), ds = wrap(nodes[i].s - nodes[j].s, track.length); this.speeds[i] = Math.min(this.speeds[i], Math.sqrt(this.speeds[j] ** 2 + 2 * 4.8 * ds)); }
      for (let i=n-1;i>=0;i--){const j=(i+1)%n,ds=wrap(nodes[j].s-nodes[i].s,track.length);this.conservativeSpeeds[i]=Math.min(this.conservativeSpeeds[i],Math.sqrt(this.conservativeSpeeds[j]**2+2*7.5*ds));}
      for(let i=0;i<n;i++){const j=wrap(i-1,n),ds=wrap(nodes[i].s-nodes[j].s,track.length);this.conservativeSpeeds[i]=Math.min(this.conservativeSpeeds[i],Math.sqrt(this.conservativeSpeeds[j]**2+2*4.8*ds));}
    }
  }
  estimatedTime(corner=PACE.corner,brake=PACE.lineBrake) {
    const points=this.track.nodes.map(p=>this.at(p.s)),n=points.length;
    const speed=new Float64Array(n),distance=new Float64Array(n);
    for(let i=0;i<n;i++) {
      const a=points[wrap(i-1,n)],b=points[i],c=points[(i+1)%n];
      distance[i]=Math.hypot(c.x-b.x,c.z-b.z);
      const curvature=Math.abs(angle(Math.atan2(c.x-b.x,c.z-b.z)-Math.atan2(b.x-a.x,b.z-a.z)))/Math.max(.1,(distance[i]+Math.hypot(b.x-a.x,b.z-a.z))/2);
      speed[i]=Math.min(78,Math.sqrt(corner/Math.max(.001,curvature)));
    }
    for(let pass=0;pass<4;pass++){
      for(let i=n-1;i>=0;i--)speed[i]=Math.min(speed[i],Math.sqrt(speed[(i+1)%n]**2+2*brake*distance[i]));
      for(let i=0;i<n;i++){const j=wrap(i-1,n);speed[i]=Math.min(speed[i],Math.sqrt(speed[j]**2+2*4.8*distance[j]));}
    }
    return points.reduce((t,p,i)=>t+2*distance[i]/(speed[i]+speed[(i+1)%n]),0);
  }
  optimizeTime() {
    // Whole-lap travel time couples each apex to its braking zone and next
    // straight. Always include the original geometry as a feasible fallback.
    const original=this.offset.slice(),nodes=this.track.nodes,n=nodes.length;
    const referenceAt=s=>{const p=this.track.at(s),i=p.index,j=(i+1)%n;return lerp(original[i],original[j],wrap(p.s-nodes[i].s,this.track.length)/wrap(nodes[j].s-nodes[i].s,this.track.length));};
    let best=this.estimatedTime();this.optimization={baselineTime:best,predictedTime:best,scale:1,shift:0};
    let selected=original;
    for(const scale of [.7,1,1.3,1.6])for(const shift of [-12,-6,0,6,12]){
      this.offset=Float64Array.from(nodes,p=>clamp(referenceAt(p.s+shift)*scale,-3.9,3.9));
      const time=this.estimatedTime();
      if(time<best){best=time;selected=this.offset;this.optimization={...this.optimization,predictedTime:time,scale,shift};}
    }
    this.offset=selected;this.rebuildSpeeds();
  }
  at(s, extra = 0) {
    const p = this.track.at(s), i = p.index, j = (i + 1) % this.offset.length;
    const ds = wrap(this.track.nodes[j].s - this.track.nodes[i].s, this.track.length);
    const t = clamp(wrap(p.s - this.track.nodes[i].s, this.track.length) / ds, 0, 1);
    let optimized=lerp(this.offset[i],this.offset[j],t);
    const entry=this.entry?.at(p.s,p,optimized);
    if(entry)optimized=entry.offset;
    const limit=this.track.id==='harbor-ring'?this.track.halfWidth-1.2:4.65;
    const offset = clamp(optimized + extra, -limit, limit);
    if(entry){p.x=entry.x+p.nx*(offset-entry.offset);p.z=entry.z+p.nz*(offset-entry.offset);}
    else {p.x+=p.nx*offset;p.z+=p.nz*offset;}
    p.offset=offset;
    p.speed=this.speeds?lerp(this.speeds[i],this.speeds[j],t):0;
    p.conservativeSpeed=this.conservativeSpeeds?lerp(this.conservativeSpeeds[i],this.conservativeSpeeds[j],t):0;
    return p;
  }
  offsetAt(s) {
    // Opponent hypotheses only need a lateral target, not an allocated pose.
    const nodes=this.track.nodes,n=nodes.length;s=wrap(s,this.track.length);
    let lo=0,hi=n-1;
    while(lo<hi){const mid=Math.ceil((lo+hi)/2);if(nodes[mid].s<=s)lo=mid;else hi=mid-1;}
    const j=(lo+1)%n,t=(s-nodes[lo].s)/wrap(nodes[j].s-nodes[lo].s,this.track.length);
    const optimized=lerp(this.offset[lo],this.offset[j],t);
    return optimized;
  }
}

export class Driver {
  constructor(id, line, skill = 0.95) {
    this.id = id; this.line = line; this.skill = skill;
    this.offset = 0; this.targetOffset = 0; this.commit = 0; this.state = 'PACE'; this.targetSpeed = 0; this.steer = 0; this.recovery = 0;
  }
  update(car, cars, dt) {
    const SPEC=car.spec;
    const track = this.line.track, current = track.nearest(car.x, car.z);
    this.commit = Math.max(0, this.commit - dt);
    let targetSpeed = this.line.at(current.s + Math.max(4, car.speed * 0.2)).speed * this.skill * (1 - track.wetness * 0.24);
    let front = null, gap = Infinity;
    for (const other of cars) {
      if (other === car) continue;
      const ds = wrap(other.s - current.s + track.length / 2, track.length) - track.length / 2;
      if (ds > 0 && ds < gap && Math.abs(other.lateral - current.lateral) < 3.2) { front = other; gap = ds; }
      // Predicted overlap protection is independent of tactical commitment.
      if (Math.abs(ds) < 6 && Math.abs(other.lateral - current.lateral) < 2.25) {
        this.targetOffset = clamp((current.lateral < other.lateral ? -1 : 1) * 2.6, -3.1, 3.1);
        this.commit = Math.max(this.commit, 0.6); this.state = 'SIDE BY SIDE';
      }
    }
    if (front && gap < 40 && this.commit === 0 && car.speed > front.speed - 1) {
      const p = this.line.at(current.s + 25);
      const candidates = [-3.0, 3.0].map(offset => {
        const lane = clamp(p.offset + offset, -4.65, 4.65);
        let risk = Math.abs(lane - current.lateral) * 0.3;
        for (const other of cars) if (other !== car) {
          const ds = wrap(other.s - current.s + track.length / 2, track.length) - track.length / 2;
          if (ds > -12 && ds < 48) risk += Math.max(0, 2.8 - Math.abs(other.lateral - lane)) * (50 - Math.abs(ds));
        }
        return { offset, risk };
      }).sort((a,b) => a.risk - b.risk);
      this.targetOffset = candidates[0].offset; this.commit = 2.5; this.state = 'OVERTAKE';
    }
    if (this.commit === 0) { this.targetOffset = 0; this.state = 'PACE'; }
    this.offset = damp(this.offset, this.targetOffset, 1.35, dt);
    if (front && gap < Math.max(9, car.speed * 0.8) && Math.abs(front.lateral - current.lateral) < 2.2) {
      targetSpeed = Math.min(targetSpeed, Math.max(0, front.speed + (gap - 8) * 0.75)); this.state = 'FOLLOW';
    }
    if (Math.abs(current.lateral) > 7.2) { targetSpeed = Math.min(targetSpeed, 14); this.offset = 0; this.state = 'REJOIN'; }
    const lookahead = clamp(6 + car.speed * 0.42, 8, 34);
    const target = this.line.at(current.s + lookahead, this.offset);
    const dx = target.x - car.x, dz = target.z - car.z;
    const lx = dx * Math.cos(car.yaw) - dz * Math.sin(car.yaw);
    const distance2 = dx * dx + dz * dz;
    const delta = Math.atan2(2 * SPEC.wheelbase * lx, Math.max(5, distance2));
    const slip = Math.atan2(car.v, Math.max(4, car.u));
    const steer = clamp((delta + slip * 0.45) / SPEC.steeringLock, -1, 1);
    this.steer = damp(this.steer, steer, 10, dt);
    const error = targetSpeed - car.speed;
    this.targetSpeed = targetSpeed;
    car.controls = { steer: this.steer, throttle: clamp(error * 0.34 + 0.12, 0, 1), brake: clamp(-error * 0.2, 0, 1) };
    if (front && gap < 6 && Math.abs(front.lateral - current.lateral) < 2) { car.controls.throttle = 0; car.controls.brake = 1; }
    this.recovery = car.speed < 1 && car.controls.throttle > 0.5 ? this.recovery + dt : 0;
  }
}
