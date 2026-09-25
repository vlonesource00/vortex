import { angle, clamp, wrap } from './math.js';

export const relativeDistance = (a,b,length) => wrap(a-b+length/2,length)-length/2;

// Track-coordinate predictions with an expanding uncertainty envelope.
// Constant acceleration for 0.8 s, then constant velocity, with bounded drift.
export class Perception {
  constructor(track){this.track=track;this.history=new Map();this.time=0;}
  scan(car,cars,dt=.08,line=null,projections=null){
    this.time+=dt;
    const track=this.track,origin=projections?.get(car.id)??track.nearest(car.x,car.z);
    const forwardRange=Math.max(170,Math.min(420,car.speed*car.speed/10+car.speed*.6+20));
    const observations=[];
    for(const other of cars){
      if(other===car)continue;
      const p=projections?.get(other.id)??track.nearest(other.x,other.z),distance=relativeDistance(p.s,origin.s,track.length);
      if(distance < -65 || distance > forwardRange)continue;
      const longitudinal=other.vx*p.tx+other.vz*p.tz;
      const lateral=other.vx*p.nx+other.vz*p.nz;
      const headingError=angle(other.yaw-p.heading);
      // Rotated footprint: a sideways car occupies a much wider corridor.
      const halfWidth=Math.abs(Math.cos(headingError))*.99+Math.abs(Math.sin(headingError))*2.3;
      const halfLength=Math.abs(Math.cos(headingError))*2.3+Math.abs(Math.sin(headingError))*.99;
      const observation={id:other.id,name:other.name,distance,s:p.s,lateral:p.lateral,speed:Math.max(0,longitudinal),lateralSpeed:lateral,acceleration:clamp(other.ax,-10,6),halfWidth,halfLength,headingError,damage:other.damage,pursuerLateral:origin.lateral,progress:other.race?.progress};
      if(line)this.learn(observation,line);
      observations.push(observation);
    }
    const lanes=Array.from({length:11},(_,i)=>-5.2+i*1.04).map(lateral=>{
      let clearance=150,closing=0;
      for(const o of observations)if(o.distance>-6&&Math.abs(o.lateral-lateral)<o.halfWidth+1.03){if(o.distance<clearance){clearance=o.distance;closing=car.speed-o.speed;}}
      return {lateral,clearance,closing,free:clearance>Math.max(12,car.speed*.6)};
    });
    for(const [id,memory] of this.history)if(this.time-memory.seen>15)this.history.delete(id);
    return {origin,observations,lanes,speed:Math.max(0,car.vx*origin.tx+car.vz*origin.tz),lateralSpeed:car.vx*origin.nx+car.vz*origin.nz};
  }
  learn(o,line){
    let m=this.history.get(o.id);
    if(!m){m={weights:[.6,.25,.15,0],seen:this.time,samples:0,error:0,brakeStrength:3,preferredLane:o.lateral,brakeS:null};this.history.set(o.id,m);}
    m.seen=this.time;
    if(m.previous&&this.time-m.sampleTime>=.8){
      const dt=this.time-m.sampleTime,predictions=this.responses(m.previous,dt,line);
      const errors=predictions.map(p=>Math.hypot(relativeDistance(p.s,o.s,this.track.length)/4,(p.lateral-o.lateral)/.7));
      const likelihood=errors.map((e,i)=>Math.exp(-Math.min(12,e*e)*.5)*Math.max(.08,m.weights[i]));
      const total=likelihood.reduce((a,b)=>a+b,0);
      m.weights=likelihood.map((v,i)=>.85*m.weights[i]+.15*(.04+.84*v/total));
      m.error=.8*m.error+.2*errors.reduce((sum,e,i)=>sum+e*m.weights[i],0);m.samples++;
      m.previous=null;
    }
    if(o.acceleration< -2){m.brakeStrength+=.1*(clamp(-o.acceleration,2,9)-m.brakeStrength);m.brakeS=o.s;}
    m.preferredLane+=.03*(o.lateral-m.preferredLane);
    if(!m.previous){m.previous={...o};m.sampleTime=this.time;}
  }
  predict(o,t){
    t=Math.max(0,t);
    const accelerationTime=Math.min(.8,t,o.acceleration<0?o.speed/-o.acceleration:Infinity);
    const speed=Math.max(0,o.speed+o.acceleration*accelerationTime);
    const ds=o.speed*accelerationTime+.5*o.acceleration*accelerationTime**2+speed*(t-accelerationTime);
    return {s:o.s+ds,distance:o.distance+ds,lateral:clamp(o.lateral+o.lateralSpeed*.48*(1-Math.exp(-t/.48)),-15,15),halfWidth:o.halfWidth+Math.min(.30,t*.08),halfLength:o.halfLength+Math.min(1.2,t*.25),speed};
  }
  responses(o,t,line){
    const hold=this.predict(o,t);
    const blend=1-Math.exp(-Math.max(0,t-.35)/1.8);
    const racingLateral=hold.lateral+((line.offsetAt?.(hold.s)??line.at(hold.s).offset)-hold.lateral)*blend;
    // Independent hypotheses, not access to another driver's chosen controls.
    // Early braking begins after reaction time and cannot move a stopped car backwards.
    const memory=this.history.get(o.id),deceleration=memory?.brakeStrength??3;
    const brakingTime=Math.min(Math.max(0,t-.6),hold.speed/deceleration);
    const lost=deceleration*brakingTime*(Math.max(0,t-.6)-brakingTime*.5);
    const weights=memory?.weights??[.6,.25,.15,0];
    const branch=(s,distance,lateral,speed,probability,behaviour)=>({s,distance,lateral,speed,halfWidth:hold.halfWidth,halfLength:hold.halfLength,probability,behaviour});
    const defendTarget=o.distance>0&&o.distance<40?clamp(o.pursuerLateral??hold.lateral,-4.65,4.65):memory?.preferredLane??hold.lateral;
    return [branch(hold.s,hold.distance,hold.lateral,hold.speed,weights[0],'HOLD'),
      branch(hold.s,hold.distance,racingLateral,hold.speed,weights[1],'RACING LINE'),
      branch(hold.s-lost,hold.distance-lost,hold.lateral,Math.max(0,hold.speed-deceleration*brakingTime),weights[2],'EARLY BRAKE'),
      branch(hold.s,hold.distance,hold.lateral+(defendTarget-hold.lateral)*blend,hold.speed,weights[3],'DEFEND')];
  }
}
