import { clamp } from './math.js';

export function attackWindow(car,observation,line,model){
  const front=observation.observations.filter(o=>o.distance>4&&o.distance<45).sort((a,b)=>a.distance-b.distance)[0];
  if(!front)return null;
  const closing=car.speed-front.speed;if(closing<1)return null;
  const envelope=model.at(car.speed,observation.origin.s,observation.origin.lateral);
  const extraStop=Math.max(0,(car.speed**2-front.speed**2)/(2*Math.max(3,envelope.brake)));
  // A late move needs space to match speed if the overlap does not materialize.
  if(extraStop>front.distance-3||Math.abs(observation.lateralSpeed)>2)return null;
  const open=observation.lanes.some(l=>l.clearance>12&&Math.abs(l.lateral-front.lateral)>2);
  return open?{id:front.id,kind:Math.abs(line.at(observation.origin.s+30).curvature)>.003?'BRAKING ATTACK':'PULL OUT',closing}:null;
}

export function simultaneousBattle(car,observations,plan,predict){
  const exit=plan.points.at(-1);let front=0,rear=0;const pursuers=[];
  for(const o of observations){
    if(Math.abs(o.distance)>65)continue;
    const time=Math.min(2.5,exit.time),p=plan.points.reduce((a,b)=>Math.abs(b.time-time)<Math.abs(a.time-time)?b:a);
    const f=predict(o,time),gap=f.distance-p.distance,separation=Math.abs(f.lateral-p.offset);
    const reach=Math.min(2,Math.abs(o.lateralSpeed??0)*time+.25*time*time);
    if(o.distance>4&&gap>0&&gap<14&&separation<2.15&&o.speed<car.speed-2)
      front+=clamp((car.speed-o.speed)/12,0,1)*.35;
    if(o.distance<-4&&gap>-12&&gap<7&&f.speed>p.speed+1){
      const risk=clamp((3.5+reach-separation)/(3.5+reach),0,1)*clamp((f.speed-p.speed)/10,0,1)*.25;
      rear+=risk;pursuers.push(o.id);
    }
  }
  return {frontCost:front,rearCost:Math.min(.75,rear),pursuers};
}
