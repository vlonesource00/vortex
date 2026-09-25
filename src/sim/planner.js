import { clamp, angle } from './math.js';
import { Perception, relativeDistance } from './perception.js';
import { PerformanceModel, speedProfile } from './performance.js';
import { RaceStrategy } from './strategy.js';
import { packExitCost } from './pack-racing.js';
import { raceAwareness } from './awareness.js';
import { raceOffsetLimit, gentleRub, needsFollowing } from './racecraft-policy.js';
import { flowOpportunity } from './race-flow.js';
import { pathCurvature } from './path-geometry.js';
import { attackWindow, simultaneousBattle } from './maneuver.js';

function polynomial(start,slope,end,length){
  // Quintic Hermite boundary: y(0), y'(0), y''(0)=0, y(1), y'(1)=y''(1)=0.
  const b=slope*length,d=end-start;
  return [start,b,0,10*d-6*b,-15*d+8*b,6*d-3*b];
}
function evaluate(c,t){return c[0]+t*(c[1]+t*(c[2]+t*(c[3]+t*(c[4]+t*c[5]))));}

export class Trajectory {
  constructor(line,startS,length,startOffset,slope,endExtra){
    this.line=line;this.startS=startS;this.length=length;this.endExtra=endExtra;
    this.coefficients=polynomial(startOffset,slope,endExtra,length);
    this.points=[];this.score=Infinity;this.hardConflict=false;this.minClearance=99;
    this.exitExtra=endExtra;this.exitLength=95;this.manoeuvre='HOLD CORRIDOR';
  }
  at(s){
    const distance=relativeDistance(s,this.startS,this.line.track.length);
    const extra=distance<=this.length?evaluate(this.coefficients,clamp(distance/this.length,0,1)):
      evaluate(polynomial(this.endExtra,0,this.exitExtra,this.exitLength),clamp((distance-this.length)/this.exitLength,0,1));
    const limit=raceOffsetLimit(this.line.track);
    const base=this.line.at(s),offset=clamp(base.offset+extra,-limit,limit);
    return {x:base.x+base.nx*(offset-base.offset),z:base.z+base.nz*(offset-base.offset),y:base.y,
      tx:base.tx,tz:base.tz,nx:base.nx,nz:base.nz,heading:base.heading,curvature:base.curvature,
      s:base.s,index:base.index,speed:base.speed,conservativeSpeed:base.conservativeSpeed,offset};
  }
  curvatureAt(s){
    if(!this.points.length)return this.line.at(s).curvature;
    const distance=relativeDistance(s,this.startS,this.line.track.length);
    let lo=0,hi=this.points.length-1;
    while(lo<hi){const mid=(lo+hi)>>1;if(this.points[mid].distance<distance)lo=mid+1;else hi=mid;}
    const b=this.points[lo],a=this.points[Math.max(0,lo-1)];
    const t=clamp((distance-a.distance)/Math.max(.001,b.distance-a.distance),0,1);
    return a.curvature+(b.curvature-a.curvature)*t;
  }
  pathDistanceAt(distance){
    if(this.points[0]?.travelDistance===undefined)return distance;
    let a={distance:0,travelDistance:0};
    for(const b of this.points){if(b.distance>=distance){const t=clamp((distance-a.distance)/Math.max(.001,b.distance-a.distance),0,1);return a.travelDistance+(b.travelDistance-a.travelDistance)*t;}a=b;}
    return a.travelDistance+Math.max(0,distance-a.distance);
  }
}

export class TacticalPlanner {
  constructor(line,model=new PerformanceModel(line.track),strategy=new RaceStrategy(0)){
    this.line=line;this.perception=new Perception(line.track);this.plan=null;this.candidates=[];this.observation=null;
    this.timer=0;this.preferred=0;this.commit=0;this.commitSide=0;this.switchTimer=0;
    this.intent='PACE';this.targetId=null;this.state='PACE';
    this.reason='Minimum-time racing line';this.stats={};this.age=0;
    this.model=model;this.strategy=strategy;
    this.sequence=null;
  }
  update(car,cars,dt,context=null){
    this.age+=dt;this.timer-=dt;this.commit=Math.max(0,this.commit-dt);this.switchTimer=Math.max(0,this.switchTimer-dt);
    if(this.timer>0&&this.plan)return this.plan;
    // Distribute initial deadlines across physics ticks; subsequent solves
    // keep the same cadence. No car waits for another car's preferred route.
    this.timer=.08+(this.line.track.id==='harbor-ring'&&!this.plan?(Number(car.id)%8)/120:0);
    const started=performance.now();
    const obs=this.perception.scan(car,cars,this.age-(this.lastScan??0),this.line,context?.projections);this.observation=obs;this.lastScan=this.age;
    this.strategy.update(car,obs,this.line,context,this.model);
    if(!this.strategy.allowDefend&&this.intent==='DEFEND'){this.commit=0;this.targetId=null;this.intent='PACE';this.commitSide=0;}
    // Every candidate in a horizon samples the same immutable base geometry.
    // Restrict caching to this solve so a selected path cannot accumulate entries.
    const geometry=new Map();let caching=true;
    const planningLine={track:this.line.track,at:s=>{
      if(!caching)return this.line.at(s);
      let point=geometry.get(s);if(!point){point=this.line.at(s);geometry.set(s,point);}return point;
    }};
    const base=this.line.at(obs.origin.s),future=this.line.at(obs.origin.s+2);
    const baselineSlope=(future.offset-base.offset)/2;
    const currentExtra=obs.origin.lateral-base.offset;
    const traffic=obs.observations.some(o=>o.distance>-32&&(o.distance<100||
      (car.speed-o.speed>5&&o.distance<car.speed*car.speed/12+12)));
    this.awareness=raceAwareness(car,obs);
    this.model.setTraffic(traffic,.08);
    const launch=this.age<4;
    const ahead=obs.observations.filter(o=>o.distance>0).sort((a,b)=>a.distance-b.distance)[0];
    const behind=obs.observations.filter(o=>o.distance<0).sort((a,b)=>b.distance-a.distance)[0];
    const opportunity=flowOpportunity(car,obs);
    const brakingAttack=this.line.track.id==='harbor-ring'?attackWindow(car,obs,this.line,this.model):null;
    const settledAlongside=new Set(obs.observations.filter(other=>Math.abs(other.distance)<other.halfLength+2.3&&gentleRub(
      Math.max(0,other.halfWidth+.98-Math.abs(other.lateral-obs.origin.lateral)),
      Math.abs(car.speed-other.speed),Math.abs(obs.lateralSpeed-other.lateralSpeed))).map(other=>other.id));
    const overlap=obs.observations.find(o=>Math.abs(o.distance)<o.halfLength+2.3&&Math.abs(o.lateral-obs.origin.lateral)>1&&Math.abs(o.lateral-obs.origin.lateral)<4.2);
    const renewWidth=this.commit<=0;
    const laneClear=(offset)=>obs.lanes.reduce((best,l)=>Math.abs(l.lateral-offset)<Math.abs(best.lateral-offset)?l:best,obs.lanes[0])?.clearance??0;
    const chooseSide=(other,defend=false)=>{
      const delta=other?other.lateral-obs.origin.lateral:0;
      let side=Math.abs(delta)>1.1?(defend?Math.sign(delta):-Math.sign(delta)):laneClear(base.offset-3.2)>=laneClear(base.offset+3.2)?-1:1;
      const chosen=laneClear(base.offset+side*3.2),alternate=laneClear(base.offset-side*3.2);
      if(chosen<5&&alternate>chosen+3)side*=-1;
      return side||1;
    };
    // The tactical layer has a short-lived commitment.  It is deliberately
    // separate from the geometric planner so a car can hold an attack or a
    // defensive wall through several 80 ms replans without lane oscillation.
    if(launch){
      this.intent='PACE';this.targetId=null;this.commitSide=0;this.commit=0;
    }else if(overlap){
      if(this.intent!=='SIDE_BY_SIDE'||this.targetId!==overlap.id){
        this.intent='SIDE_BY_SIDE';this.targetId=overlap.id;
        // Existing physical overlap takes priority over an earlier attack side.
        this.commitSide=Math.sign(obs.origin.lateral-overlap.lateral)||this.commitSide||chooseSide(overlap);
      }
      this.commit=Math.max(this.commit,.9);
    }else if(opportunity&&this.strategy.allowAttack){
      // A lost-momentum car ahead overrides a rear-focused commitment. Keep
      // an established passing side; do not flip it on every braking sample.
      if(this.intent!=='ATTACK'||this.targetId!==opportunity.id){
        this.commitSide=chooseSide(opportunity,false);this.sequence=null;this.switchTimer=0;
      }
      this.intent='ATTACK';this.targetId=opportunity.id;this.commit=Math.max(this.commit,.8);
    }else if(this.commit>0&&this.targetId!==null){
      const tracked=obs.observations.find(o=>o.id===this.targetId);
      if(!tracked||Math.abs(tracked.distance)>115){this.commit=0;this.intent='PACE';this.targetId=null;this.commitSide=0;}
    }else if(this.strategy.allowAttack&&ahead&&ahead.distance<52&&(car.speed-ahead.speed>-.5||ahead.distance<16)){
      this.intent='ATTACK';this.targetId=ahead.id;this.commitSide=chooseSide(ahead,false);this.commit=1.8;
    }else if(this.strategy.allowDefend&&behind&&behind.distance>-28&&(behind.speed>car.speed+.5||behind.distance>-12)&&(!ahead||ahead.distance>14)){
      this.intent='DEFEND';this.targetId=behind.id;this.commitSide=chooseSide(behind,true);this.commit=1.35;
    }else{
      this.intent='PACE';this.targetId=null;this.commitSide=0;
    }
    if(this.sequence&&this.sequence.targetId===this.targetId&&!overlap&&relativeDistance(this.sequence.exitS,obs.origin.s,this.line.track.length)<=22){
      this.preferred=this.sequence.exitExtra;this.switchTimer=.9;
      this.commitSide=Math.sign(this.sequence.exitExtra);this.commit=.9;
      this.intent=Math.abs(this.sequence.exitExtra)<.1?'EXIT':'ATTACK';
      this.sequence=null;
    }
    // Freeze the chosen width for the commitment, just like its side. A gap
    // crossing the pressure threshold must not move the aim point every solve.
    if(renewWidth||!this.widthCommit||this.widthCommit.intent!==this.intent||this.widthCommit.target!==this.targetId||this.widthCommit.side!==this.commitSide){
      const economical=(this.intent==='DEFEND'||this.intent==='ATTACK')&&ahead&&ahead.distance<55&&behind&&behind.distance< -10&&behind.distance> -32;
      this.widthCommit={intent:this.intent,target:this.targetId,side:this.commitSide,width:economical?2.4:3.55};
    }
    const offsets=launch?[currentExtra]:traffic?[-5.2,-3.9,-2.6,-1.3,0,1.3,2.6,3.9,5.2]:[0];
    const friction=this.model.at(car.speed,obs.origin.s,obs.origin.lateral).mu;
    const candidates=[];
    const rapidPass=(opportunity||brakingAttack)&&!overlap;
    for(const horizon of (rapidPass?[1.1,2.0,2.8,3.6]:traffic?[2.0,2.8,3.6]:[2.8]))for(const endExtra of offsets){
      let length=clamp(car.speed*horizon+18,35,145);
      // Anchor a selected exit in track coordinates. Without this, repeated
      // replanning would keep pushing the switchback further into the future.
      if(this.sequence&&this.sequence.targetId===this.targetId&&Math.abs(endExtra-this.sequence.endExtra)<.1){
        const remaining=relativeDistance(this.sequence.exitS,obs.origin.s,this.line.track.length);
        if(remaining>20)length=Math.min(length,remaining);
      }
      // Preserve the previous geometric path over replans. Reinitializing every
      // 80 ms at the measured lateral velocity perpetually postpones turn-in.
      let previousExtra=this.plan?this.plan.at(obs.origin.s).offset-base.offset:currentExtra;
      let plannedSlope=this.plan?(this.plan.at(obs.origin.s+2).offset-this.plan.at(obs.origin.s).offset)/2-baselineSlope:0;
      if(this.line.track.id==='harbor-ring'&&Math.abs(previousExtra-currentExtra)>2.5){
        // Recover a missed path from measured motion, while retaining its exit
        // commitment. Do not repeatedly restart an already reachable manoeuvre.
        previousExtra=previousExtra*.35+currentExtra*.65;
        plannedSlope=plannedSlope*.35+(obs.lateralSpeed/Math.max(8,car.speed)-baselineSlope)*.65;
      }
      const transitioning=traffic||launch||Math.abs(previousExtra)>.15;
      const plan=new Trajectory(planningLine,obs.origin.s,length,transitioning?previousExtra:0,transitioning?clamp(plannedSlope,-.10,.10):0,endExtra);
      plan.horizon=horizon;plan.trafficCost=0;plan.contactCost=0;plan.gripCost=0;plan.edgeCost=0;plan.intentCost=0;plan.progress=0;
      let time=0,speed=Math.max(7,car.speed),previous=plan.at(obs.origin.s),lastLat=previous.offset;
      for(let i=1;i<=24;i++){
        const ds=length/24,distance=ds*i,p=plan.at(obs.origin.s+distance);
        const a=plan.at(obs.origin.s+distance-2),b=plan.at(obs.origin.s+distance+2);
        const headingA=Math.atan2(p.x-a.x,p.z-a.z),headingB=Math.atan2(b.x-p.x,b.z-p.z);
        const curvature=this.line.track.id==='harbor-ring'?pathCurvature(a,p,b):angle(headingB-headingA)/2;
        const envelope=this.model.at(speed,p.s,p.offset);
        const lateralCapacity=envelope.lateral;
        const curveSpeed=Math.sqrt(Math.min(lateralCapacity,envelope.corner*envelope.pace**2)/Math.max(.0008,Math.abs(curvature)));
        const targetSpeed=Math.min(this.model.lineSpeed(p)*envelope.pace,curveSpeed,78);
        const physicalStep=this.line.track.id==='harbor-ring'?Math.hypot(p.x-previous.x,p.z-previous.z):ds;
        speed=clamp(targetSpeed,Math.sqrt(Math.max(9,speed*speed-2*8.5*physicalStep)),Math.sqrt(speed*speed+2*5*physicalStep));
        time+=physicalStep/Math.max(4,speed);
        const demand=speed*speed*Math.abs(curvature)/Math.max(1,lateralCapacity);
        plan.gripCost+=Math.max(0,demand-1)**2*85;
        plan.edgeCost+=Math.max(0,Math.abs(p.offset)-(this.line.track.halfWidth-1.7))**2*2;
        const latRate=(p.offset-lastLat)/Math.max(.05,ds/Math.max(5,speed));
        let trafficSpeed=targetSpeed;
        for(const other of obs.observations){
          const prediction=this.perception.predict(other,time);
          const gap=prediction.distance-distance;
          const lateralGap=Math.abs(prediction.lateral-p.offset);
          const separation=lateralGap-(prediction.halfWidth+.98);
          if(Math.abs(gap)<prediction.halfLength+2.3){
            plan.minClearance=Math.min(plan.minClearance,separation);
            if(separation<0){
              const closing=Math.abs(speed-prediction.speed),sideSpeed=Math.abs(latRate-other.lateralSpeed);
              const severity=(-separation)*14+closing*.9+sideSpeed*2.5;
              // Rubbing is an expensive but feasible option; large overlap or
              // energetic impact disqualifies the path. No ghosting or immunity.
              const physicalOverlap=other.halfWidth+.98-lateralGap;
              if(physicalOverlap>0&&!gentleRub(physicalOverlap,closing,sideSpeed)){
                // Uncertainty padding is not car bodywork; actual predicted
                // body conflicts still exclude a path across its horizon.
                plan.contactCost+=12000+severity*1200;plan.hardConflict=true;
              }
              else plan.contactCost+=8+severity*12;
            }else plan.trafficCost+=Math.max(0,.35-separation)*4;
          }
          if(needsFollowing({gap,lateralGap,halfWidth:other.halfWidth,halfLength:other.halfLength,
            closing:Math.abs(speed-prediction.speed),sideSpeed:Math.abs(latRate-other.lateralSpeed),alongside:settledAlongside.has(other.id)})){
            const safe=5+Math.max(0,speed-prediction.speed)**2/16;
            plan.trafficCost+=Math.max(0,safe+4-gap)*4;
            const followingSpeed=Math.max(0,prediction.speed+(gap-5)*.9);
            trafficSpeed=Math.min(trafficSpeed,followingSpeed);
            speed=Math.min(speed,Math.max(3,followingSpeed));
          }
        }
        plan.points.push({...p,distance,time,speed,speedLimit:targetSpeed,trafficSpeed,curvature,demand});
        plan.progress+=speed;lastLat=p.offset;previous=p;
      }
      if(this.commit>0&&this.commitSide!==0){
        // A pack battle needs room to pass without automatically spending the
        // whole track width. Keep the same side; overlap still takes priority.
        const committedExtra=this.commitSide*this.widthCommit.width;
        // Keep a committed side through the corner, but let a physically
        // unsafe candidate escape the commitment through its hardConflict.
        plan.intentCost=Math.abs(endExtra-committedExtra)*(this.intent==='SIDE_BY_SIDE'?3.2:1.45);
        if(this.intent==='DEFEND')plan.intentCost+=Math.max(0,2.5-Math.abs(endExtra-committedExtra))*1.1;
      }
      const switching=Math.abs(endExtra-this.preferred)*(this.switchTimer>0?2.0:.45);
      // Minimize travel time + physical risk, with a small bias to global line.
      plan.score=time*12/horizon+plan.gripCost+plan.edgeCost+plan.contactCost+plan.trafficCost+plan.intentCost+switching+Math.abs(endExtra)*.16;
      candidates.push(plan);
    }
    // The immediate corridor has alternative exit continuations. Evaluate the
    // same downstream distance for every candidate so shorter horizons cannot
    // win merely by omitting the cost of the next corner.
    const continuationDistance=clamp(car.speed*6.5+30,130,270);
    const seeds=candidates.filter(p=>!p.hardConflict).sort((a,b)=>a.score-b.score).slice(0,3);
    for(const seed of seeds)for(const exitExtra of [0,-seed.endExtra*.65]) {
      if(Math.abs(exitExtra-seed.endExtra)<.5)continue;
      const p=new Trajectory(planningLine,seed.startS,seed.length,seed.coefficients[0],seed.coefficients[1]/seed.length,seed.endExtra);
      Object.assign(p,{horizon:seed.horizon,exitExtra,manoeuvre:Math.abs(exitExtra)<.1?'PASS THEN EXIT':'SWITCHBACK',
        points:seed.points.map(p=>({...p})),score:seed.score,hardConflict:seed.hardConflict,minClearance:seed.minClearance,
        contactCost:seed.contactCost,intentCost:seed.intentCost,trafficCost:seed.trafficCost,gripCost:seed.gripCost});
      candidates.push(p);
    }
    for(const plan of candidates) {
      const nearLength=plan.length;
      for(let distance=nearLength+10;distance<=continuationDistance;distance+=10) {
        const p=plan.at(obs.origin.s+distance),a=plan.at(obs.origin.s+distance-2),b=plan.at(obs.origin.s+distance+2);
        const curvature=this.line.track.id==='harbor-ring'?pathCurvature(a,p,b):angle(Math.atan2(b.x-p.x,b.z-p.z)-Math.atan2(p.x-a.x,p.z-a.z))/2;
        const e=this.model.at(car.speed,p.s,p.offset);
        const speedLimit=Math.min(this.model.lineSpeed(p)*e.pace,Math.sqrt(Math.min(e.lateral,e.corner*e.pace**2)/Math.max(.0008,Math.abs(curvature))),78);
        plan.points.push({...p,distance,curvature,speedLimit,trafficSpeed:speedLimit});
      }
      // Distant slow traffic must constrain the continuation too; otherwise a
      // stopped car can be visible yet outside the controller's braking plan.
      for(const p of plan.points)if(p.distance>nearLength){
        const arrival=p.distance/Math.max(10,car.speed);
        for(const other of obs.observations){
          if(other.speed>=5)continue;
          const predicted=this.perception.predict(other,arrival),gap=predicted.distance-p.distance;
          if(gap>0&&gap<40&&Math.abs(predicted.lateral-p.offset)<2.15)
            p.trafficSpeed=Math.min(p.trafficSpeed,Math.max(0,predicted.speed+(gap-6)*.65));
        }
      }
      if(this.line.track.id==='harbor-ring'){
        let previous=plan.at(obs.origin.s),distance=0;
        for(const p of plan.points){distance+=Math.hypot(p.x-previous.x,p.z-previous.z);p.travelDistance=distance;previous=p;}
      }
      const travelTime=speedProfile(plan.points,car,this.model);
      let responseRisk=0;
      for(const p of plan.points)for(const other of obs.observations) {
        for(const predicted of this.perception.responses(other,p.time,this.line)) {
          const gap=predicted.distance-p.distance,clearance=Math.abs(predicted.lateral-p.offset)-predicted.halfWidth-.98;
          if(Math.abs(gap)<predicted.halfLength+2.3&&clearance<0) {
            const probability=predicted.probability;
            responseRisk+=probability*Math.min(12,-clearance*4+Math.abs(p.speed-predicted.speed)*.25);
            // Only imminent high-confidence conflicts are hard exclusions;
            // uncertain downstream reactions carry a graded cost.
            const physicalOverlap=other.halfWidth+.98-Math.abs(predicted.lateral-p.offset);
            if(predicted.behaviour==='HOLD'&&p.time<2&&physicalOverlap>0&&!gentleRub(physicalOverlap,Math.abs(p.speed-predicted.speed),Math.abs(other.lateralSpeed-obs.lateralSpeed)))plan.hardConflict=true;
          }
        }
      }
      const exit=plan.points.at(-1);
      plan.responseRisk=responseRisk;plan.travelTime=travelTime;plan.exitSpeed=exit.speed;
      plan.exitAdvantage=0;
      const target=obs.observations.find(o=>o.id===this.targetId);
      if(target)plan.exitAdvantage=exit.distance-this.perception.predict(target,exit.time).distance;
      plan.score+=responseRisk*(1.3-this.strategy.aggression*.45)
        +travelTime*.7-exit.speed*.035-this.strategy.attackValue*clamp(plan.exitAdvantage,-12,12)*.025;
      plan.pack=packExitCost(car,obs.observations,exit,(other,time)=>this.perception.predict(other,time),this.strategy.attackValue);
      if(this.line.continuation){
        const previous=plan.points.at(-2);
        plan.terminalCost=this.line.continuation.cost(exit.s,exit.offset,(exit.offset-previous.offset)/Math.max(1,exit.distance-previous.distance));
        plan.score+=plan.terminalCost;
        plan.battle=simultaneousBattle(car,obs.observations,plan,(other,time)=>this.perception.predict(other,time));
        plan.pack.frontCost+=plan.battle.frontCost;plan.pack.rearCost+=plan.battle.rearCost;
        plan.pack.total=plan.pack.frontCost+plan.pack.rearCost;
      }
      plan.score+=plan.pack.total;
    }
    candidates.sort((a,b)=>Number(a.hardConflict)-Number(b.hardConflict)||a.score-b.score);
    caching=false;geometry.clear();
    this.plan=candidates[0];this.candidates=candidates;
    if(this.plan.manoeuvre!=='HOLD CORRIDOR'&&this.targetId!==null){
      if(!this.sequence||this.sequence.targetId!==this.targetId||this.sequence.manoeuvre!==this.plan.manoeuvre||Math.abs(this.sequence.endExtra-this.plan.endExtra)>.1)
        this.sequence={targetId:this.targetId,manoeuvre:this.plan.manoeuvre,endExtra:this.plan.endExtra,exitExtra:this.plan.exitExtra,exitS:obs.origin.s+this.plan.length};
    }else this.sequence=null;
    if(Math.abs(this.plan.endExtra-this.preferred)>.8){this.preferred=this.plan.endExtra;this.switchTimer=1.25;}
    this.plan.intent=this.intent;this.plan.targetId=this.targetId;this.plan.commitSide=this.commitSide;this.plan.commit=this.commit;
    this.state=Math.abs(obs.origin.lateral)>7?'REJOIN':this.plan.hardConflict?'YIELD':this.intent==='SIDE_BY_SIDE'?'SIDE BY SIDE':this.intent==='ATTACK'?'OVERTAKE':this.intent==='DEFEND'?'DEFEND':ahead&&ahead.distance<32?'DRAFT':'PACE';
    const targetName=obs.observations.find(o=>o.id===this.targetId)?.name;
    const sideName=this.commitSide<0?'left':'right';
    this.reason=this.state==='SIDE BY SIDE'?`Hold ${sideName} corridor; leave racing room through exit`:this.state==='OVERTAKE'?`Attack ${targetName||'the car ahead'} on the ${sideName}; reassess after commitment`:this.state==='DEFEND'?`Cover ${sideName} against ${targetName||'the car behind'}; hold chosen line`:this.state==='YIELD'?'Every candidate predicts conflict; least-risk path with traffic speed reduction':this.state==='DRAFT'?'Use tow; preserve braking margin':this.state==='REJOIN'?'Reduce speed and rejoin with traffic prediction':'Minimum-time line within live friction envelope';
    this.stats={candidates:candidates.length,rejected:candidates.filter(c=>c.hardConflict).length,ms:performance.now()-started,friction,clearance:this.plan.minClearance,gripPeak:Math.max(...this.plan.points.map(p=>p.demand)),intent:this.intent,targetId:this.targetId,side:this.commitSide,commit:this.commit};
    this.stats.flowTarget=opportunity?.name??null;
    if(opportunity&&!overlap)this.reason=`Lost momentum ahead: evaluate a quicker pass around ${opportunity.name}; retain front and rear clearance checks`;
    Object.assign(this.stats,{manoeuvre:this.plan.manoeuvre,exitSpeed:this.plan.exitSpeed,exitAdvantage:this.plan.exitAdvantage,responseRisk:this.plan.responseRisk,strategy:this.strategy.mode,aggression:this.strategy.aggression});
    Object.assign(this.stats,{frontTradeoff:this.plan.pack?.frontCost??0,rearExposure:this.plan.pack?.rearCost??0});
    return this.plan;
  }
}
