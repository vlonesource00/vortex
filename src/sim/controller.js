import { TacticalPlanner } from './planner.js';
import { clamp, damp, angle } from './math.js';
import { SPEC } from './vehicle.js';
import { PerformanceModel, predictChassis } from './performance.js';
import { RaceStrategy } from './strategy.js';
import { supervise } from './supervisor.js';
import { PACE } from './pace.js';
import { pedals, pedalAcceleration } from './driver-controls.js';
import { relativeDistance } from './perception.js';
import { brakingTarget } from './speed-target.js';
import { HARBOR_CONTROL } from './race-policy.js';
import { pathCurvature } from './path-geometry.js';

export function rejoinClear(car,cars,track,mergeS) {
  const horizon=clamp(Math.abs(car.lateral)/2+1,2,5);
  for(const other of cars){
    if(other===car)continue;
    if(Math.hypot(other.x-car.x,other.z-car.z)<7)return false;
    const p=track.nearest(other.x,other.z),distance=relativeDistance(p.s,mergeS,track.length);
    const velocity=other.vx*p.tx+other.vz*p.tz;
    const end=distance+velocity*horizon;
    if(Math.min(distance,end)<10&&Math.max(distance,end)>-10&&Math.abs(p.lateral)<7.5)return false;
  }
  return true;
}

export class AdaptiveDriver {
  constructor(id,line,skill=.97,aggression=.72){
    this.id=id;this.line=line;this.skill=skill;this.model=new PerformanceModel(line.track);this.strategy=new RaceStrategy(id,aggression);
    this.line=line;
    this.planner=new TacticalPlanner(this.line,this.model,this.strategy);
    this.state='PACE';this.targetSpeed=0;this.steer=0;this.recovery=0;this.controlTimer=0;
    this.modelGain=1;this.debug={rollouts:[],selectedRollout:0};this.reverseTimer=0;
    this.captureRollouts=false;this.rolloutPool=Array.from({length:15},()=>({points:[],cost:0,accelerationBias:0}));
    this.age=0;this.forecasts=[];this.predictionError={samples:0,position:0,speed:0};
  }
  update(car,cars,dt,context=null){
    const SPEC=car.spec;
    this.age+=dt;
    while(this.forecasts.length&&this.forecasts[0].time<=this.age){
      const p=this.forecasts.shift(),extra=this.age-p.time;
      const position=Math.hypot(p.x+p.vx*extra-car.x,p.z+p.vz*extra-car.z);
      const speed=Math.abs(p.speed-car.speed),e=this.predictionError;
      e.position+=(position-e.position)*.1;e.speed+=(speed-e.speed)*.1;e.samples++;
    }
    this.model.update(car,dt);
    const current=context?.projections.get(car.id)??this.line.track.nearest(car.x,car.z);
    const headingError=angle(current.heading-car.yaw);
    const edge=this.line.track.halfWidth;
    if((Math.abs(current.lateral)>edge+.7&&car.speed<16)||Math.abs(headingError)>1.3||this.reverseTimer>0||
      (this.wasRecovering&&(Math.abs(current.lateral)>edge-.7||Math.abs(headingError)>.6))){
      if(!this.wasRecovering){
        this.forecasts.length=0;
        this.planner.plan=null;this.planner.sequence=null;this.planner.commit=0;this.planner.targetId=null;
        this.planner.preferred=0;this.planner.timer=0;this.planner.intent='PACE';
      }
      this.wasRecovering=true;
      this.recover(car,cars,current,dt);return;
    }
    if(this.wasRecovering){this.controlTimer=0;this.predictiveCorrection=0;this.steer=car.controls.steer;}
    this.wasRecovering=false;
    const plan=this.planner.update(car,cars,dt,context);
    const freedom=this.model.paceBlend,steeringFreedom=this.model.controlBlend;
    const policy=this.line.track.id==='harbor-ring'?(this.line.controlPolicy??HARBOR_CONTROL):{...PACE,tracking:.7};
    const lookahead=clamp(5.5+car.speed*(.5+(policy.lookahead-.5)*steeringFreedom),8,35),target=plan.at(current.s+lookahead);
    const dx=target.x-car.x,dz=target.z-car.z;
    const lx=dx*Math.cos(car.yaw)-dz*Math.sin(car.yaw);
    const slip=Math.atan2(car.v,Math.max(4,car.u));
    const pathHere=plan.at(current.s);
    const trackingError=current.lateral-pathHere.offset;
    const trackingCorrection=clamp(-Math.atan2(trackingError*policy.tracking,Math.max(12,car.speed)),-.05,.05);
    // Sample ahead of the replanning seam; sampling behind it would mistake
    // the clamped start offset for a real curvature change during an overtake.
    const middle=plan.at(current.s+3),after=plan.at(current.s+6);
    const localCurvature=this.line.track.id==='harbor-ring'?pathCurvature(pathHere,middle,after):angle(Math.atan2(after.x-middle.x,after.z-middle.z)-Math.atan2(middle.x-pathHere.x,middle.z-pathHere.z))/3;
    const rotationGain=policy.rotation;
    const rotation=clamp((localCurvature*car.speed-car.yawRate)*SPEC.wheelbase/Math.max(8,car.speed)*rotationGain*steeringFreedom,-.035,.035);
    const pursuit=Math.atan2(2*SPEC.wheelbase*lx,Math.max(5,dx*dx+dz*dz))+slip*(.45+(policy.slipCompensation-.45)*steeringFreedom)+trackingCorrection+rotation;
    const envelope=this.model.at(car.speed,current.s,current.lateral);
    let targetSpeed=this.model.lineSpeed(this.line.at(current.s+car.speed*.18+3))*this.skill*envelope.pace;
    targetSpeed=brakingTarget(plan,current.s,this.line.track.length,this.skill,envelope.brake,targetSpeed);
    // Observed yaw response is diagnostic; dynamic rollouts use measured axle
    // forces, slip, steering and the live grip envelope directly.
    const expected=car.u*Math.tan(car.steering)/SPEC.wheelbase;
    if(Math.abs(expected)>.15&&Math.abs(car.yawRate)>.05&&Math.abs(slip)<.2){
      const observed=clamp(car.yawRate/expected,.55,1.35);this.modelGain=damp(this.modelGain,observed,.4,dt);
    }
    this.controlTimer-=dt;
    if(this.controlTimer<=0){
      this.controlTimer=.04;
      this.predictiveCorrection=this.trackMPC(car,plan,current,pursuit,targetSpeed);
    }
    const steer=clamp((pursuit+(this.predictiveCorrection||0))/SPEC.steeringLock,-1,1);
    this.steer=damp(this.steer,steer,11,dt);
    if(Math.abs(current.lateral)>edge+.8)targetSpeed=Math.min(targetSpeed,11);
    if(Math.abs(slip)>.20)targetSpeed=Math.min(targetSpeed,car.speed*(1-clamp((Math.abs(slip)-.2)*1.1,0,.45)));
    this.safety=supervise(car,cars,current,this.model);
    targetSpeed=Math.min(targetSpeed,this.safety.maxSpeed);
    this.targetSpeed=targetSpeed;this.state=this.planner.state;
    const error=targetSpeed-car.speed;
    const bias=car.speed>12?this.longitudinalBias||0:0;
    this.controlEnvelope=this.line.track.id==='harbor-ring'?
      {...envelope,throttleLimit:this.model.longitudinal(car.speed,current.s,current.lateral,car.yawRate/Math.max(4,car.speed)).throttleLimit}:envelope;
    car.controls={steer:this.steer,...pedals(error,freedom,this.controlEnvelope,bias)};
    if(this.safety.emergency){car.controls.throttle=0;car.controls.brake=1;this.state='COLLISION AVOIDANCE';}
    this.recovery=car.speed<1&&car.controls.throttle>.5?this.recovery+dt:0;
  }
  recover(car,cars,current,dt){
    const SPEC=car.spec,edge=this.line.track.halfWidth;
    const headingError=angle(current.heading-car.yaw);
    // Hysteresis prevents forward/reverse cancellation from chattering when a
    // crawling car straddles the asphalt/runoff boundary.
    const outside=this.recoveryOutside=Math.abs(current.lateral)>edge+.3||Boolean(this.recoveryOutside)&&Math.abs(current.lateral)>edge-.4;
    const target=this.line.track.at(current.s+(outside?6:12),clamp(current.lateral*.35,-3,3));
    const dx=target.x-car.x,dz=target.z-car.z;
    const lx=dx*Math.cos(car.yaw)-dz*Math.sin(car.yaw),lz=dx*Math.sin(car.yaw)+dz*Math.cos(car.yaw);
    this.reverseTimer=Math.max(0,this.reverseTimer-dt);
    this.recovery=car.speed<.7?this.recovery+dt:0;
    if((lz<0||this.recovery>1.7)&&car.speed<2&&this.reverseTimer===0){this.reverseTimer=2.0;this.recovery=0;}
    if(this.reverseTimer>0&&Math.abs(headingError)<.65)this.reverseTimer=0;
    const forwardOutward=(Math.sin(car.yaw)*current.nx+Math.cos(car.yaw)*current.nz)*Math.sign(current.lateral);
    // Do not reverse further into runoff when forward motion leads back in.
    if(outside&&forwardOutward<-.15)this.reverseTimer=0;
    const reverse=this.reverseTimer>0;
    const steer=reverse?-clamp(headingError/1.2,-.85,.85):clamp(Math.atan2(2*SPEC.wheelbase*lx,Math.max(12,dx*dx+dz*dz))/SPEC.steeringLock,-.85,.85);
    const safe=rejoinClear(car,cars,this.line.track,target.s);
    const recoverySpeed=reverse?2.8:outside?4.5:8;
    car.gear=1;car.controls={steer,throttle:safe&&car.speed<recoverySpeed?(outside?.25:.34):0,brake:!safe||car.speed>recoverySpeed+1?.65:0,reverse};
    this.state=reverse?'REVERSE RECOVERY':'SAFE REJOIN';this.targetSpeed=recoverySpeed;
    this.planner.reason=reverse?'Controlled reverse manoeuvre; no teleport or extra grip':'Rejoin after checking nearby traffic';
  }
  trackMPC(car,plan,current,nominal,targetSpeed=car.speed){
    const SPEC=car.spec,edge=this.line.track.halfWidth-1.3;
    const initialSpeed=Math.max(3,car.speed);
    const authority=Math.abs(current.lateral)>edge||Math.abs(Math.atan2(car.v,Math.max(4,car.u)))>.08?2:1;
    const corrections=[-.012,-.006,0,.006,.012].map(v=>v*authority);
    let best=Infinity,result=0,selected=0;const rollouts=this.rolloutPool;
    let candidateIndex=0;
    for(let j=0;j<corrections.length;j++)for(const accelerationBias of [0,-1.2,.7]){
      const correction=corrections[j];let cost=0,speed=initialSpeed,distance=0;
      const rollout=rollouts[candidateIndex++],points=rollout.points;
      if(!this.captureRollouts)points.length=0;
      const predicted={x:car.x,z:car.z,yaw:car.yaw,rate:car.yawRate,u:Math.max(3,car.u),v:car.v,steering:car.steering,
        frontForce:car.wheels[0].tyre.fy+car.wheels[1].tyre.fy,rearForce:car.wheels[2].tyre.fy+car.wheels[3].tyre.fy};
      let commandSteer=this.steer;
      for(let i=0;i<12;i++){
        const h=.055,t=(i+1)*h;
        const path=plan.at(current.s+distance),curvature=plan.curvatureAt?.(current.s+distance)??path.curvature;
        const e=this.model.longitudinal(speed,current.s+distance,path.offset,curvature);
        // Two segments: act now, then release the correction on corner exit.
        const release=i<6?1:.5;
        const controls=pedals(targetSpeed-speed,this.model.paceBlend,e,accelerationBias*release);
        const acceleration=pedalAcceleration(controls,e);
        for(let sub=0;sub<2;sub++){
          commandSteer=damp(commandSteer,clamp((nominal+correction*release)/SPEC.steeringLock,-1,1),11,h/2);
          predictChassis(predicted,commandSteer*SPEC.steeringLock,acceleration,e,SPEC.mass+car.fuel*.75,car.setup.brakeBias,h/2,SPEC);
        }
        speed=Math.hypot(predicted.u,predicted.v);distance+=speed*h;
        const target=plan.at(current.s+distance);
        const lateralError=(predicted.x-target.x)*target.nx+(predicted.z-target.z)*target.nz;
        cost+=lateralError*lateralError*(.4+t)+correction*correction*170
          +Math.max(0,Math.abs(lateralError+target.offset)-edge)**2*12
          +(speed-targetSpeed)**2*.003+accelerationBias*accelerationBias*.012
          +Math.max(0,(speed*Math.abs(predicted.rate)/e.lateral)**2+(acceleration/Math.max(1,e.brake))**2-1)**2*.2
          +Math.max(0,Math.abs(Math.atan2(predicted.v,predicted.u))-(.09+(PACE.slipTarget-.09)*this.model.paceBlend))**2*80;
        if(i===0)rollout.forecast={time:this.age+h,x:predicted.x,z:predicted.z,speed,
          vx:predicted.u*Math.sin(predicted.yaw)+predicted.v*Math.cos(predicted.yaw),vz:predicted.u*Math.cos(predicted.yaw)-predicted.v*Math.sin(predicted.yaw)};
        if(this.captureRollouts){const p=points[i]??(points[i]={y:.15});p.x=predicted.x;p.z=predicted.z;}
      }
      rollout.cost=cost;rollout.accelerationBias=accelerationBias;
      if(cost<best){best=cost;result=correction;selected=candidateIndex-1;}
    }
    this.longitudinalBias=rollouts[selected].accelerationBias;
    this.forecasts.push(rollouts[selected].forecast);
    this.debug={rollouts,selectedRollout:selected};return result;
  }
}
