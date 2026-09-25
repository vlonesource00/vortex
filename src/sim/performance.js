import { clamp, damp } from './math.js';
import { SPEC as DEFAULT_SPEC } from './vehicle.js';
import { tyreGrip } from './tyre.js';
import { PACE } from './pace.js';
import { RACECRAFT } from './racecraft-policy.js';

// A conservative reduced model, calibrated only from this car's telemetry.
// No privileged grip, impulses, opponent controls or changes to Vehicle.step.
export class PerformanceModel {
  constructor(track) {
    this.track=track; this.brakeScale=1; this.samples=0; this.confidence=0;
    this.car=null; this.referenceGrip=1.34;
    this.paceBlend=0;this.controlBlend=0;this.traffic=false;this.thermalFreedom=1;
    this.predictedRearHeat=68;this.thermalLoad=0;this.rearPower=0;this.thermalHorizon=PACE.thermalHorizon;
  }
  setTraffic(traffic,dt){
    // Rear-tyre heat changes the balance before average four-tyre grip falls
    // far enough to expose it. Retain a rotation reserve during a long stint.
    const rearHeat=this.car?Math.max(this.car.wheels[2].tyre.core,this.car.wheels[3].tyre.core):68;
    this.traffic=traffic;this.thermalFreedom=clamp((125-Math.max(rearHeat,this.predictedRearHeat))/30,0,1);
    const target=Math.min(traffic?RACECRAFT.trafficBlend:1,this.thermalFreedom);
    this.paceBlend+=clamp(target-this.paceBlend,-dt*2,dt*.4);
    this.controlBlend+=clamp((traffic?.35:1)-this.controlBlend,-dt*2,dt*.4);
  }
  lineSpeed(point){const safe=point.conservativeSpeed??point.speed;return safe+(point.speed-safe)*this.paceBlend;}
  update(car,dt) {
    this.car=car;
    const rear=car.wheels.slice(2);
    this.rearPower=damp(this.rearPower,rear.reduce((s,w)=>s+w.tyre.slipPower,0)/2,.5,dt);
    // Integrate observed slip-energy input through the same thermal time scales
    // as the tyre model. This predicts heat; it never edits a tyre state.
    let core=Math.max(...rear.map(w=>w.tyre.core)),surface=Math.max(...rear.map(w=>w.tyre.surface));
    const load=Math.max(3300,rear.reduce((s,w)=>s+w.load,0)/2),rolling=load*car.speed*.012;
    for(let t=0;t<this.thermalHorizon;t+=2){
      const flow=(surface-core)*75;
      surface=clamp(surface+2*(this.rearPower*.55+rolling-(surface-24)*(23+car.speed*1.1)-flow)/6000,24,210);
      core=clamp(core+2*(flow+rolling*.3-(core-24)*4)/18000,24,170);
    }
    this.predictedRearHeat=core;this.thermalLoad=clamp((core-95)/30,0,1);
    // All queried wheel loads are equal in this reduced model. Cache the
    // thermal/pressure/wear contribution once per telemetry sample.
    this.tyreFactor=car.wheels.reduce((sum,w)=>sum+tyreGrip(w.tyre,3300),0)/4;
    this.frontFactor=(tyreGrip(car.wheels[0].tyre,3300)+tyreGrip(car.wheels[1].tyre,3300))/2;
    this.rearFactor=(tyreGrip(car.wheels[2].tyre,3300)+tyreGrip(car.wheels[3].tyre,3300))/2;
    const limits=this.at(car.speed,car.s,car.lateral);
    // Only identify braking on settled, straight asphalt, away from collisions.
    if(car.controls.brake>.85&&car.speed>15&&Math.abs(car.ay)<2&&car.zone==='asphalt'&&car.impact<.01&&car.ax< -3) {
      const observed=clamp(-car.ax/Math.max(1,limits.brake/this.brakeScale),.8,1.12);
      this.brakeScale=damp(this.brakeScale,observed,.25,dt);
      this.samples+=dt; this.confidence=1-Math.exp(-this.samples/6);
    }
  }
  at(speed,s,lateral=0) {
    const c=this.car;
    if(!c)return {lateral:10,brake:7.4,drive:4.8,mu:1.34,pace:1,corner:10,lateralFraction:.78};
    const SPEC=c.spec??DEFAULT_SPEC;
    const mass=SPEC.mass+c.fuel*.75,q=.5*1.225*speed*speed;
    const platform=clamp(1-Math.abs(c.pitch)*1.4,.65,1);
    const downforce=q*SPEC.area*(SPEC.cl+(c.setup.wing-6)*.11)*platform*(1-c.aero.wake*.32);
    const load=(mass*9.81+downforce)/4;
    const mu=this.tyreFactor*SPEC.tyreGrip*clamp(1-.13*Math.log(Math.max(.1,load/3300)),.68,1.18);
    const p=this.track.at(s),lane=this.track.laneAt(lateral);
    const rubber=this.track.rubber[p.index*13+lane];
    // Budget grip at both wheel tracks, including a small tracking allowance.
    // Centre-only sampling incorrectly promises asphalt grip with tyres on kerbs.
    const surfaceBase=offset=>({asphalt:1,kerb:.88,gravel:.52,grass:.42}[this.track.zoneAt(offset)]);
    const footprint=SPEC.track/2+.2;
    const base=(surfaceBase(lateral-footprint)+surfaceBase(lateral+footprint))*.5;
    const surface=(1+rubber*.1)*(1-this.track.wetness*(.36+rubber*.2))*base;
    const lateralFraction=.78+(PACE.lateralReserve-.78)*this.paceBlend;
    const balanceReserve=Math.min(this.frontFactor,this.rearFactor)/this.tyreFactor;
    const lateralCapacity=mu*surface*(9.81+downforce/mass)*lateralFraction*balanceReserve;
    const drag=q*SPEC.area*(SPEC.cd+(c.setup.wing-6)*.013)*(1-c.aero.wake*.24)*(1+c.damage*.2)/mass;
    let gear=1;
    while(gear<6&&speed/SPEC.radius*SPEC.gears[gear]*SPEC.finalDrive*9.5493>7450)gear++;
    const ratio=SPEC.gears[gear]*SPEC.finalDrive,rpm=Math.max(1100,speed/SPEC.radius*ratio*9.5493);
    const drive=SPEC.maxTorque*clamp(1-((rpm-5500)/6700)**2,.45,1)*ratio*.91*(1-c.damage*.28)/(SPEC.radius*mass)-drag-.13;
    const classCorner=SPEC.key==='gt'?1:SPEC.tyreGrip*Math.sqrt((9.81+downforce/mass)/(9.81+q*DEFAULT_SPEC.area*DEFAULT_SPEC.cl/(DEFAULT_SPEC.mass+c.fuel*.75)));
    return {coastWindow:this.track.id==='harbor-ring'?.4:0,mu:mu*surface,frontMu:mu*surface*this.frontFactor/this.tyreFactor,rearMu:mu*surface*this.rearFactor/this.tyreFactor,lateral:lateralCapacity,lateralFraction,drag:drag+.13,corner:(10+(PACE.corner-10)*this.paceBlend)*classCorner,
      brake:clamp((7.4+(PACE.brake-7.4)*this.paceBlend)*(mu/this.referenceGrip)*surface*this.brakeScale,4,9.2+(PACE.brakeMax-9.2)*this.paceBlend),
      drive:c.fuel>0?clamp(Math.min(drive,lateralCapacity*.48),.3,7):0,
      pace:clamp(Math.sqrt(mu/this.referenceGrip*surface*balanceReserve),.6,1.045)};
  }
  longitudinal(speed,s,lateral,curvature) {
    const e=this.at(speed,s,lateral);
    const utilisation=clamp(speed*speed*Math.abs(curvature)/Math.max(1,e.lateral),0,.98);
    const reserve=Math.sqrt(1-utilisation*utilisation);
    return {...e,brake:e.brake*reserve,drive:e.drive*reserve,utilisation,
      ...(this.track.id==='harbor-ring'?{throttleLimit:reserve,actuationDrive:e.drive}:{})};
  }
}

// Backward braking feasibility followed by forward acceleration feasibility.
// Paths and speed are coupled through their curvature and the friction ellipse.
export function speedProfile(points,car,model) {
  for(const p of points)p.speedLimit=Math.min(p.speedLimit,p.trafficSpeed??Infinity);
  for(let i=points.length-2;i>=0;i--) {
    const a=points[i],b=points[i+1],ds=Math.max(.01,(b.travelDistance??b.distance)-(a.travelDistance??a.distance));
    const e=model.longitudinal(b.speedLimit,b.s,b.offset,b.curvature);
    a.speedLimit=Math.min(a.speedLimit,Math.sqrt(b.speedLimit*b.speedLimit+2*e.brake*ds));
  }
  let speed=Math.max(.5,car.speed),distance=0,time=0;
  for(const p of points) {
    const ds=Math.max(.01,(p.travelDistance??p.distance)-distance),e=model.longitudinal(speed,p.s,p.offset,p.curvature);
    const previousSpeed=speed;
    const next=Math.min(p.speedLimit,Math.sqrt(speed*speed+2*e.drive*ds));
    // An infeasible initial state must not imply instantaneous deceleration.
    speed=Math.max(next,Math.sqrt(Math.max(.25,speed*speed-2*e.brake*ds)));
    time+=2*ds/Math.max(1,speed+previousSpeed);
    p.speed=speed;p.time=time;p.demand=speed*speed*Math.abs(p.curvature)/e.lateral;
    distance=p.travelDistance??p.distance;
  }
  return time;
}

// Two-axle dynamic bicycle model with lateral tyre relaxation, load transfer,
// combined slip and steering actuator lag. Used only for prediction.
export function predictChassis(state,steering,acceleration,envelope,mass,brakeBias,dt,SPEC=DEFAULT_SPEC) {
  const a=SPEC.wheelbase*(1-SPEC.frontWeight),b=SPEC.wheelbase*SPEC.frontWeight;
  state.steering=damp(state.steering,steering,12,dt);
  // Effective load follows the conservative grip envelope, including the
  // weaker-axle reserve; this predictor does not grant the full aero budget.
  const downforce=Math.max(0,envelope.lateral/(envelope.mu*(envelope.lateralFraction??PACE.lateralReserve))-9.81)*mass;
  const transfer=acceleration*mass*SPEC.cg/SPEC.wheelbase;
  const frontLoad=Math.max(100,mass*9.81*SPEC.frontWeight+downforce*SPEC.frontAero-transfer);
  const rearLoad=Math.max(100,mass*9.81*(1-SPEC.frontWeight)+downforce*(SPEC.key==='gt'?.57:1-SPEC.frontAero)+transfer);
  const fx=acceleration*mass,frontFx=acceleration<0?fx*brakeBias:SPEC.drive==='front'?fx:0,rearFx=fx-frontFx;
  const frontPeak=(envelope.frontMu??envelope.mu)*frontLoad,rearPeak=(envelope.rearMu??envelope.mu)*rearLoad;
  const frontCapacity=Math.sqrt(Math.max(0,frontPeak*frontPeak-frontFx*frontFx));
  const rearCapacity=Math.sqrt(Math.max(0,rearPeak*rearPeak-rearFx*rearFx));
  const frontSlip=Math.atan2(state.v+state.rate*a,Math.max(3,state.u))-state.steering;
  const rearSlip=Math.atan2(state.v-state.rate*b,Math.max(3,state.u));
  const relaxation=Math.max(3,state.u)/.45;
  state.frontForce=damp(state.frontForce,-frontCapacity*Math.tanh(8.6*Math.tan(clamp(frontSlip,-1,1))),relaxation,dt);
  state.rearForce=damp(state.rearForce,-rearCapacity*Math.tanh(8.6*Math.tan(clamp(rearSlip,-1,1))),relaxation,dt);
  const frontSide=state.frontForce*Math.cos(state.steering)+frontFx*Math.sin(state.steering);
  const lateralAcceleration=(frontSide+state.rearForce)/mass;
  state.v+=(lateralAcceleration-state.rate*state.u)*dt;
  state.u=Math.max(1,state.u+(acceleration+state.rate*state.v)*dt);
  state.rate+=((frontSide*a-state.rearForce*b-state.rate*130)/SPEC.yawInertia)*dt;
  state.yaw+=state.rate*dt;
  state.x+=(state.u*Math.sin(state.yaw)+state.v*Math.cos(state.yaw))*dt;
  state.z+=(state.u*Math.cos(state.yaw)-state.v*Math.sin(state.yaw))*dt;
  return state;
}
