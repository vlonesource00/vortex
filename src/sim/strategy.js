import { clamp } from './math.js';
import { PACE } from './pace.js';

// A bounded race-context layer. Aggression changes opportunity value, never
// footprint dimensions, impact limits, vehicle power or emergency protection.
export class RaceStrategy {
  constructor(id,aggression=.72) {
    this.aggression=clamp(aggression,0,1);this.mode='PUSH';this.pressure=0;
    this.attackValue=1;this.allowAttack=true;this.allowDefend=true;
    // Small, deterministic preferences affect decisions, not available grip.
    this.patience=.85+(id%4)*.1;this.remainingLaps=Infinity;this.position=null;
  }
  update(car,obs,line,context=null,model=null) {
    const wear=car.wheels.reduce((s,w)=>s+w.tyre.wear,0)/4;
    const heat=car.wheels.reduce((s,w)=>s+w.tyre.core,0)/4;
    const ahead=obs.observations.filter(o=>o.distance>0).sort((a,b)=>a.distance-b.distance)[0];
    const behind=obs.observations.filter(o=>o.distance<0).sort((a,b)=>b.distance-a.distance)[0];
    this.position=context?context.order.findIndex(c=>c.id===car.id)+1:null;
    this.remainingLaps=context&&context.mode!=='practice'?Math.max(0,context.totalLaps-(car.race?.progress??0)/line.track.length):Infinity;
    this.yieldTo=behind&&Number.isFinite(behind.progress)&&behind.progress-(car.race?.progress??0)>line.track.length*.5?behind.id:null;
    const backmarker=ahead&&Number.isFinite(ahead.progress)&&(car.race?.progress??0)-ahead.progress>line.track.length*.5;
    const lastLap=this.remainingLaps<1;
    const projectedHeat=model?.predictedRearHeat??heat;
    this.pressure=behind?clamp((28+Math.max(0,behind.speed-car.speed)*2+behind.distance)/28,0,1):0;
    const preserve=wear>.55||heat>120||car.damage>.35||(!lastLap&&projectedHeat>115);
    this.mode=this.yieldTo!==null?'BLUE FLAG':preserve?'PRESERVE':lastLap&&this.position===1&&this.pressure<.15?'PROTECT LEAD':ahead&&ahead.distance<55?'BATTLE':'PUSH';
    this.allowDefend=this.yieldTo===null;
    this.allowAttack=this.yieldTo===null&&!(preserve&&ahead&&ahead.distance>20&&car.speed-ahead.speed<.5*this.patience&&!lastLap&&!backmarker);
    this.attackValue=(.6+this.aggression*.8)*(this.mode==='PRESERVE'?.55:1);
    if(lastLap&&this.position>1)this.attackValue*=1.25;
    if(this.mode==='PROTECT LEAD')this.attackValue*=.65;
    this.objective=context?.paceObjective??'race';
    if(model)model.thermalHorizon=this.objective==='qualifying'?0:lastLap?Math.min(PACE.thermalHorizon,this.remainingLaps*80):PACE.thermalHorizon;
    this.reason=this.mode==='BLUE FLAG'?'Hold a predictable corridor; do not defend against a lapping car':this.mode==='PROTECT LEAD'?'Protect the lead on the final lap':this.mode==='PRESERVE'?'Avoid a low-value attack; protect projected tyre temperature':this.mode==='BATTLE'?'Value exit speed and a sustainable pass':'Maximise clear-track pace';
    return this;
  }
}
