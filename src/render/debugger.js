import * as THREE from 'three';

import { DebugLines } from './debug-lines.js';
import { battleView, battleAlternatives } from './battle-view.js';
import { RACECRAFT } from '../sim/racecraft-policy.js';
const colors={selected:'#b8ff64',candidate:'#6badcb',rejected:'#f05241',prediction:'#ffbc52',occupancy:'#dd9957',mpc:'#c1a1fa',braking:'#ff725e'};
export class AIDebugger {
  constructor(scene,session,onCamera){
    this.session=session;this.root=new THREE.Group();this.root.renderOrder=30;this.root.visible=false;scene.add(this.root);
    this.lines=new DebugLines(this.root);
    this.enabled=false;this.keepPaths=false;this.focus=0;this.last=0;this.frozen=false;
    this.fullPaths=true;
    this.panel=document.createElement('section');this.panel.id='ai-debug';this.panel.hidden=true;
    this.panel.innerHTML=`<div class="debug-header"><span>ASTRA / RACE ENGINEER</span><button id="debug-close" aria-label="Close AI debugger">×</button></div><div class="debug-select"><label>OBSERVE <select id="debug-car">${session.cars.map(c=>`<option value="${c.id}">${c.id===0?'07 · YOU':c.name}</option>`).join('')}</select></label><button id="debug-freeze">FREEZE PATHS</button></div><div id="debug-status"></div><div id="debug-metrics"></div><div class="debug-legend"><span style="--c:${colors.selected}">SELECTED</span><span style="--c:${colors.candidate}">CANDIDATE</span><span style="--c:${colors.rejected}">REJECTED</span><span style="--c:${colors.prediction}">PREDICTED CARS</span><span style="--c:${colors.mpc}">CONTROL ROLLOUT</span></div><p id="debug-reason"></p><div class="debug-foot">12.5 HZ PLANNING · 25 HZ MPC · 120 HZ CONTROL<br><kbd>B</kbd> TOGGLE DEBUGGER · <kbd>P</kbd> AI DEMO</div>`;
    document.querySelector('#app').append(this.panel);
    const metrics=this.panel.querySelector('#debug-metrics'),details=document.createElement('details');
    details.className='debug-details';details.innerHTML='<summary>Vehicle, pace & solver telemetry</summary>';
    metrics.before(details);details.append(metrics);
    const battle=document.createElement('div');battle.id='debug-battle';details.before(battle);
    this.panel.querySelector('.debug-legend').innerHTML=[['SELECTED',colors.selected],['FRONT', '#68d9ed'],['REAR','#ffbf71'],['ALONGSIDE','#ff7666'],['CANDIDATE',colors.candidate],['REJECTED',colors.rejected],['CONTROL ROLLOUT',colors.mpc]].map(([label,color])=>`<span style="--c:${color}">${label}</span>`).join('');
    const pathMode=document.createElement('button');pathMode.id='debug-path-mode';pathMode.textContent='SHOW BATTLE PATHS ONLY';pathMode.setAttribute('aria-pressed','true');
    this.panel.querySelector('.debug-select').append(pathMode);
    pathMode.onclick=()=>{this.fullPaths=!this.fullPaths;this.frozen=false;this.last=0;this.panel.querySelector('#debug-freeze').textContent='FREEZE PATHS';pathMode.textContent=this.fullPaths?'SHOW BATTLE PATHS ONLY':'SHOW ALL DEBUG PATHS';pathMode.setAttribute('aria-pressed',String(this.fullPaths));};
    this.battleOverlay=document.createElement('aside');this.battleOverlay.id='battle-overlay';this.battleOverlay.hidden=true;
    this.battleOverlay.innerHTML='<span class="attack">FRONT / ATTACK</span><span class="defence">REAR / DEFENCE</span><small>Green: selected · cyan dashed: front alternative · orange dotted: rear alternative</small>';
    document.querySelector('#app').append(this.battleOverlay);
    const cameraButton=document.createElement('button');cameraButton.className='debug-camera';cameraButton.textContent='TACTICAL / DRIVER CAMERA';cameraButton.onclick=onCamera;this.panel.querySelector('.debug-foot').prepend(cameraButton);
    this.panel.querySelector('#debug-close').onclick=()=>this.toggle(false);
    this.panel.querySelector('#debug-car').onchange=e=>{this.focus=Number(e.target.value);this.last=0;};
    this.panel.querySelector('#debug-freeze').onclick=e=>{this.frozen=!this.frozen;e.target.textContent=this.frozen?'LIVE PATHS':'FREEZE PATHS';};
  }
  get pathsEnabled(){return this.enabled||this.keepPaths;}
  toggle(value=!this.enabled){this.enabled=value;this.root.visible=this.pathsEnabled;this.panel.hidden=!value;this.battleOverlay.hidden=!this.pathsEnabled;return value;}
  clear(){this.lines.clear();}
  update(now){
    this.battleOverlay.hidden=!this.pathsEnabled;
    this.session.drivers.forEach((d,i)=>d.captureRollouts=this.pathsEnabled&&this.fullPaths&&!this.frozen&&i===this.focus);
    if(!this.pathsEnabled||now-this.last<120)return;this.last=now;if(!this.frozen)this.clear();
    const session=this.session,car=session.cars[this.focus],driver=session.drivers[this.focus],planner=driver.planner;
    const status=this.panel.querySelector('#debug-status');
    const suspend=message=>{status.textContent=message;this.clear();this.lines.flush();this.panel.querySelector('#debug-battle').textContent='';this.panel.querySelector('#debug-metrics').textContent='';this.panel.querySelector('#debug-reason').textContent=driver.wasRecovering?planner.reason:'';this.battleOverlay.querySelector('.attack').textContent=message;this.battleOverlay.querySelector('.defence').textContent='';this.battleOverlay.querySelector('small').textContent='Battle paths resume when tactical planning is active.';};
    if(!session.activeCars.includes(car)){suspend('DRIVER NOT IN THIS SESSION');return;}
    if(this.focus===0&&!session.autopilot){suspend('MANUAL CONTROL · PRESS P FOR AI');return;}
    if(!planner.plan){suspend(driver.wasRecovering?driver.state:'WAITING FOR GREEN');return;}
    status.innerHTML=`<b>${driver.state}</b><span>${car.name}</span>`;
    const selected=planner.plan;
    this.panel.querySelector('#debug-battle').innerHTML=battleView(car,planner);
    const alternatives=battleAlternatives(planner);
    const describe=(rival,path)=>!rival?'clear':`${Math.abs(rival.distance).toFixed(0)} m · ${rival.overlap?'alongside':path===selected?'shared path':path?'alternative shown':'no safe alternative'}`;
    this.battleOverlay.querySelector('.attack').textContent='FRONT / ATTACK · '+describe(planner.awareness?.front,alternatives.attack);
    this.battleOverlay.querySelector('.defence').textContent='REAR / DEFENCE · '+describe(planner.awareness?.rear,alternatives.defence);
    this.battleOverlay.querySelector('small').textContent=this.frozen?'PATHS FROZEN · telemetry remains live':'Green: selected · cyan dashed: front alternative · orange dotted: rear alternative';
    if(!this.frozen){
    for(const candidate of this.fullPaths?planner.candidates:[]){
      if(candidate===selected)continue;
      this.lines.add(candidate.points,candidate.hardConflict?colors.rejected:colors.candidate,candidate.hardConflict?.17:.23);
    }
    this.lines.add(selected.points,colors.selected);
    // Actual feasible candidates, not invented commands. Preserve separate dash
    // patterns when both objectives prefer the same alternative.
    for(const [path,color,stride,phase] of [[alternatives.attack,'#68d9ed',4,0],[alternatives.defence,'#ffbf71',2,1]]){
      if(!path||path===selected)continue;
      for(let i=1;i<path.points.length;i++)if(i%stride===phase)this.lines.add([path.points[i-1],path.points[i]],color,1);
    }
    // Red crossbars mark a falling planned speed; long continuations show the
    // selected pass/exit sequence, not merely the immediate steering target.
    selected.points.forEach((p,i)=>{
      const previous=selected.points[i-1];
      if(previous&&p.speedLimit<previous.speedLimit-.35)this.lines.add(
        [-1.15,1.15].map(side=>({x:p.x+p.nx*side,z:p.z+p.nz*side,y:.2})),colors.braking);
    });
    // Selected corridor bounds show the physical footprint along its path.
    for(const side of [-1,1])this.lines.add(selected.points.map(p=>({x:p.x+p.nx*1.01*side,z:p.z+p.nz*1.01*side,y:.10})),colors.selected,.55);
    const obs=planner.observation;
    const rear=planner.awareness?.rearThreat??planner.awareness?.rear;
    if(rear){
      const p=selected.points.find(p=>p.time>=2.5)??selected.points.at(-1),prediction=planner.perception.predict(rear,p.time);
      // A gate marks the selected corridor at the forecast time, not a barrier
      // the AI is guaranteed to defend. Arrowhead marks the rival's motion.
      this.lines.add([-1.2,1.2].map(side=>({x:p.x+p.nx*side,z:p.z+p.nz*side,y:.3})),'#ffbf71',1);
      const tip=session.track.at(prediction.s,prediction.lateral);
      this.lines.add([{x:tip.x-tip.tx*3+tip.nx,z:tip.z-tip.tz*3+tip.nz,y:.3},tip,{x:tip.x-tip.tx*3-tip.nx,z:tip.z-tip.tz*3-tip.nz,y:.3}],'#ffbf71',1);
    }
    for(const o of obs.observations){
      if(!this.fullPaths&&o.id!==planner.awareness?.front?.id&&o.id!==planner.awareness?.rear?.id&&Math.abs(o.distance)>12)continue;
      const rivalColor=o.distance>=0?'#68d9ed':'#ffbf71';
      const footprintColor=Math.abs(o.distance)<o.halfLength+2.3?'#ff7666':rivalColor;
      const predictions=[];
      for(let t=0;t<=3;t+=.5){
        const p=planner.perception.predict(o,t),point=session.track.at(p.s,p.lateral);predictions.push(point);
        const corners=[[-p.halfWidth,-p.halfLength],[p.halfWidth,-p.halfLength],[p.halfWidth,p.halfLength],[-p.halfWidth,p.halfLength],[-p.halfWidth,-p.halfLength]].map(([x,z])=>({x:point.x+point.nx*x+point.tx*z,z:point.z+point.nz*x+point.tz*z,y:.16}));
        this.lines.add(corners,footprintColor,1-t*.22);
      }
      this.lines.add(predictions,rivalColor,.6);
      for(const branch of this.fullPaths?[1,2]:[]){
        const points=[];
        for(let t=0;t<=6;t+=.4){
          const p=planner.perception.responses(o,t,driver.line)[branch];
          points.push(session.track.at(p.s,p.lateral));
        }
        this.lines.add(points,rivalColor,branch===1?.35:.18);
      }
    }
    for(const lane of this.fullPaths?obs.lanes:[]){
      const points=[];for(let d=3;d<=Math.min(65,Math.max(4,lane.clearance));d+=3)points.push(session.track.at(obs.origin.s+d,lane.lateral));
      if(points.length>1)this.lines.add(points,lane.free?'#6da86b':'#d87d48',.35);
    }
    if(this.fullPaths)for(let i=0;i<driver.debug.rollouts.length;i++)this.lines.add(driver.debug.rollouts[i].points,colors.mpc,i===driver.debug.selectedRollout?1:.22);
    this.lines.flush();
    }
    const stats=planner.stats;
    const target=planner.targetId===null?'OPEN':session.cars[planner.targetId]?.name||'CAR '+planner.targetId;
    const metrics=[['TACTICAL INTENT',stats.intent||planner.intent||'PACE'],['TARGET / COMMIT',`${target} · ${(stats.commit||0).toFixed(1)} S`],['TARGET SPEED',`${(driver.targetSpeed*3.6).toFixed(0)} KM/H`],['PATHS / REJECTED',`${stats.candidates} / ${stats.rejected}`],['FRICTION ESTIMATE',stats.friction.toFixed(2)+' μ'],['PEAK GRIP DEMAND',Math.round(stats.gripPeak*100)+'%'],['MIN. CLEARANCE',stats.clearance===99?'CLEAR':stats.clearance.toFixed(2)+' M'],['SOLVE TIME',stats.ms.toFixed(2)+' MS'],['YAW MODEL GAIN',driver.modelGain.toFixed(2)],['CONTACTS',String(session.contacts)]];
    metrics.splice(2,0,['EXIT MANOEUVRE',stats.manoeuvre],['RACE STRATEGY',stats.strategy],
      ['FLOW RESPONSE',stats.flowTarget?`PASS SLOWING ${stats.flowTarget}`:'NORMAL RACING'],
      ['PACE MODE',driver.model.thermalFreedom<(driver.model.traffic?RACECRAFT.trafficBlend:1)?'TYRE MANAGEMENT':driver.model.traffic?'TRAFFIC RESERVE':'CLEAR AIR'],
      ['PACE CALIBRATION BLEND',`${Math.round(driver.model.paceBlend*100)}%`],
      ['REAR HEAT FORECAST',`${driver.model.predictedRearHeat.toFixed(1)}°C`],
      ['CONTROL PREDICTION ERROR',`${driver.predictionError.position.toFixed(2)} M`],
      ['BODY SLIP',`${(Math.atan2(car.v,Math.max(1,car.u))*180/Math.PI).toFixed(1)}°`],
      ['PREDICTED EXIT',`${(stats.exitSpeed*3.6).toFixed(0)} KM/H`],['PROJECTED GAP',`${stats.exitAdvantage.toFixed(1)} M`],
      ['AGGRESSION',`${Math.round(stats.aggression*100)}%`],['BRAKE MODEL',`${Math.round(driver.model.confidence*100)}% CONFIDENCE`],
      ['REAR PRESSURE',`${Math.round(driver.strategy.pressure*100)}%`],['SUPERVISOR',driver.safety?.reason||'CLEAR']);
    const leaderGap=session.interval(car);
    metrics.push(['LEADER INTERVAL',leaderGap===null?'—':leaderGap.toFixed(2)+' S'],['DRIVER SKILL',driver.skill.toFixed(3)]);
    this.panel.querySelector('#debug-metrics').innerHTML=metrics.map(([k,v])=>`<span>${k}<b>${v}</b></span>`).join('');
    this.panel.querySelector('#debug-metrics').insertAdjacentHTML('beforeend',`<span>FRONT PACE COST<b>${(stats.frontTradeoff??0).toFixed(2)}</b></span><span>REAR EXPOSURE COST<b>${(stats.rearExposure??0).toFixed(2)}</b></span>`);
    this.panel.querySelector('#debug-reason').textContent=planner.reason+'. '+driver.strategy.reason+'. Red crossbars: braking. Cyan forecasts: cars ahead; orange: cars behind; coral: alongside. Faint branches: possible rival responses.';
  }
}
