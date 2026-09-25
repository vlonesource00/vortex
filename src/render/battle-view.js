const escape=value=>String(value).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const time=value=>Number.isFinite(value)?value.toFixed(1)+' s':'—';
const position=value=>`${Math.abs(value).toFixed(1)} m ${value>=0?'ahead':'behind'}`;

export function battleAlternatives(planner){
  const safe=planner.candidates.filter(p=>!p.hardConflict&&p.pack);
  const best=key=>safe.reduce((winner,p)=>!winner||p.pack[key]<winner.pack[key]-1e-6||Math.abs(p.pack[key]-winner.pack[key])<1e-6&&p.score<winner.score?p:winner,null);
  return {attack:planner.awareness?.front?best('frontCost'):null,defence:planner.awareness?.rear?best('rearCost'):null};
}

export function battleView(car,planner){
  const a=planner.awareness;if(!a)return '';
  const plan=planner.plan,exit=plan.points.at(-1);
  const alternatives=battleAlternatives(planner);
  const frontObjective=a.front?.overlap?'Complete the pass with exit speed':a.front?'Close the gap without sacrificing the exit':'Build clear-track pace';
  const rearObjective=a.rearThreat?`Closing threat: ${a.rearThreat.name} · ${Number.isFinite(a.rearThreat.ttc)?time(a.rearThreat.ttc)+' to overlap at current speeds':'close pressure'} · ${a.rearThreatSeparateLane?'separate lane from intervening traffic':'traffic in its current lane'}`:a.rear&&a.rear.closing<-.5?'Nearest pursuer is falling back':'Preserve exit speed; monitor the rear';
  const card=(rival,front)=>{
    const committed=rival?.id===planner.targetId;
    const action=!rival?'Clear':rival.overlap?'Alongside':committed&&planner.intent==='ATTACK'?'Attacking':committed&&planner.intent==='DEFEND'?'Defending':
      front?(rival.closing>.5?'Closing':'Following'):(rival.ttc<3?'Under pressure':'Monitoring');
    return `<section class="battle-card ${front?'attack':'defence'}"><h3>${front?'ATTACK / FRONT':'DEFENCE / REAR'}</h3><strong>${action}</strong><span>${escape(rival?.name??'No observed rival')}</span><small>${rival?`${Math.abs(rival.distance).toFixed(1)} m · ${rival.closing>=0?'+':''}${(rival.closing*3.6).toFixed(0)} km/h closing`:'Keep building pace'}</small><small>${escape(front?frontObjective:rearObjective)}</small></section>`;
  };
  const color=o=>o.overlap?'#ff7666':o.distance>=0?'#68d9ed':'#ffbf71';
  const x=offset=>110+offset*8,y=distance=>125-distance*1.4;
  const roadHalfWidth=planner.line?.track.halfWidth??6.5,roadLeft=x(-roadHalfWidth),roadRight=x(roadHalfWidth);
  const objectivePaths=[[alternatives.attack,'#68d9ed','5 5'],[alternatives.defence,'#ffbf71','1 5']].filter(([p])=>p&&p!==plan).map(([p,c,dash])=>`<polyline points="${p.points.filter(p=>p.distance<75).map(p=>`${x(p.offset)},${y(p.distance)}`).join(' ')}" fill="none" stroke="${c}" stroke-width="2" stroke-dasharray="${dash}"/>`).join('');
  const dots=a.rivals.filter(o=>Math.abs(o.distance)<70).map(o=>`<g><title>${escape(o.name)}: ${position(o.distance)}</title><rect x="${x(o.lateral)-o.halfWidth*8}" y="${y(o.distance)-o.halfLength*1.4}" width="${o.halfWidth*16}" height="${o.halfLength*2.8}" rx="2" fill="${color(o)}"/><text x="${x(o.lateral)+14}" y="${y(o.distance)+3}" fill="${color(o)}">${escape(o.id===a.front?.id?'FRONT':o.id===a.rear?.id?'REAR':'CAR')}</text></g>`).join('');
  const path=plan.points.filter(p=>p.distance<75).map(p=>`${x(p.offset)},${y(p.distance)}`).join(' ');
  const clearance=v=>Number.isFinite(v)?Math.max(0,v).toFixed(1)+' m':'Clear';
  const forecastRivals=[...new Map([a.front,a.rear,a.rearThreat].filter(Boolean).map(o=>[o.id,o])).values()];
  const rows=forecastRivals.map(o=>{
    const future=planner.perception.predict(o,exit.time).distance-exit.distance;
    return `<tr><th scope="row">${o===a.front?'Front':o===a.rear?'Nearest rear':'Closing threat'}<small>${escape(o.name)}</small></th><td>${Math.abs(o.distance).toFixed(1)} m</td><td>${time(o.ttc)}</td><td>${position(future)}</td></tr>`;
  }).join('');
  const comparison=new Map();
  for(const [p,label] of [[plan,'Selected'],[alternatives.attack,'Front'],[alternatives.defence,'Rear']])if(p)comparison.set(p,[...(comparison.get(p)??[]),label]);
  const comparisonTable=`<table class="battle-table"><caption>Path trade-offs · lower costs are better</caption><thead><tr><th>Path</th><th>Front cost</th><th>Rear cost</th><th>Exit speed</th></tr></thead><tbody>${[...comparison].map(([p,labels])=>`<tr><th>${labels.join(' / ')}${p.hardConflict?'<small>Conflict flagged</small>':''}</th><td>${(p.pack?.frontCost??0).toFixed(2)}</td><td>${(p.pack?.rearCost??0).toFixed(2)}</td><td>${((p.exitSpeed??p.points.at(-1).speed??0)*3.6).toFixed(0)} km/h</td></tr>`).join('')}</tbody></table><p class="battle-note">Costs are separate scoring terms. The selected path also weighs time, grip, collision risk and commitment.</p>`;
  return `<div class="battle-cards">${card(a.front,true)}${card(a.rear,false)}</div><p class="battle-note">Front-progress path: ${alternatives.attack===plan?'selected':alternatives.attack?'cyan dashed alternative':'none'}. Rear-protection path: ${alternatives.defence===plan?'selected':alternatives.defence?'orange dotted alternative':'none'}. Green is the path being driven.</p>
    <div class="battle-space"><svg viewBox="0 0 220 235" role="img" aria-label="Track-relative proximity diagram. Cyan rivals ahead, orange behind, coral alongside; red is the observed car."><rect x="${roadLeft}" y="10" width="${roadRight-roadLeft}" height="215" rx="8" fill="#223c37"/><path d="M${roadLeft} 10V225M${roadRight} 10V225" stroke="#69867b"/><path d="M110 10V225" stroke="#69867b" stroke-dasharray="4 8"/><text x="10" y="18">AHEAD</text><text x="10" y="223">BEHIND</text><polyline points="${path}" fill="none" stroke="#b8ff64" stroke-width="2"/>${dots}<rect x="${x(planner.observation.origin.lateral)-8}" y="120" width="16" height="10" rx="2" fill="#ff5145" stroke="white"/><text x="10" y="128">YOU</text></svg><div class="battle-space-data"><span>LEFT SPACE<b>${clearance(a.left)}</b></span><span>RIGHT SPACE<b>${clearance(a.right)}</b></span><span>COMMITTED SIDE<b>${planner.commitSide<0?'Left':planner.commitSide>0?'Right':'Racing line'}</b></span><span>WIDTH / HOLD<b>${(planner.widthCommit?.width??0).toFixed(1)} m / ${planner.commit.toFixed(1)} s</b></span><small>Local footprint gaps within 12 m. Clear is not permission to change lanes.</small></div></div>
    <table class="battle-table"><caption>Battle rivals · exit forecast ${time(exit.time)}</caption><thead><tr><th>Rival</th><th>Gap</th><th>Closing time</th><th>At exit</th></tr></thead><tbody>${rows||'<tr><td colspan="4">No rivals in observation range</td></tr>'}</tbody></table><p class="battle-note">Closing time uses bumper gaps and current relative speed; intervening traffic may prevent that overlap. The orange rear forecast tracks the closing threat when present. These warnings do not change the committed target. Exit positions are predictions, not guaranteed outcomes. Pace calibration blend is the blend between conservative and faster limits, not throttle percentage.</p>${comparisonTable}`.replace('</svg>',objectivePaths+'</svg>');
}
