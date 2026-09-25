import { clamp } from './sim/math.js';
export const timeFormat = seconds => seconds===null||!Number.isFinite(seconds)?'—:——.———':`${Math.floor(seconds/60)}:${(seconds%60).toFixed(3).padStart(6,'0')}`;
const arrow='<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5"><path d="M4 12h15M13 5l7 7-7 7"/></svg>';
const logo='<svg viewBox="0 0 36 34" fill="none"><path d="M2 30L17 3h5L7 30H2Z" fill="currentColor"/><path d="M17 19l6-11 12 22h-7l-5-10-6 10H10l7-11Z" fill="currentColor"/></svg>';

export class UI {
  constructor(session,handlers){
    this.session=session;this.handlers=handlers;this.tab='session';this.telemetry=true;
    document.querySelector('#app').innerHTML=`
    <header id="header"><a class="brand" href="#" aria-label="VORTEX home">${logo}<span>VORTEX<small>MOTORSPORT</small></span></a>
      <nav aria-label="Main navigation"><button class="nav active" data-tab="session">DRIVE</button><button class="nav" data-tab="garage">GARAGE</button><button class="nav" data-tab="settings">SETTINGS</button></nav>
      <div class="header-right"><span class="live-dot"></span> LOCAL SESSION <span class="version">BUILD 01</span></div>
    </header>
    <main id="menu">
      <section class="menu-panel" id="session-panel">
        <div class="eyebrow"><span class="red-line"></span> PURE DRIVING. NOTHING ELSE.</div>
        <h1>Find your<br><em>limit.</em></h1>
        <p class="intro">Every input matters. Every corner counts.<br>An uncompromising drive, right in your browser.</p>
        <div class="session-tabs" role="group" aria-label="Session type"><button class="selected" data-mode="race">QUICK RACE</button><button data-mode="practice">FREE PRACTICE</button></div>
        <div class="circuit-card"><div class="circuit-heading"><div><small>YOUR NEXT CHALLENGE</small><h2>Harbor Ring <span>↗</span></h2><p>Waterfront circuit <span>·</span> Grand Prix layout</p></div><canvas id="menu-map" width="230" height="142" aria-label="Harbor Ring track map"></canvas></div>
          <div class="circuit-stats"><div><strong id="track-length">—</strong><small>KM CIRCUIT</small></div><div><strong>16</strong><small>CORNERS</small></div><div><strong>31°<span>C</span></strong><small>TRACK TEMP</small></div><div><strong>17:42</strong><small>GOLDEN HOUR</small></div></div>
        </div>
        <div class="session-options"><label>RACE DISTANCE<select id="laps"><option value="3">3 laps · Sprint</option><option value="5">5 laps · Club</option><option value="10">10 laps · Endurance</option></select></label><label>GRID SIZE<select id="field"><option value="6">6 drivers</option><option value="4">4 drivers</option><option value="8">8 drivers</option></select></label></div>
        <button class="primary" id="start"><span>TAKE THE GRID</span>${arrow}</button>
        <button class="ai-launch" id="ai-demo">WATCH AI RACE <span>OPEN RACE ENGINEER ↗</span></button>
        <div class="start-note"><span class="key-mini">W</span><span class="key-mini">A</span><span class="key-mini">S</span><span class="key-mini">D</span><span>Keyboard ready</span><i></i><span>Automatic transmission</span></div>
      </section>
      <section class="menu-panel secondary-panel" id="garage-panel" hidden><div class="eyebrow"><span class="red-line"></span> ENGINEER YOUR ADVANTAGE</div><h1>Make it<br><em>yours.</em></h1><p class="intro">A1 GT / rear-wheel drive / six-speed sequential.<br>Setup changes take effect at the next session.</p><h3>LIVERY</h3><div class="swatches"><button aria-label="Vermilion livery" data-color="#df482d" style="--swatch:#df482d" class="chosen"></button><button aria-label="Alpine green livery" data-color="#356653" style="--swatch:#356653"></button><button aria-label="Ivory livery" data-color="#deded2" style="--swatch:#deded2"></button><button aria-label="Midnight blue livery" data-color="#304d75" style="--swatch:#304d75"></button></div><div class="setup-controls">
        <label>REAR WING <output id="wing-value">6 / 10</output><input id="wing" type="range" min="1" max="10" value="6"></label>
        <label>BRAKE BIAS <output id="bias-value">58% FRONT</output><input id="bias" type="range" min="50" max="68" value="58"></label>
        <label>COLD TYRE PRESSURE <output id="pressure-value">1.65 BAR</output><input id="pressure" type="range" min="145" max="195" value="165"></label>
        <label>FUEL LOAD <output id="fuel-value">35 L</output><input id="fuel" type="range" min="10" max="80" value="35"></label>
      </div><button class="primary" id="garage-drive"><span>BACK TO DRIVE</span>${arrow}</button></section>
      <section class="menu-panel secondary-panel" id="settings-panel" hidden><div class="eyebrow"><span class="red-line"></span> YOUR DRIVING EXPERIENCE</div><h1>In your<br><em>control.</em></h1><div class="settings-list">
      <label>GRAPHICS<select id="quality"><option value="high">High · 2K shadows</option><option value="ultra">Ultra · 4K shadows</option><option value="performance">Performance</option></select></label>
      <label>LIGHTING<select id="lighting"><option value="golden">Golden hour</option><option value="day">Clear daylight</option><option value="overcast">Overcast</option></select></label>
      <label>TRACK WEATHER<select id="weather"><option value="0">Dry</option><option value="0.35">Damp surface</option><option value="0.75">Rain · wet circuit</option></select></label>
      <label>TRANSMISSION<select id="transmission"><option value="auto">Automatic</option><option value="manual">Manual · Q / E</option></select></label>
      <label>KEYBOARD ASSIST<select id="assist"><option value="on">On · progressive steering</option><option value="off">Off · direct steering</option></select></label>
      <label>TRACTION CONTROL<select id="tc"><option value="3">Sport · 3</option><option value="6">Safe · 6</option><option value="0">Off</option></select></label>
      <label>ANTI-LOCK BRAKES<select id="abs"><option value="4">Sport · 4</option><option value="7">Safe · 7</option><option value="0">Off</option></select></label>
      <label>AI AGGRESSION<select id="aggression"><option value="0.72">Race · committed attacks</option><option value="0.95">Fierce · higher risk tolerance</option><option value="0.4">Measured · patient attacks</option></select></label>
      <label>AI PACE GOAL<select id="pace-objective"><option value="race">Race · sustained stint pace</option><option value="qualifying">Qualifying · single-lap pace</option></select></label>
      <label class="volume-label">AUDIO VOLUME<input id="volume" type="range" min="0" max="100" value="36"></label>
      </div><button class="text-button" id="controls-open">VIEW KEYBOARD CONTROLS ${arrow}</button></section>
      <div class="vehicle-caption"><div class="eyebrow">VORTEX WORKS TEAM <span> / </span> 07</div><div class="vehicle-name">A1 <em>GT</em></div><div class="vehicle-specs"><span>575 <small>NM</small></span><span>1,290 <small>KG DRY</small></span><span>RWD <small>DRIVETRAIN</small></span></div><p>Engineered for the edge.</p></div>
      <div class="scene-marker"><span class="live-dot"></span> LIVE 3D <span> / </span> HARBOR RING</div>
    </main>
    <footer id="footer"><span>BUILT FOR THE FEELING.</span><div><span>120 HZ PHYSICS</span><i></i><span>DYNAMIC TYRES</span><i></i><span>ORIGINAL RACE AI</span></div><button id="help">CONTROLS <span>?</span></button></footer>
    <div id="hud" hidden>
      <div class="race-top"><div class="race-brand">${logo}<span>VORTEX <b>LIVE</b></span></div><div class="race-phase"><span class="live-dot"></span><span id="phase-label">SPRINT RACE</span></div><div class="race-actions"><button id="debug-toggle">RACE ENGINEER <small>B</small></button><button id="pause" aria-label="Pause race">Ⅱ <small>ESC</small></button></div></div>
      <section class="leaderboard hud-panel"><div class="hud-title">CLASSIFICATION <span>GT</span></div><div id="standings"></div></section>
      <div class="lap-panel hud-panel"><div><small>LAP</small><strong id="lap">1 <span>/ 3</span></strong></div><div><small>CURRENT LAP</small><strong id="lap-time">0:00.000</strong></div><div><small>BEST</small><strong id="best-lap">—:——.———</strong></div></div>
      <div class="timing-message" id="timing-message"></div><div id="countdown"></div>
      <section class="telemetry-panel hud-panel" id="telemetry-panel"><div class="hud-title">TYRE TELEMETRY <span>LIVE</span></div><div class="tyres" id="tyres"></div><div class="telemetry-values"><span>AERO <b id="downforce">0 N</b></span><span>FUEL <b id="fuel-readout">35.0 L</b></span><span>BALANCE <b id="balance">58% F</b></span><span>DAMAGE <b id="damage">0%</b></span></div></section>
      <section class="speed-panel"><div id="rpm-leds"></div><div class="speed-main"><div class="gear" id="gear">1</div><div class="speed-number"><strong id="speed">0</strong><span>KM/H</span></div><div class="pedals"><div><i id="brake-bar"></i></div><div><i id="throttle-bar"></i></div></div></div><div class="speed-sub"><span id="rpm">1,100 RPM</span><span id="electronics">TC 3 <i>·</i> ABS 4</span><span id="drive-mode">AUTO</span></div></section>
      <div class="minimap-panel"><canvas id="race-map" width="250" height="180" aria-label="Live circuit map"></canvas><span>${session.track.name.toUpperCase()} <b id="position">P1</b></span></div>
      <div class="race-bottom"><span><kbd>C</kbd> CAMERA <kbd>T</kbd> TELEMETRY <kbd>P</kbd> AI DEMO <kbd>R</kbd> RECOVER</span><span id="camera-label">CHASE CAMERA</span><span id="fps">60 FPS</span></div>
    </div>
    <div id="pause-overlay" class="overlay" hidden><section class="dialog"><span class="eyebrow">VORTEX MOTORSPORT</span><h2 id="pause-title">Take a breath.</h2><p id="pause-text">Your session is paused.</p><button class="primary" id="resume">RESUME SESSION ${arrow}</button><button class="secondary" id="restart">RESTART RACE</button><button class="text-button" id="exit">RETURN TO PADDOCK</button></section></div>
    <div id="help-overlay" class="overlay" hidden><section class="dialog controls-dialog"><button id="help-close" class="close" aria-label="Close controls">×</button><span class="eyebrow">THE DRIVER'S HANDBOOK</span><h2>You’re in control.</h2><div class="controls-grid"><span><kbd>W</kbd> / <kbd>↑</kbd></span><p>Throttle</p><span><kbd>S</kbd> / <kbd>↓</kbd></span><p>Brake</p><span><kbd>A</kbd><kbd>D</kbd> / <kbd>←</kbd><kbd>→</kbd></span><p>Steer</p><span><kbd>Q</kbd> / <kbd>E</kbd></span><p>Shift down / up (manual)</p><span><kbd>C</kbd></span><p>Chase / bonnet / cockpit / trackside</p><span><kbd>P</kbd></span><p>Toggle AI demonstration</p><span><kbd>R</kbd></span><p>Recover to circuit (+5 seconds)</p><span><kbd>T</kbd> / <kbd>M</kbd></span><p>Telemetry / mute</p><span><kbd>ESC</kbd></span><p>Pause / resume</p></div><p class="help-note">Brake before the corner. Release progressively, turn in, then feed in the throttle. Cold tyres and gravel reduce grip.</p></section></div>
    <div id="toast" role="status"></div>`;
    const $=s=>document.querySelector(s);this.$=$;
    const spec=session.player.spec;
    $('.circuit-heading h2').textContent=session.track.name;
    $('.circuit-heading p').textContent='Waterfront circuit · canonical Harbor Ring layout';
    $('#menu-map').setAttribute('aria-label',session.track.name+' track map');
    $('.vehicle-name').textContent='VORTEX '+spec.label;
    $('.vehicle-specs').textContent=`${spec.maxTorque} NM · ${spec.mass} KG · ${spec.drive==='front'?'FWD':'RWD'}`;
    $('#garage-panel .intro').textContent=`${spec.label} / ${spec.drive==='front'?'front':'rear'}-wheel drive / six-speed sequential. Setup changes take effect at the next session.`;
    $('.scene-marker').textContent='LIVE 3D / '+session.track.name.toUpperCase();
    $('.leaderboard .hud-title span').textContent=session.mixed?'MULTI-CLASS':spec.label;
    const selection=document.createElement('div');selection.className='session-options';
    selection.innerHTML='<label>CIRCUIT<select id="circuit-choice"><option value="harbor-ring">Harbor Ring</option></select></label><label>YOUR CAR CLASS<select id="class-choice"><option value="gt">GT · rear-wheel drive</option><option value="touring">Touring · front-wheel drive</option><option value="prototype">Prototype · high downforce</option></select></label><label>OPPONENT CLASSES<select id="grid-choice"><option value="single">Same class as you</option><option value="mixed">Mixed · all three classes</option></select></label>';
    $('.circuit-card').after(selection);$('#circuit-choice').value=session.track.id;$('#class-choice').value=session.classId;$('#grid-choice').value=session.mixed?'mixed':'single';
    const initial=new URLSearchParams(location.search);for(const id of ['laps','field']){const value=initial.get(id);if([...$('#'+id).options].some(o=>o.value===value))$('#'+id).value=value;}
    session.laps=Number($('#laps').value);session.field=Number($('#field').value);
    const reloadSelection=()=>{const q=new URLSearchParams(location.search);q.delete('showcase');q.set('track',$('#circuit-choice').value);q.set('class',$('#class-choice').value);q.set('grid',$('#grid-choice').value);q.set('laps',$('#laps').value);q.set('field',$('#field').value);location.search=q.toString();};
    $('#circuit-choice').onchange=$('#class-choice').onchange=$('#grid-choice').onchange=reloadSelection;
    const controllerNote=document.createElement('p');controllerNote.className='start-note';controllerNote.id='controller-status';controllerNote.textContent='Keyboard ready · Connect a gamepad and press a button';$('#start').before(controllerNote);
    const padHelp=document.createElement('p');padHelp.className='help-note';padHelp.textContent='Gamepad: left stick steer · RT accelerate · LT brake · hold X + RT to reverse at low speed · LB/RB shift · Y camera · A start/resume · Start pause. Menu: D-pad up/down changes class; left/right changes circuit. Standard Xbox/PlayStation browser mapping.';$('.controls-dialog').append(padHelp);
    $('#track-length').textContent=(session.track.length/1000).toFixed(2);
    document.querySelectorAll('[data-tab]').forEach(b=>b.onclick=()=>this.showTab(b.dataset.tab));
    $('.brand').onclick=e=>{e.preventDefault();if(session.phase==='menu')this.showTab('session');else handlers.pause();};
    document.querySelectorAll('[data-mode]').forEach(b=>b.onclick=()=>{session.mode=b.dataset.mode;document.querySelectorAll('[data-mode]').forEach(x=>x.classList.toggle('selected',x===b));$('#start span').textContent=session.mode==='race'?'TAKE THE GRID':'ENTER THE CIRCUIT';$('.session-options').style.opacity=session.mode==='practice'?'.4':'1';$('#laps').disabled=$('#field').disabled=session.mode==='practice';});
    $('#laps').onchange=e=>session.laps=Number(e.target.value);$('#field').onchange=e=>session.field=Number(e.target.value);
    $('#start').onclick=()=>handlers.drive();$('#ai-demo').onclick=()=>handlers.startAI();$('#debug-toggle').onclick=()=>handlers.debug();$('#garage-drive').onclick=()=>this.showTab('session');
    $('#help').onclick=$('#controls-open').onclick=()=>this.help(true);$('#help-close').onclick=()=>this.help(false);
    $('#pause').onclick=()=>handlers.pause();$('#resume').onclick=()=>handlers.resume();$('#restart').onclick=()=>handlers.start();$('#exit').onclick=()=>handlers.exit();
    document.querySelectorAll('[data-color]').forEach(b=>b.onclick=()=>{document.querySelectorAll('[data-color]').forEach(x=>x.classList.toggle('chosen',x===b));handlers.color(b.dataset.color);});
    const setup=session.player.setup;
    for(const [id,prop,scale,format] of [['wing','wing',1,v=>`${v} / 10`],['bias','brakeBias',.01,v=>`${v}% FRONT`],['pressure','pressure',.01,v=>`${(v/100).toFixed(2)} BAR`],['fuel','fuel',1,v=>`${v} L`]])$('#'+id).oninput=e=>{const v=Number(e.target.value);setup[prop]=v*scale;$('#'+id+'-value').textContent=format(v);};
    $('#quality').onchange=e=>handlers.quality(e.target.value);$('#lighting').onchange=e=>handlers.lighting(e.target.value);
    $('#weather').onchange=e=>{session.track.wetness=Number(e.target.value);if(session.track.wetness>.5){$('#lighting').value='overcast';handlers.lighting('overcast');}};
    $('#transmission').onchange=e=>{session.player.automatic=e.target.value==='auto';$('.start-note span:last-child').textContent=session.player.automatic?'Automatic transmission':'Manual · Q / E';};
    $('#assist').onchange=e=>handlers.assist(e.target.value==='on');$('#tc').onchange=e=>setup.tc=Number(e.target.value);$('#abs').onchange=e=>setup.abs=Number(e.target.value);$('#volume').oninput=e=>handlers.volume(Number(e.target.value)/100);
    $('#aggression').onchange=e=>{session.aggression=Number(e.target.value);session.drivers.forEach(d=>d.strategy.aggression=session.aggression);};
    $('#pace-objective').onchange=e=>session.paceObjective=e.target.value;
    $('#rpm-leds').innerHTML=Array.from({length:15},()=>'<i></i>').join('');
    this.drawMap($('#menu-map'),false);this.hudLast=0;
  }
  showTab(tab){this.tab=tab;document.querySelectorAll('[data-tab]').forEach(b=>b.classList.toggle('active',b.dataset.tab===tab));for(const id of ['session','garage','settings'])this.$('#'+id+'-panel').hidden=id!==tab;}
  help(show){this.$('#help-overlay').hidden=!show;if(show)this.$('#help-close').focus();}
  toast(text){this.$('#toast').textContent=text;this.$('#toast').classList.add('visible');clearTimeout(this.toastTimer);this.toastTimer=setTimeout(()=>this.$('#toast').classList.remove('visible'),3200);}
  setScreen(screen){const menu=screen==='menu';for(const id of ['menu','header','footer'])this.$('#'+id).hidden=!menu;this.$('#hud').hidden=menu;this.$('#pause-overlay').hidden=screen!=='paused'&&screen!=='finished';document.body.classList.toggle('driving',!menu);}
  finish(){this.setScreen('finished');this.$('#pause-title').textContent='Across the line.';const rank=this.session.standings().indexOf(this.session.player)+1;this.$('#pause-text').textContent=`P${rank} of ${this.session.activeCars.length} · ${timeFormat(this.session.time)} · Best lap ${timeFormat(this.session.player.race.bestLap)}`;this.$('#resume').hidden=true;}
  paused(){this.setScreen('paused');this.$('#pause-title').textContent='Take a breath.';this.$('#pause-text').textContent='Your session is paused.';this.$('#resume').hidden=false;}
  drawMap(canvas,live){
    const c=canvas.getContext('2d'),w=canvas.width,h=canvas.height; c.clearRect(0,0,w,h);
    const tx=x=>w/2+x*(w-25)/1050,tz=z=>h/2+(z-30)*(h-25)/680;
    c.lineJoin='round';c.lineCap='round';c.beginPath();this.session.track.nodes.forEach((p,i)=>i?c.lineTo(tx(p.x),tz(p.z)):c.moveTo(tx(p.x),tz(p.z)));c.closePath();c.strokeStyle=live?'#7a8a80':'#9c9e91';c.lineWidth=live?4:3;c.stroke();
    const start=this.session.track.at(this.session.track.finishS);c.fillStyle='#e65032';c.fillRect(tx(start.x)-3,tz(start.z)-4,6,8);
    if(live)for(const car of this.session.activeCars){c.beginPath();c.arc(tx(car.x),tz(car.z),car.id===0?4:2.8,0,Math.PI*2);c.fillStyle=car.id===0?'#fb5c39':'#ebe9d9';c.fill();}
  }
  update(now,fps,camera){
    const s=this.session,c=s.player,$=this.$;if(s.phase==='menu')return;
    $('#countdown').textContent=s.phase==='countdown'?Math.max(1,Math.ceil(s.countdown)).toString():s.time<1.5?'GO':'';
    if(now-this.hudLast<80)return;this.hudLast=now;
    const order=s.standings(),position=order.indexOf(c)+1;
    $('#speed').textContent=Math.round(c.speed*3.6);$('#gear').textContent=c.gear;$('#rpm').textContent=Math.round(c.rpm).toLocaleString()+' RPM';$('#drive-mode').textContent=s.autopilot?'AI DEMO':c.automatic?'AUTO':'MANUAL';
    $('#phase-label').textContent=s.autopilot?'AI DEMONSTRATION':s.mode==='practice'?'FREE PRACTICE':'SPRINT RACE';
    $('#lap').innerHTML=`${Math.min(c.race.lap,s.laps)} <span>/ ${s.mode==='practice'?'∞':s.laps}</span>`;$('#lap-time').textContent=timeFormat(s.time-c.race.lapStart);$('#best-lap').textContent=timeFormat(c.race.bestLap);
    $('#throttle-bar').style.height=c.controls.throttle*100+'%';$('#brake-bar').style.height=c.controls.brake*100+'%';
    $('#electronics').innerHTML=`<b class="${c.tcActive?'active-assist':''}">TC ${c.setup.tc}</b><i>·</i><b class="${c.absActive?'active-assist':''}">ABS ${c.setup.abs}</b>`;
    $('#rpm-leds').querySelectorAll('i').forEach((led,i)=>{led.className=c.rpm>3100+i*300?(i<7?'green':i<12?'red':'blue'):'';});
    $('#standings').innerHTML=order.map((v,i)=>{const gap=s.interval(v,order[0]);return `<div class="standing ${v===c?'you':''}"><span>${i+1}</span><i style="background:${v.color}"></i><b>${v.name}</b><small>${v===c?'YOU':i===0?'LEADER':gap===null?'—':'+'+gap.toFixed(1)}</small></div>`;}).join('');
    $('#tyres').innerHTML=c.wheels.map((w,i)=>`<div class="tyre"><span>${['FL','FR','RL','RR'][i]}</span><i style="--heat:${w.tyre.core>105?'#e68141':w.tyre.core>75?'#9dbba4':'#80adba'}"><b>${Math.round(w.tyre.core)}°</b></i><small>${w.tyre.pressure.toFixed(2)} BAR</small><em>${(100-w.tyre.wear*100).toFixed(0)}%</em></div>`).join('');
    $('#downforce').textContent=Math.round(c.aero.downforce)+' N';$('#fuel-readout').textContent=c.fuel.toFixed(1)+' L';$('#balance').textContent=Math.round(c.setup.brakeBias*100)+'% F';$('#damage').textContent=Math.round(c.damage*100)+'%';
    $('#position').textContent='P'+position;$('#camera-label').textContent=camera.toUpperCase()+' CAMERA';$('#fps').textContent=Math.round(fps)+' FPS';
    $('#timing-message').textContent=Math.abs(c.lateral)>s.track.halfWidth?'TRACK LIMITS · LAP INVALID':!c.race.valid?'LAP INVALID':s.autopilot?s.drivers[0].state:'';
    this.drawMap($('#race-map'),true);
  }
}
