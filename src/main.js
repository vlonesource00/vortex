import * as THREE from 'three';
import { Track } from './sim/track.js';
import { VortexSession } from './vortex-session.js';
import { Keyboard } from './sim/input.js';
import { CarModel } from './render/car.js';
import { World } from './render/world.js';
import { AudioEngine } from './render/audio.js';
import { CarEffects } from './render/effects.js';
import { VortexDebugger } from './vortex-debugger.js';
import { UI } from './ui.js';
import { damp } from './sim/math.js';
import { showcaseOptions, showcaseRestartAt } from './showcase.js';
import { VisualFinish } from './render/finish.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { WeatherEffects } from './render/weather.js';
import './style.css';

const showcase=showcaseOptions(location.search);
let restartAt=null;

const renderer=new THREE.WebGLRenderer({canvas:document.querySelector('#world'),antialias:true,powerPreference:'high-performance'});
renderer.setPixelRatio(Math.min(window.devicePixelRatio,1.5));renderer.setSize(innerWidth,innerHeight);
renderer.shadowMap.enabled=true;renderer.shadowMap.type=THREE.PCFSoftShadowMap;
renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=.82;renderer.outputColorSpace=THREE.SRGBColorSpace;
const scene=new THREE.Scene(),camera=new THREE.PerspectiveCamera(48,innerWidth/innerHeight,.04,10000);
const track=new Track('harbor-ring'),session=new VortexSession(track,{classId:showcase.classId,mixed:showcase.mixed}),world=new World(scene,renderer,track);
session.player.place(track,210,2.3);
const models=session.cars.map(c=>{const m=new CarModel(c);scene.add(m.root);return m;});
new GLTFLoader().load('/assets/gt-wheel.glb',asset=>models.forEach(model=>model.setWheelAsset(asset.scene)),undefined,()=>console.warn('Detailed wheel asset unavailable; using built-in wheels.'));
const finish=new VisualFinish(renderer,scene,camera);
const audio=new AudioEngine();
const effects=new CarEffects(scene);
const weatherEffects=new WeatherEffects(scene);
const cameras=['chase','bonnet','cockpit','trackside','engineer'];let cameraIndex=0,assisted=true,last=performance.now(),accumulator=0,fps=60,elapsed=0;
let previousPhase='menu',pausePhase='racing',photoFrozen=false;
const reduceMotion=matchMedia('(prefers-reduced-motion: reduce)').matches;
function unfreeze(){photoFrozen=false;document.querySelector('#photo-frame')?.remove();renderer.domElement.style.visibility='visible';}
function start(){unfreeze();input.clear();if(showcase.enabled)session.autopilot=true;restartAt=null;session.start({freshTrack:showcase.enabled});effects.reset();accumulator=0;ui.setScreen('racing');if(!showcase.enabled)audio.unlock().catch(()=>ui.toast('Audio unavailable; driving is still ready.'));setCamera(true);}
function pause(){if(['racing','countdown'].includes(session.phase)){pausePhase=session.phase;session.phase='paused';input.clear();ui.paused();}else if(session.phase==='paused')resume();}
function resume(){session.phase=pausePhase;ui.setScreen('racing');input.clear();accumulator=0;}
function exit(){if(showcase.enabled){const q=new URLSearchParams(location.search);q.delete('showcase');location.search=q.toString();return;}unfreeze();session.phase='menu';session.reset();session.player.place(track,210,2.3);session.autopilot=false;input.clear();ui.setScreen('menu');ui.showTab('session');setCamera(true);}
function quality(value){const ratio=value==='ultra'?Math.min(devicePixelRatio,2):value==='performance'?1:Math.min(devicePixelRatio,1.5);renderer.setPixelRatio(ratio);world.sun.shadow.mapSize.setScalar(value==='ultra'?4096:value==='performance'?1024:2048);world.sun.shadow.map?.dispose();world.sun.shadow.map=null;renderer.shadowMap.needsUpdate=true;finish.setQuality(value);}
const ui=new UI(session,{start,drive:()=>{session.autopilot=false;start();},pause,resume,exit,startAI:()=>{session.autopilot=true;start();aiDebugger.toggle(true);cameraIndex=4;setCamera(true);},debug:()=>aiDebugger.toggle(),color:color=>models[0].setColor(color),quality,lighting:mode=>world.setLighting(mode),assist:value=>assisted=value,volume:value=>audio.setVolume(value)});
const aiDebugger=new VortexDebugger(scene,session,()=>{cameraIndex=cameraIndex===4?0:4;setCamera(true);});
const input=new Keyboard(code=>{
  if(code==='GamepadLost'){if(!showcase.enabled&&!session.autopilot&&['racing','countdown'].includes(session.phase))pause();ui.toast('Controller disconnected');return;}
  if(code==='Enter'){if(session.phase==='menu'){session.autopilot=false;start();}else if(session.phase==='paused')resume();return;}
  if(session.phase==='menu'&&code.startsWith('Pad')){const select=ui.$(code==='PadUp'||code==='PadDown'?'#class-choice':'#circuit-choice');select.selectedIndex=(select.selectedIndex+(code==='PadUp'||code==='PadLeft'?-1:1)+select.options.length)%select.options.length;select.onchange();return;}
  if(showcase.enabled&&['KeyP','KeyR','KeyQ','KeyE','F8'].includes(code))return;
  if(code==='F8'){
    photoFrozen=!photoFrozen;
    if(photoFrozen){
      renderer.render(scene,camera);
      const photo=document.createElement('img');photo.id='photo-frame';photo.alt='Frozen VORTEX 3D scene';photo.src=renderer.domElement.toDataURL('image/png');
      photo.style.cssText='position:fixed;inset:0;width:100vw;height:100vh;object-fit:cover;pointer-events:none;';document.body.insertBefore(photo,document.querySelector('#app'));
      renderer.domElement.style.visibility='hidden';
    }else { document.querySelector('#photo-frame')?.remove();renderer.domElement.style.visibility='visible'; }
    ui.toast(photoFrozen?'Photo freeze · F8 to resume':'Photo freeze off');return;
  }
  if(code==='Escape'){if(!ui.$('#help-overlay').hidden)ui.help(false);else pause();}
  if(session.phase==='menu')return;
  if(code==='KeyB')ui.toast(aiDebugger.toggle()?'Race engineer debugger enabled':'Debugger hidden');
  if(code==='KeyC'){cameraIndex=(cameraIndex+1)%cameras.length;setCamera(true);ui.toast(cameras[cameraIndex].toUpperCase()+' CAMERA');}
  if(code==='KeyP'&&session.phase==='racing'){session.autopilot=!session.autopilot;input.clear();ui.toast(session.autopilot?'AI demonstration enabled · P to take control':'You have control');}
  if(code==='KeyR'&&session.phase==='racing'){session.recover();input.clear();ui.toast('Recovered to circuit · +5 seconds · lap invalid');setCamera(true);}
  if(code==='KeyT'){ui.telemetry=!ui.telemetry;ui.$('#telemetry-panel').hidden=!ui.telemetry;}
  if(code==='KeyM')ui.toast(audio.toggle()?'Audio on':'Audio muted');
  if(code==='KeyQ')session.player.shift(-1);if(code==='KeyE')session.player.shift(1);
});
window.addEventListener('blur',()=>{if(!showcase.enabled&&['racing','countdown'].includes(session.phase))pause();});
document.addEventListener('visibilitychange',()=>{if(!showcase.enabled&&document.hidden&&['racing','countdown'].includes(session.phase))pause();});
window.addEventListener('resize',()=>{camera.aspect=innerWidth/innerHeight;camera.updateProjectionMatrix();renderer.setSize(innerWidth,innerHeight);finish.resize(innerWidth,innerHeight);});

const desired=new THREE.Vector3(),look=new THREE.Vector3(),forward=new THREE.Vector3(),right=new THREE.Vector3(),pos=new THREE.Vector3();
function setCamera(snap=false,dt=1/60){
  const car=cameras[cameraIndex]==='engineer'&&session.phase!=='menu'?session.cars[aiDebugger.focus]:session.player,mode=cameras[cameraIndex];
  forward.set(Math.sin(car.yaw),0,Math.cos(car.yaw));right.set(Math.cos(car.yaw),0,-Math.sin(car.yaw));pos.set(car.x,0,car.z);
  const oldFov=camera.fov,oldNear=camera.near;
  // A cockpit clip distance wastes nearly all depth precision in aerial views.
  camera.near=session.phase==='menu'?.3:mode==='engineer'?1:mode==='cockpit'?.04:mode==='bonnet'?.1:.3;
  models[0].cockpit(session.phase!=='menu'&&mode==='cockpit');
  if(session.phase==='menu'){
    const orbit=reduceMotion?0:Math.sin(elapsed*.09)*.15;
    desired.copy(pos).addScaledVector(forward,7.2+orbit).addScaledVector(right,6.3);desired.y=2.35;
    look.copy(pos).addScaledVector(right,-2.4);look.y=.68;
    camera.fov=42;
  }else if(mode==='chase'){
    desired.copy(pos).addScaledVector(forward,-7.6-car.speed*.024);desired.y=2.65;
    look.copy(pos).addScaledVector(forward,8+car.speed*.09);look.y=.7;camera.fov=55+Math.min(car.speed*.12,8);
  }else if(mode==='bonnet'){
    desired.copy(pos).addScaledVector(forward,.63);desired.y=1.12;look.copy(pos).addScaledVector(forward,50);look.y=.7;camera.fov=68;
  }else if(mode==='cockpit'){
    desired.copy(pos).addScaledVector(right,-.32).addScaledVector(forward,-.3);desired.y=1.06;look.copy(pos).addScaledVector(forward,50);look.y=.95;camera.fov=74;
  }else if(mode==='engineer'){
    desired.copy(pos).addScaledVector(forward,-27);desired.y=65;look.copy(pos).addScaledVector(forward,27);look.y=0;camera.fov=56;
  }else{
    const p=track.at(car.s+25,23);desired.set(p.x,6,p.z);look.copy(pos);look.y=.65;camera.fov=42;
  }
  camera.position.lerp(desired,snap||mode==='cockpit'||mode==='bonnet'?1:1-Math.exp(-5*dt));camera.lookAt(look);
  if(oldFov!==camera.fov||oldNear!==camera.near)camera.updateProjectionMatrix();
}
setCamera(true);
quality(innerWidth<=950?'performance':'high');
ui.$('#quality').value=innerWidth<=950?'performance':'high';
if(showcase.enabled){
  document.body.classList.add('showcase');
  session.laps=showcase.laps;session.field=showcase.field;session.paceObjective=showcase.objective;
  const appearance=new URLSearchParams(location.search),light=appearance.get('lighting');
  if(['golden','day','overcast'].includes(light))world.setLighting(light);
  if(['0','0.35','0.75'].includes(appearance.get('wetness')))session.track.wetness=Number(appearance.get('wetness'));
  if(session.track.wetness>.5&&!light)world.setLighting('overcast');
  session.player.name='VORTEX AI';quality('performance');audio.setVolume(0);
  aiDebugger.panel.querySelector('#debug-car option').textContent='VORTEX AI';
  if(innerWidth<=950){ui.telemetry=false;ui.$('#telemetry-panel').hidden=true;}
  const bar=document.createElement('div');bar.id='showcase-bar';
  bar.innerHTML='<span id="showcase-status" role="status">AI SHOWCASE</span><button id="showcase-camera">CAMERA</button><button id="showcase-telemetry">TELEMETRY</button><button id="showcase-drive">DRIVE / CLASSES</button>';
  bar.querySelector('#showcase-drive').onclick=exit;
  document.querySelector('#app').append(bar);
  bar.querySelector('#showcase-camera').onclick=()=>{cameraIndex=(cameraIndex+1)%cameras.length;setCamera(true);};
  bar.querySelector('#showcase-telemetry').onclick=()=>{ui.telemetry=!ui.telemetry;ui.$('#telemetry-panel').hidden=!ui.telemetry;};
  aiDebugger.keepPaths=true;
  cameraIndex=cameras.indexOf(showcase.camera);start();aiDebugger.toggle(innerWidth>950);
}
// Small read-only snapshot for reproducible diagnostics and browser verification.
const snapshot=()=>({phase:session.phase,time:session.time,mode:session.mode,autopilot:session.autopilot,track:'harbor-ring',trackLength:track.length,speed:session.player.speed*3.6,position:{x:session.player.x,z:session.player.z},lap:session.player.race.lap,contacts:session.contacts,physicsHz:120,drawCalls:renderer.info.render.calls,triangles:renderer.info.render.triangles,fps,tyres:session.player.wheels.map(w=>({temperature:w.tyre.core,pressure:w.tyre.pressure,wear:w.tyre.wear})),cars:session.activeCars.map(c=>({name:c.name,speed:c.speed,s:c.s,lateral:c.lateral,state:session.drivers[c.id].state,diagnostics:session.drivers[c.id].diagnostics}))});
window.vortex={snapshot};
// Compatibility hook for existing local browser smoke tooling.
window.astra=window.vortex;
function frame(now){
  const frameStart=performance.now();
  requestAnimationFrame(frame);
  input.pollGamepad();
  const inputLabel=input.connected?'Controller ready · A to drive · RT throttle / LT brake':'Keyboard ready · Connect a gamepad and press a button';
  if(ui.$('#controller-status').textContent!==inputLabel)ui.$('#controller-status').textContent=inputLabel;
  if(photoFrozen){last=now;return;}
  const raw=Math.min((now-last)/1000,.12);last=now;elapsed+=raw;fps=damp(fps,1/Math.max(.001,raw),1,raw);
  if(['racing','countdown'].includes(session.phase)){
    accumulator+=raw;let steps=0;
    while(accumulator>=1/120&&steps<16){const controls=showcase.enabled?{throttle:0,brake:0,steer:0}:input.update(session.player,1/120,assisted);session.step(1/120,controls);accumulator-=1/120;steps++;}
  }else accumulator=0;
  if(session.phase==='finished'&&previousPhase!=='finished')ui.finish();previousPhase=session.phase;
  if(showcase.enabled){
    restartAt=showcaseRestartAt(session.phase,now,restartAt);
    if(restartAt!==null&&now>=restartAt)start();
    document.querySelector('#showcase-status').textContent=!navigator.onLine?'OFFLINE · LOCAL SHOWCASE':restartAt!==null?'NEXT RACE IN '+Math.max(0,Math.ceil((restartAt-now)/1000))+'s':'AI SHOWCASE · '+session.phase.toUpperCase();
  }
  const visibleCars=session.phase==='menu'?1:session.activeCars.length;
  models.forEach((m,i)=>{m.root.visible=i<visibleCars;if(m.root.visible)m.update(session.phase==='racing'?raw:0);});
  world.update(cameras[cameraIndex]==='engineer'&&session.phase!=='menu'?session.cars[aiDebugger.focus]:session.player,session.time,session.phase==='countdown'?session.countdown:0,cameras[cameraIndex]==='engineer');setCamera(false,raw);
  if(session.phase==='racing')effects.update(session.activeCars,raw,track,renderer.domElement.height);
  effects.cloud.visible=effects.marks.visible=session.phase!=='menu';
  weatherEffects.update(session.player,raw,session.track.wetness);
  audio.update(session.player,session.phase==='racing',session.activeCars);ui.update(now,fps,cameras[cameraIndex]);
  aiDebugger.root.visible=aiDebugger.pathsEnabled&&session.phase!=='menu';aiDebugger.panel.hidden=!aiDebugger.enabled||session.phase==='menu';aiDebugger.update(now);
  finish.render();
  if(import.meta.env.DEV&&Math.floor(now/500)!==frame.diagnosticTick){
    frame.diagnosticTick=Math.floor(now/500);
    renderer.domElement.dataset.performance=JSON.stringify({cpuMs:+(performance.now()-frameStart).toFixed(2),fps:Math.round(fps),drawCalls:renderer.info.render.calls,triangles:renderer.info.render.triangles,marks:effects.count,particles:effects.particles.filter(p=>p.life>0).length});
  }
}
requestAnimationFrame(frame);
