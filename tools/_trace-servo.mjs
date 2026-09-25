import { Track } from '../src/sim/track.js';
import { VortexSession } from '../src/vortex-session.js';
const session = new VortexSession(new Track('harbor-ring'), { classId: 'gt' });
session.mode='practice'; session.field=1; session.laps=2; session.autopilot=true;
session.start({ freshTrack: true });
const zero={steer:0,throttle:0,brake:0};
let t=0;
for(let i=0;i<120*45;i++){
  session.step(1/120, zero); t+=1/120;
  const c=session.player, d=session.drivers[0], S=d.limiter?.servo;
  if(c.s>815 && c.s<860 && i%12===0 && S){
    console.log(t.toFixed(2).padStart(6),
      's',c.s.toFixed(1).padStart(6),
      'v',c.speed.toFixed(2).padStart(6),
      'err',S.trackingError.toFixed(2).padStart(6),
      'purs',S.pursuit.toFixed(3).padStart(7),
      'slip',S.slip.toFixed(3).padStart(7),
      'trk',S.tracking.toFixed(3).padStart(7),
      'rot',S.rotation.toFixed(3).padStart(7),
      'ff',S.feedforward.toFixed(3).padStart(7),
      'yaw',S.yawError.toFixed(3).padStart(7),
      'corr',S.correction.toFixed(4).padStart(8),
      'raw',S.raw.toFixed(3).padStart(7),
      'des',S.desired.toFixed(3).padStart(7),
      'cmd',c.controls.steer.toFixed(3).padStart(6),
      'lat',c.lateral.toFixed(2).padStart(6),
      'hereOff',S.hereOffset.toFixed(2).padStart(6),
      'tgtOff',S.targetOffset.toFixed(2).padStart(6),
      'lx',S.localX.toFixed(2).padStart(6));
  }
}
