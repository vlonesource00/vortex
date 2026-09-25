import { Track } from '../src/sim/track.js';
import { VortexSession } from '../src/vortex-session.js';
const session = new VortexSession(new Track('harbor-ring'), { classId: 'gt' });
session.mode='practice'; session.field=1; session.laps=2; session.autopilot=true;
session.start({ freshTrack: true });
const zero={steer:0,throttle:0,brake:0};
let t=0;
for(let i=0;i<120*45;i++){
  session.step(1/120, zero); t+=1/120;
  const c=session.player, d=session.drivers[0];
  if(c.s>770 && c.s<960 && i%6===0){
    const p=d.planner.at(c.s);
    console.log(t.toFixed(2).padStart(6),
      's',c.s.toFixed(1).padStart(6),
      'v',c.speed.toFixed(2).padStart(6),
      'lat',c.lateral.toFixed(2).padStart(6),
      'line',p.offset.toFixed(2).padStart(6),
      'err',(c.lateral-p.offset).toFixed(2).padStart(6),
      'tgt',d.targetSpeed.toFixed(1).padStart(5),
      'thr',c.controls.throttle.toFixed(2),
      'brk',c.controls.brake.toFixed(2),
      'str',c.controls.steer.toFixed(2),
      'steerRad',c.steering.toFixed(3),
      'ay',c.ay.toFixed(1).padStart(6),
      'beta',(Math.atan2(c.v,Math.max(4,c.u))*57.3).toFixed(1).padStart(5),
      d.state);
  }
}
