import { Track } from '../src/sim/track.js';
import { VortexSession } from '../src/vortex-session.js';
const session = new VortexSession(new Track('harbor-ring'), { classId: 'gt' });
session.mode='practice'; session.field=1; session.laps=2; session.autopilot=true;
session.start({ freshTrack: true });
const zero={steer:0,throttle:0,brake:0};
for(let i=0;i<120*220;i++){
  session.step(1/120, zero);
  if(i%(120*10)===0){
    const c=session.player, d=session.drivers[0], L=d.limiter;
    console.log((i/120).toFixed(0).padStart(4),
      's',c.s.toFixed(0).padStart(5),
      'v',c.speed.toFixed(2).padStart(6),
      'lat',c.lateral.toFixed(2).padStart(6),
      'plan',L?.planned.toFixed(2).padStart(6),
      'tgt',L?.targetSpeed.toFixed(2).padStart(6),
      'ff',L?.feedforward.toFixed(2).padStart(6),
      'thr',c.controls.throttle.toFixed(2),
      'brk',c.controls.brake.toFixed(2),
      'str',c.controls.steer.toFixed(2),
      'lap',c.race.lap, 'off',session.player.race.offtrack.toFixed(1),
      d.state);
  }
}
