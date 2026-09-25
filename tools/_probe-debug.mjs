import { Track } from '../src/sim/track.js';
import { Vehicle } from '../src/sim/vehicle.js';
import { angle, clamp } from '../src/sim/math.js';
const track=new Track('harbor-ring'); const dt=1/120;
function fresh(speed){ const car=new Vehicle(0,'P','#fff','gt'); car.place(track,320,0,speed);
 for(const w of car.wheels){ w.tyre.core=85; w.tyre.surface=85; w.tyre.pressure=(w.tyre.coldPressure+1.01325)*((85+273.15)/(24+273.15))-1.01325; w.omega=speed/car.spec.radius; }
 car.u=speed; car.speed=speed; return car; }
function laneHold(car){ const p=track.nearest(car.x,car.z);
  const look=clamp(6+car.speed*0.4,8,30); const t=track.at(p.s+look);
  const dx=t.x-car.x, dz=t.z-car.z; const lx=dx*Math.cos(car.yaw)-dz*Math.sin(car.yaw);
  const pursuit=Math.atan2(2*car.spec.wheelbase*lx, Math.max(20,dx*dx+dz*dz));
  const slip=Math.atan2(car.v,Math.max(4,car.u));
  return clamp((pursuit+slip*0.85)/car.spec.steeringLock,-0.35,0.35); }
console.log('--- brake trace from 78 ---');
const car=fresh(78);
for(let i=0;i<120*10;i++){ car.controls={steer:laneHold(car),throttle:0,brake:1}; car.step(dt,track,0);
 if(i%30===0) console.log((i/120).toFixed(2),'v',car.speed.toFixed(2),'ax',car.ax.toFixed(2),'lat',car.lateral.toFixed(2),'zone',car.zone,'kappa',car.wheels[2].tyre.kappa.toFixed(3),'abs',car.absActive); }
console.log('--- drive trace from 14 ---');
const c2=fresh(14);
for(let i=0;i<120*40;i++){ c2.controls={steer:laneHold(c2),throttle:1,brake:0}; c2.step(dt,track,0);
 if(i%120===0) console.log((i/120).toFixed(0),'v',c2.speed.toFixed(2),'ax',c2.ax.toFixed(2),'lat',c2.lateral.toFixed(2),'zone',c2.zone,'gear',c2.gear,'rpm',c2.rpm.toFixed(0)); }
