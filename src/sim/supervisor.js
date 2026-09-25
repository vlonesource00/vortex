// Runs at physics frequency independently of tactical plans. All outputs are
// ordinary driver controls; no position correction or privileged contact rules.
import { gentleRub } from './racecraft-policy.js';
export function supervise(car,cars,current,model) {
  const sn=Math.sin(car.yaw),cs=Math.cos(car.yaw),e=model.at(car.speed,current.s,current.lateral);
  let maxSpeed=Infinity,emergency=false,reason='CLEAR';
  for(const other of cars){
    if(other===car)continue;
    const dx=other.x-car.x,dz=other.z-car.z;
    const ahead=dx*sn+dz*cs,side=dx*cs-dz*sn;
    const opponentSpeed=other.vx*sn+other.vz*cs;
    const closing=Math.max(0,car.u-opponentSpeed);
    const sideVelocity=(other.vx-car.vx)*cs-(other.vz-car.vz)*sn;
    const heading=other.yaw-car.yaw;
    const halfLength=2.3*Math.abs(Math.cos(heading))+.99*Math.abs(Math.sin(heading));
    const halfWidth=.99*Math.abs(Math.cos(heading))+2.3*Math.abs(Math.sin(heading));
    const approachingLane=Math.abs(side+sideVelocity*.3)<halfWidth+1.0;
    const penetration=halfWidth+1.0-Math.abs(side);
    // Do not turn a shallow, parallel side rub into an emergency stop. Closing
    // into the back of a car or crossing its lane still uses collision braking.
    if(ahead>0&&ahead<halfLength+2.3&&penetration>=0&&gentleRub(penetration,Math.abs(car.u-opponentSpeed),Math.abs(sideVelocity)))continue;
    if(ahead>0&&ahead<55&&(Math.abs(side)<halfWidth+1.0||approachingLane)){
      const clearance=ahead-halfLength-2.3;
      const safe=1+closing*.15+closing*closing/(2*Math.max(3,e.brake));
      maxSpeed=Math.min(maxSpeed,Math.max(0,opponentSpeed+(clearance-safe)*.8));
      if(clearance<Math.max(.25,closing*.3)){emergency=true;reason='IMMINENT CONTACT';}
    }
  }
  const outward=(car.vx*current.nx+car.vz*current.nz)*Math.sign(current.lateral);
  const edge=(model.track?.halfWidth??6.5)-.7;
  if(Math.abs(current.lateral)>edge&&outward>.1){
    maxSpeed=Math.min(maxSpeed,Math.max(10,car.speed-(Math.abs(current.lateral)-edge)*4-outward*1.5));
    if(!emergency)reason='TRACK EDGE';
  }
  return {maxSpeed,emergency,reason};
}
