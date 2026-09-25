import { clamp } from './math.js';
import { PACE } from './pace.js';

// One pedal mapping for the executed action and every predictive rollout.
// Bias is a bounded acceleration request, not an extra force on the vehicle.
export function pedals(error, freedom, envelope, bias=0) {
  let throttle=clamp(error*(.34+(PACE.throttleGain-.34)*freedom)+.1+(PACE.throttleBase-.1)*freedom,0,1);
  let brake=clamp(-error*(.22+(PACE.brakeGain-.22)*freedom),0,1);
  if(bias<0) {
    const reduction=-bias/Math.max(1,envelope.drive);
    const remaining=Math.max(0,reduction-throttle);
    throttle=Math.max(0,throttle-reduction);
    brake=clamp(brake+remaining*Math.max(1,envelope.drive)/Math.max(1,envelope.brake),0,1);
  } else if(error>0&&brake===0) throttle=clamp(throttle+bias/Math.max(1,envelope.drive),0,1);
  // Avoid heating the tyres by asking for propulsion and braking together.
  if(brake>.05)throttle=0;
  throttle=Math.min(throttle,envelope.throttleLimit??1);
  // Tiny overspeed can dissipate through drag. Larger speed deficits retain
  // the ordinary braking law; the collision supervisor still overrides it.
  if(error<0&&error>-(envelope.coastWindow??0)&&brake<.25){throttle=0;brake=0;}
  return {throttle,brake};
}

export function pedalAcceleration(controls,envelope) {
  const drag=envelope.drag??0;
  return controls.throttle*((envelope.actuationDrive??envelope.drive)+drag)-drag-controls.brake*envelope.brake;
}
