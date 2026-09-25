import { clamp } from './math.js';
export function gamepadState(pad,deadzone=.12){
  if(!pad?.connected||pad.mapping!=='standard')return null;
  const axis=Number.isFinite(pad.axes[0])?clamp(pad.axes[0],-1,1):0;
  const steering=Math.abs(axis)<=deadzone?0:Math.sign(axis)*(Math.abs(axis)-deadzone)/(1-deadzone);
  const button=i=>clamp(Number(pad.buttons[i]?.value)||0,0,1);
  return {steer:steering===0?0:-steering,throttle:button(7),brake:button(6),reverse:button(2)>.5,
    buttons:pad.buttons.map(b=>Boolean(b.pressed||b.value>.5))};
}
export const GAMEPAD_ACTIONS={0:'Enter',3:'KeyC',4:'KeyQ',5:'KeyE',9:'Escape',12:'PadUp',13:'PadDown',14:'PadLeft',15:'PadRight'};
