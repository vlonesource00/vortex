import { clamp, damp, move } from './math.js';
import { SPEC } from './vehicle.js';
import { gamepadState, GAMEPAD_ACTIONS } from './gamepad.js';

export class Keyboard {
  constructor(onAction) {
    this.onAction=onAction;this.pad=null;this.previousButtons=null;this.connected=false;
    this.keys = new Set(); this.throttle = 0; this.brake = 0; this.steer = 0;
    window.addEventListener('keydown', e => {
      if (/INPUT|SELECT|TEXTAREA/.test(e.target.tagName)) return;
      if (['ArrowUp','ArrowDown','ArrowLeft','ArrowRight','Space'].includes(e.code)) e.preventDefault();
      this.keys.add(e.code);
      if (!e.repeat) onAction(e.code);
    });
    window.addEventListener('keyup', e => this.keys.delete(e.code));
    window.addEventListener('blur', () => this.clear());
  }
  clear() { this.keys.clear(); this.throttle = this.brake = this.steer = 0; }
  down(...keys) { return keys.some(k => this.keys.has(k)); }
  pollGamepad(){
    const pads=navigator.getGamepads?.()??[];
    const pad=Array.from(pads).find(p=>p?.connected&&p.mapping==='standard');
    const state=gamepadState(pad);
    if(!state){if(this.connected){this.clear();this.onAction('GamepadLost');}this.connected=false;this.pad=null;this.previousButtons=null;return;}
    this.connected=true;this.pad=state;
    if(typeof document!=='undefined'&&(document.hidden||!document.hasFocus())){this.pad=null;this.previousButtons=state.buttons;return;}
    if(this.previousButtons)for(const [index,action] of Object.entries(GAMEPAD_ACTIONS))if(state.buttons[index]&&!this.previousButtons[index])this.onAction(action);
    this.previousButtons=state.buttons;
  }
  update(car, dt, assisted = true) {
    const SPEC=car.spec;
    // Facing +Z with +Y up, positive yaw turns to the driver's LEFT
    // (+local X), which projects left in the driving camera.
    const keyboardDirection = Number(this.down('KeyA','ArrowLeft')) - Number(this.down('KeyD','ArrowRight'));
    const direction=keyboardDirection||this.pad?.steer||0;
    const speed = Math.max(3, car.speed);
    const gripBudget = 11.3 + car.aero.downforce / (SPEC.mass + car.fuel) * 0.8;
    const lock = assisted ? clamp(Math.atan(SPEC.wheelbase * gripBudget / (speed * speed)) / SPEC.steeringLock, 0.065, 1) : 1;
    const slip = Math.atan2(car.v, Math.max(5, car.u));
    const counter = assisted ? clamp(slip * 0.55 - car.yawRate * 0.06, -lock * 0.4, lock * 0.4) : 0;
    this.steer = move(this.steer, clamp(direction * lock + counter, -1, 1), dt * (direction ? 2.8 : 4.2));
    this.throttle = damp(this.throttle, Math.max(Number(this.down('KeyW','ArrowUp')),this.pad?.throttle??0), 9, dt);
    this.brake = damp(this.brake, Math.max(Number(this.down('KeyS','ArrowDown')),this.pad?.brake??0), 12, dt);
    return { throttle: this.throttle, brake: this.brake, steer: this.steer,reverse:Boolean(this.pad?.reverse&&car.u<2) };
  }
}
