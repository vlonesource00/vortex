import { clamp } from '../sim/math.js';

export class AudioEngine {
  constructor(){this.volume=.36;this.enabled=true;}
  async unlock(){
    if(this.ctx){await this.ctx.resume();return;}
    const C=window.AudioContext||window.webkitAudioContext;if(!C)return;
    this.ctx=new C();const c=this.ctx;
    this.master=c.createGain();this.master.gain.value=this.volume;
    const compressor=c.createDynamicsCompressor();compressor.threshold.value=-18;compressor.ratio.value=4;compressor.attack.value=.004;compressor.release.value=.18;
    this.master.connect(compressor);compressor.connect(c.destination);
    this.filter=c.createBiquadFilter();this.filter.type='lowpass';this.filter.frequency.value=900;this.filter.Q.value=.7;this.filter.connect(this.master);
    this.engine=c.createGain();this.engine.gain.value=0;this.engine.connect(this.filter);
    this.osc=[];
    for(let i=0;i<4;i++){const o=c.createOscillator();o.type=i%2?'triangle':'sawtooth';const g=c.createGain();g.gain.value=.13/(i+1);o.connect(g);g.connect(this.engine);o.start();this.osc.push(o);}
    const buffer=c.createBuffer(1,c.sampleRate*2,c.sampleRate),data=buffer.getChannelData(0);for(let i=0;i<data.length;i++)data[i]=Math.random()*2-1;
    const noise=c.createBufferSource();noise.buffer=buffer;noise.loop=true;
    this.noiseFilter=c.createBiquadFilter();this.noiseFilter.type='bandpass';this.noiseFilter.frequency.value=1800;this.noiseFilter.Q.value=1.5;
    this.tyres=c.createGain();this.tyres.gain.value=0;noise.connect(this.noiseFilter);this.noiseFilter.connect(this.tyres);this.tyres.connect(this.master);noise.start();
    this.noiseBuffer=buffer;
    this.wind=this.noiseLayer('lowpass',420,.7);
    this.road=this.noiseLayer('bandpass',280,2);
    this.gravel=this.noiseLayer('highpass',750,.7);
    this.transmission=c.createOscillator();this.transmission.type='sine';this.whine=c.createGain();this.whine.gain.value=0;this.transmission.connect(this.whine);this.whine.connect(this.master);this.transmission.start();
    this.rivals=Array.from({length:3},()=>{
      const oscillator=c.createOscillator(),gain=c.createGain(),pan=c.createStereoPanner(),filter=c.createBiquadFilter();
      oscillator.type='sawtooth';filter.type='lowpass';filter.frequency.value=650;gain.gain.value=0;
      oscillator.connect(filter);filter.connect(gain);gain.connect(pan);pan.connect(this.master);oscillator.start();return {oscillator,gain,pan,filter,id:null};
    });
    this.lastGear=null;this.lastImpact=0;this.lastThrottle=0;this.lastPop=-1;this.wasRunning=false;
    await c.resume();
  }
  noiseLayer(type,frequency,q){
    const c=this.ctx,source=c.createBufferSource(),filter=c.createBiquadFilter(),gain=c.createGain();
    source.buffer=this.noiseBuffer;source.loop=true;filter.type=type;filter.frequency.value=frequency;filter.Q.value=q;gain.gain.value=0;
    source.connect(filter);filter.connect(gain);gain.connect(this.master);source.start();return {gain,filter};
  }
  transient(frequency,strength,duration=.12){
    const c=this.ctx,t=c.currentTime,source=c.createBufferSource(),filter=c.createBiquadFilter(),gain=c.createGain();
    source.buffer=this.noiseBuffer;filter.type='lowpass';filter.frequency.value=frequency;
    gain.gain.setValueAtTime(Math.max(.001,strength),t);gain.gain.exponentialRampToValueAtTime(.001,t+duration);
    source.connect(filter);filter.connect(gain);gain.connect(this.master);source.start(t);source.stop(t+duration);
    source.onended=()=>{source.disconnect();filter.disconnect();gain.disconnect();};
  }
  setVolume(v){this.volume=v;if(this.master)this.master.gain.setTargetAtTime(this.enabled?v:0,this.ctx.currentTime,.08);}
  toggle(){this.enabled=!this.enabled;this.setVolume(this.volume);return this.enabled;}
  update(car,running,cars=[]){
    if(!this.ctx)return;const t=this.ctx.currentTime;
    this.engine.gain.setTargetAtTime(running?.18+car.controls.throttle*.65:0,t,.05);
    this.osc.forEach((o,i)=>o.frequency.setTargetAtTime((car.rpm/60*2)*(i+1)*(i%2?1.004:1),t,.035));
    this.filter.frequency.setTargetAtTime(450+car.rpm*.16+car.controls.throttle*1300,t,.04);
    const slip=Math.max(...car.wheels.map(w=>Math.abs(w.tyre.alpha)+Math.abs(w.tyre.kappa)*.25));
    this.tyres.gain.setTargetAtTime(running?clamp((slip-.10)*.55,0,.16)*clamp(car.speed/10,0,1):0,t,.03);
    this.noiseFilter.frequency.setTargetAtTime(1100+clamp(slip,0,1)*1400,t,.08);
    const speed=clamp(car.speed/75,0,1),surface=car.zone;
    this.wind.gain.gain.setTargetAtTime(running?speed*speed*.20:0,t,.15);
    this.road.gain.gain.setTargetAtTime(running?(surface==='kerb'?.16:.028)*speed:0,t,.035);
    this.road.filter.frequency.setTargetAtTime(120+car.speed*8,t,.05);
    this.gravel.gain.gain.setTargetAtTime(running&&surface!=='asphalt'&&surface!=='kerb'?speed*.22:0,t,.05);
    this.whine.gain.setTargetAtTime(running?speed*.022*(1-car.controls.throttle*.5):0,t,.05);
    this.transmission.frequency.setTargetAtTime(180+car.speed*19,t,.04);
    if(running&&this.wasRunning){
      if(car.gear!==this.lastGear&&car.speed>5)this.transient(car.gear>this.lastGear?180:340,.16,.09);
      if(car.impact>this.lastImpact+.015)this.transient(450,clamp(car.impact*.7,.08,.5),.18);
      if(this.lastThrottle>.65&&car.controls.throttle<.2&&car.rpm>4600&&t-this.lastPop>.5){this.transient(260,.12,.085);this.lastPop=t;}
    }
    this.lastGear=car.gear;this.lastImpact=car.impact;this.lastThrottle=car.controls.throttle;this.wasRunning=running;
    const nearby=cars.filter(c=>c!==car).map(c=>({car:c,distance:Math.hypot(c.x-car.x,c.z-car.z)})).filter(c=>c.distance<90).sort((a,b)=>a.distance-b.distance).slice(0,3);
    // Keep voices assigned to the same opponent while it stays nearby.
    for(const voice of this.rivals)if(!nearby.some(n=>n.car.id===voice.id))voice.id=null;
    for(const n of nearby)if(!this.rivals.some(v=>v.id===n.car.id)){const free=this.rivals.find(v=>v.id===null);if(free)free.id=n.car.id;}
    for(const voice of this.rivals){
      const n=nearby.find(n=>n.car.id===voice.id);
      voice.gain.gain.setTargetAtTime(running&&n?.car.speed>2?.07/(1+n.distance*.09):0,t,.09);
      if(!n)continue;
      const dx=n.car.x-car.x,dz=n.car.z-car.z,d=Math.max(1,n.distance);
      const closing=((car.vx-n.car.vx)*dx+(car.vz-n.car.vz)*dz)/d;
      voice.oscillator.frequency.setTargetAtTime(n.car.rpm/30*clamp(343/(343-closing),.8,1.25),t,.05);
      voice.filter.frequency.setTargetAtTime(400+2200/(1+d*.08),t,.1);
      voice.pan.pan.setTargetAtTime(clamp((-dx*Math.cos(car.yaw)+dz*Math.sin(car.yaw))/d,-1,1),t,.08);
    }
  }
}
