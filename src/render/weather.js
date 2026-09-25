import * as THREE from 'three';
import { random } from '../sim/math.js';

// One bounded geometry buffer, no emitters or allocations during a frame.
export class WeatherEffects {
  constructor(scene,count=420){
    this.count=count;this.drops=new Float32Array(count*3);const rng=random(244);
    for(let i=0;i<count;i++){this.drops[i*3]=(rng()-.5)*90;this.drops[i*3+1]=rng()*45;this.drops[i*3+2]=(rng()-.5)*90;}
    const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.BufferAttribute(new Float32Array(count*6),3).setUsage(THREE.DynamicDrawUsage));
    this.rain=new THREE.LineSegments(g,new THREE.LineBasicMaterial({color:'#c6dce8',transparent:true,opacity:.2,depthWrite:false}));
    this.rain.frustumCulled=false;this.rain.visible=false;scene.add(this.rain);
  }
  update(car,dt,wetness){
    const amount=THREE.MathUtils.clamp(((wetness||0)-.5)*3,0,1);this.rain.visible=amount>0;if(!amount)return;
    this.rain.material.opacity=.1+amount*.15;const pos=this.rain.geometry.attributes.position;
    const count=Math.floor(this.count*amount);this.rain.geometry.setDrawRange(0,count*2);
    for(let i=0;i<count;i++){
      const k=i*3;this.drops[k+1]=(this.drops[k+1]-dt*28+45)%45;
      const x=car.x+this.drops[k],y=this.drops[k+1],z=car.z+this.drops[k+2];
      pos.setXYZ(i*2,x,y,z);pos.setXYZ(i*2+1,x+.18,y-1.1,z+.05);
    }
    pos.needsUpdate=true;
  }
}
