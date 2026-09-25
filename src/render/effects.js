import * as THREE from 'three';
import { clamp } from '../sim/math.js';

// Fixed pools keep a long race bounded: one draw call for tyre marks, one for
// transient smoke, dust and exhaust. Marks follow contact patches, not a spline.
export class CarEffects {
  constructor(scene,capacity=48000){
    this.capacity=capacity;this.cursor=0;this.count=0;this.previous=new Map();this.timer=0;
    const geometry=new THREE.BufferGeometry();
    geometry.setAttribute('position',new THREE.BufferAttribute(new Float32Array(capacity*18),3).setUsage(THREE.DynamicDrawUsage));
    geometry.setAttribute('color',new THREE.BufferAttribute(new Float32Array(capacity*24),4).setUsage(THREE.DynamicDrawUsage));
    geometry.setDrawRange(0,0);
    this.marks=new THREE.Mesh(geometry,new THREE.MeshBasicMaterial({vertexColors:true,transparent:true,depthWrite:false,side:THREE.DoubleSide,polygonOffset:true,polygonOffsetFactor:-1,polygonOffsetUnits:-1}));
    this.marks.frustumCulled=false;this.marks.renderOrder=2;scene.add(this.marks);
    this.particles=Array.from({length:384},()=>({life:0}));this.nextParticle=0;
    const g=new THREE.BufferGeometry();
    for(const [name,size] of [['position',3],['tint',4],['size',1]])g.setAttribute(name,new THREE.BufferAttribute(new Float32Array(384*size),size).setUsage(THREE.DynamicDrawUsage));
    this.cloud=new THREE.Points(g,new THREE.ShaderMaterial({transparent:true,depthWrite:false,
      uniforms:{pixelScale:{value:400}},
      vertexShader:`attribute vec4 tint; attribute float size; varying vec4 vTint; uniform float pixelScale;
        void main(){vTint=tint;vec4 p=modelViewMatrix*vec4(position,1.);gl_Position=projectionMatrix*p;gl_PointSize=clamp(size*pixelScale/max(1.,-p.z),0.,90.);}`,
      fragmentShader:`varying vec4 vTint;void main(){float r=length(gl_PointCoord-.5)*2.;float a=(1.-smoothstep(.1,1.,r))*vTint.a;if(a<.005)discard;gl_FragColor=vec4(vTint.rgb,a);
        #include <tonemapping_fragment>
        #include <colorspace_fragment>
      }`}));
    this.cloud.frustumCulled=false;scene.add(this.cloud);
  }
  reset(){this.previous.clear();this.count=this.cursor=0;this.marks.geometry.setDrawRange(0,0);for(const p of this.particles)p.life=0;}
  emit(x,y,z,color,size,life,vx=0,vz=0){Object.assign(this.particles[this.nextParticle++%384],{x,y,z,color,size,life,maxLife:life,vx,vz});}
  segment(a,b,width,opacity){
    const dx=b.x-a.x,dz=b.z-a.z,length=Math.hypot(dx,dz);
    if(length<.035||length>5)return false;
    const nx=dz/length*width/2,nz=-dx/length*width/2;
    const corners=[[a.x-nx,a.z-nz],[a.x+nx,a.z+nz],[b.x-nx,b.z-nz],[b.x+nx,b.z+nz]];
    const g=this.marks.geometry,base=this.cursor*6;
    [0,2,1,1,2,3].forEach((index,j)=>{
      g.attributes.position.setXYZ(base+j,corners[index][0],.042,corners[index][1]);
      g.attributes.color.setXYZW(base+j,.025,.022,.019,opacity);
    });
    g.attributes.position.addUpdateRange(base*3,18);g.attributes.color.addUpdateRange(base*4,24);
    this.cursor=(this.cursor+1)%this.capacity;this.count=Math.min(this.capacity,this.count+1);return true;
  }
  update(cars,dt,track,pixelHeight=800){
    this.cloud.material.uniforms.pixelScale.value=pixelHeight*.65;
    this.timer+=dt;const sample=this.timer>=.05;
    if(sample){
      this.timer%=.05;
      const g=this.marks.geometry;let changed=false;
      for(const car of cars){
        const c=Math.cos(car.yaw),s=Math.sin(car.yaw);
        let previous=this.previous.get(car.id);
        if(!previous){previous={wheels:[],gear:car.gear};this.previous.set(car.id,previous);}
        car.wheels.forEach((w,i)=>{
          const point={x:car.x+c*w.x+s*w.z,z:car.z-s*w.x+c*w.z};
          const surface=track.surface(point.x,point.z),slip=Math.abs(w.tyre.alpha)+Math.abs(w.tyre.kappa)*.6;
          if(surface.zone==='asphalt'&&w.load>100&&car.speed>2&&slip>.035&&previous.wheels[i]){
            const opacity=clamp(.025+slip*.65,.025,.42);
            changed=this.segment(previous.wheels[i],point,.27,opacity)||changed;
          }
          if(car.speed>8&&w.load>100){
            if(surface.zone!=='asphalt'&&surface.zone!=='kerb')this.emit(point.x,.13,point.z,[.54,.44,.30],.6,1.8,car.vx*.06,car.vz*.06);
            else if((track.wetness||0)>.15&&car.speed>14)this.emit(point.x,.14,point.z,[.72,.81,.85],.25+track.wetness*.5,.7+track.wetness*.4,-car.vx*.12,-car.vz*.12);
            else if(slip>.24&&w.tyre.slipPower>3000)this.emit(point.x,.16,point.z,[.64,.67,.68],.35,1.25,car.vx*.04,car.vz*.04);
          }
          previous.wheels[i]=point;
        });
        if(car.gear>previous.gear&&car.controls.throttle>.4&&car.speed>12)this.emit(car.x-s*2.3,.35,car.z-c*2.3,[1,.32,.055],.42,.13,-s*2,-c*2);
        previous.gear=car.gear;
      }
      if(changed){g.attributes.position.needsUpdate=true;g.attributes.color.needsUpdate=true;g.setDrawRange(0,this.count*6);}
    }
    const g=this.cloud.geometry;
    for(let i=0;i<this.particles.length;i++){
      const p=this.particles[i];p.life=Math.max(0,p.life-dt);
      if(p.life>0){p.x+=p.vx*dt;p.z+=p.vz*dt;p.y+=dt*.35;const age=1-p.life/p.maxLife;
        g.attributes.position.setXYZ(i,p.x,p.y,p.z);g.attributes.tint.setXYZW(i,...p.color,(1-age)*.28);g.attributes.size.setX(i,p.size*(1+age*3));
      }else{g.attributes.size.setX(i,0);g.attributes.tint.setW(i,0);}
    }
    for(const attribute of Object.values(g.attributes))attribute.needsUpdate=true;
  }
}
