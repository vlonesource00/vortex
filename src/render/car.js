import * as THREE from 'three';
import { surfaceMaps } from './surfaces.js';

const carbon = new THREE.MeshStandardMaterial({ color: '#161b1c', roughness: 0.5, metalness: 0.35 });
const rubber = new THREE.MeshStandardMaterial({ color: '#101213', roughness: 0.94 });
const alloy = new THREE.MeshStandardMaterial({ color: '#959ca0', metalness: 0.95, roughness: 0.27 });
const glass = new THREE.MeshPhysicalMaterial({ color: '#193139', metalness: 0.32, roughness: 0.09, clearcoat: 1, transparent: true, opacity: 0.88 });
const white = new THREE.MeshStandardMaterial({ color: '#eeece4', roughness: 0.35 });
let contactMaterial;
function contactShadow(){
  if(!contactMaterial){
    const size=64,data=new Uint8Array(size*size*4);
    for(let y=0;y<size;y++)for(let x=0;x<size;x++){
      const r=((x/(size-1)-.5)*2)**4+((y/(size-1)-.5)*2)**4,i=(y*size+x)*4;
      data[i+3]=Math.round(Math.max(0,1-r)*45);
    }
    const map=new THREE.DataTexture(data,size,size);map.needsUpdate=true;map.magFilter=THREE.LinearFilter;
    contactMaterial=new THREE.MeshBasicMaterial({map,transparent:true,depthWrite:false,polygonOffset:true,polygonOffsetFactor:-1});
  }
  const shadow=new THREE.Mesh(new THREE.PlaneGeometry(2.1,4.5),contactMaterial);shadow.rotation.x=-Math.PI/2;shadow.position.y=.012;return shadow;
}

function mesh(geo, mat, parent, x=0,y=0,z=0) {
  const m = new THREE.Mesh(geo,mat); m.position.set(x,y,z); m.castShadow = true; m.receiveShadow = true; parent.add(m); return m;
}
function box(parent,mat,x,y,z,w,h,d) { return mesh(new THREE.BoxGeometry(w,h,d),mat,parent,x,y,z); }
function loft(sections,segments=40) {
  const v=[],indices=[];
  for (const [z,w,bottom,top] of sections) for(let j=0;j<=segments;j++) {
    const a=j/segments*Math.PI*2;
    const x=Math.cos(a)*w;
    const yy=Math.sin(a);
    const y=yy>=0 ? bottom+(top-bottom)*(0.36+0.64*Math.pow(yy,0.45)) : bottom+(top-bottom)*0.36*(1+yy);
    v.push(x,y,z);
  }
  for(let i=0;i<sections.length-1;i++)for(let j=0;j<segments;j++){const a=i*(segments+1)+j,b=a+segments+1;indices.push(a,a+1,b,b,a+1,b+1);}
  const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(v,3));g.setIndex(indices);g.computeVertexNormals();return g;
}
function panel(parent,mat,points) {
  const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(points.flat(),3));g.setIndex([0,1,2,0,2,3]);g.computeVertexNormals();return mesh(g,mat,parent);
}
function label(text,fg,bg,w=512,h=128) {
  const canvas=document.createElement('canvas');canvas.width=w;canvas.height=h;const ctx=canvas.getContext('2d');
  ctx.fillStyle=bg;ctx.fillRect(0,0,w,h);ctx.fillStyle=fg;ctx.font=`italic 800 ${h*0.65}px Arial`;ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(text,w/2,h/2+3);
  const map=new THREE.CanvasTexture(canvas);map.colorSpace=THREE.SRGBColorSpace;map.anisotropy=8;
  return new THREE.MeshStandardMaterial({map,roughness:.48,polygonOffset:true,polygonOffsetFactor:-2,side:THREE.DoubleSide});
}

export class CarModel {
  constructor(car) {
    this.car=car;this.root=new THREE.Group();this.body=new THREE.Group();this.root.add(this.body);
    this.root.add(contactShadow());
    this.paint=new THREE.MeshPhysicalMaterial({color:car.color,metalness:0.48,roughness:0.28,clearcoat:1,clearcoatRoughness:0.10});
    if(!carbon.map){Object.assign(carbon,surfaceMaps('carbon'));carbon.bumpScale=.002;carbon.needsUpdate=true;}
    this.brakeMaterials=[];
    this.paint2=new THREE.MeshPhysicalMaterial({color:'#e7e4d9',metalness:0.24,roughness:0.32,clearcoat:1});
    mesh(loft([[-2.26,.76,.3,.67],[-2.08,.98,.27,.78],[-1.65,1.02,.27,.83],[-1.25,1.00,.29,.79],[-.65,.9,.30,.77],[.3,.89,.29,.76],[1.05,1.01,.27,.82],[1.6,1.02,.25,.75],[2.12,.91,.26,.60],[2.30,.74,.31,.48]]),this.paint,this.body);
    box(this.body,carbon,0,.255,0,1.92,.12,4.46);
    box(this.body,carbon,0,.22,2.04,2.03,.045,.59);
    box(this.body,carbon,0,.24,-2.05,1.92,.08,.64);
    // Distinct greenhouse and aero silhouettes, sharing Astra's PBR materials.
    if(car.classId==='prototype'){
      mesh(loft([[-1.5,.35,.68,.72],[-.95,.42,.70,.92],[-.45,.43,.75,1.12],[.20,.35,.75,1.10],[.76,.22,.65,.81]],32),glass,this.body);
      box(this.body,this.paint,0,1.02,-1.25,.06,.50,1.40);
      for(const side of [-1,1]){box(this.body,carbon,side*.72,.65,.78,.28,.06,.7);box(this.body,this.paint,side*.76,.48,.2,.43,.33,1.6);}
    }else if(car.classId==='touring'){
      mesh(loft([[-1.62,.77,.74,.90],[-1.26,.73,.79,1.46],[-.75,.73,.82,1.52],[.26,.70,.81,1.50],[.83,.71,.74,1.00]],32),glass,this.body);
      box(this.body,this.paint,0,1.48,-.50,1.44,.10,1.45);
      box(this.body,this.paint,0,.76,-1.72,1.74,.5,.18);
    }else{
      mesh(loft([[-1.30,.74,.68,.77],[-.92,.70,.72,1.20],[-.62,.67,.77,1.32],[.05,.63,.77,1.32],[.46,.67,.73,1.16],[.98,.76,.70,.77]],32),glass,this.body);
      mesh(loft([[-.90,.64,1.13,1.22],[-.60,.64,1.22,1.35],[.03,.60,1.23,1.35],[.27,.60,1.19,1.29]],24),this.paint,this.body);
    }
    // Windshield surround, splitter, hood stripes, diffuser, vents.
    for(const side of [-1,1]) {
      const sill=box(this.body,carbon,side*.965,.31,-.05,.1,.13,2.1);
      const mirror=mesh(new THREE.SphereGeometry(.13,16,10),this.paint,this.body,side*1.09,1.01,.46);mirror.scale.set(1.35,.55,.75);
      box(this.body,carbon,side*.94,.97,.44,.23,.03,.035);
      const door=mesh(new THREE.PlaneGeometry(1.1,.35),label(String(car.id===0?'07':21+car.id),'#141716','#eae8dd',256,128),this.body,side*1.012,.58,-.2);
      door.rotation.y=side*Math.PI/2;
      for(let i=0;i<5;i++)box(this.body,carbon,side*.59,.786,1.06+i*.065,.35,.014,.023).rotation.z=side*.06;
      for(let i=0;i<4;i++)box(this.body,carbon,side*(.25+i*.19),.24,-2.19,.025,.18,.39);
      const intake=box(this.body,carbon,side*.7,.49,2.18,.34,.15,.07);intake.rotation.y=side*.13;
      // Six LED elements in recessed black headlight housings.
      box(this.body,carbon,side*.72,.63,1.97,.4,.055,.26).rotation.x=.25;
      const led=new THREE.MeshStandardMaterial({color:'#e4f8ff',emissive:'#c4eaff',emissiveIntensity:2.4});
      for(let i=0;i<4;i++)box(this.body,led,side*(.57+i*.083),.667,2.01,.051,.018,.12);
      const red=new THREE.MeshStandardMaterial({color:'#e43d20',emissive:'#ff240c',emissiveIntensity:.9});
      box(this.body,red,side*.57,.665,-2.18,.60,.035,.025);
      this.brakeMaterials.push(red);
      const support=box(this.body,carbon,side*.6,1.02,-1.91,.045,.63,.1);support.rotation.x=-.2;
    }
    const wing=box(this.body,carbon,0,1.34,-2.00,2.12,.065,.4);wing.rotation.x=-.08;
    for(const side of [-1,1])box(this.body,this.paint,side*1.065,1.33,-2.02,.035,.23,.51);
    for(const side of [-1,1]) {
      panel(this.body,this.paint2,[[side*.07,.791,.80],[side*.07,.613,2.12],[side*.22,.613,2.12],[side*.22,.791,.80]]);
      const stripe=box(this.body,this.paint2,side*.11,1.356,-.27,.12,.009,.66);
    }
    const hoodName=mesh(new THREE.PlaneGeometry(.61,.15),label('ASTRA','#e8e5da','#c74129'),this.body,0,.78,1.22);hoodName.rotation.x=-Math.PI/2;
    // Visible cockpit: roll cage, seat, dashboard, wheel and instrument screen.
    this.interior=new THREE.Group();this.root.add(this.interior);
    box(this.interior,carbon,0,.81,.50,1.30,.15,.23);
    box(this.interior,carbon,-.32,.58,-.21,.43,.25,.51);
    box(this.interior,carbon,-.32,.88,-.45,.44,.62,.13).rotation.x=-.14;
    const screen=box(this.interior,new THREE.MeshBasicMaterial({color:'#75bfbd'}),-.31,.87,.368,.25,.12,.01);
    this.steeringWheel=mesh(new THREE.TorusGeometry(.16,.024,8,24),carbon,this.interior,-.32,.85,.16);
    box(this.steeringWheel,alloy,0,0,0,.3,.032,.024);
    for(const side of [-1,1]) {
      const cage=mesh(new THREE.CylinderGeometry(.018,.018,.70,8),white,this.interior,side*.62,1.00,-.72);cage.rotation.x=-.08;
    }
    this.wheels=[];
    car.wheels.forEach((w,i)=>{
      const discMaterial=carbon.clone();discMaterial.emissive.set('#ed4814');discMaterial.emissiveIntensity=0;
      const group=new THREE.Group();group.position.set(w.x,.345,w.z);this.root.add(group);
      const spin=new THREE.Group();group.add(spin);
      const tire=mesh(new THREE.CylinderGeometry(.338,.338,.29,40,1),rubber,spin);tire.rotation.z=Math.PI/2;
      for(const side of [-1,1]) {
        const ring=mesh(new THREE.TorusGeometry(.26,.014,8,40),rubber,spin,side*.15,0,0);ring.rotation.y=Math.PI/2;
        const rim=mesh(new THREE.CylinderGeometry(.244,.244,.016,32),alloy,spin,side*.153,0,0);rim.rotation.z=Math.PI/2;
        const inset=mesh(new THREE.CylinderGeometry(.21,.21,.018,32),discMaterial,spin,side*.163,0,0);inset.rotation.z=Math.PI/2;
        const hub=mesh(new THREE.CylinderGeometry(.055,.055,.04,12),alloy,spin,side*.177,0,0);hub.rotation.z=Math.PI/2;
        for(let j=0;j<10;j++) {
          const a=j/10*Math.PI*2;
          const spoke=box(spin,alloy,side*.181,Math.sin(a)*.13,Math.cos(a)*.13,.025,.025,.20);spoke.rotation.x=-a;
        }
      }
      const caliper=box(group,new THREE.MeshStandardMaterial({color:'#ac7521',metalness:.65,roughness:.4}),Math.sign(w.x)*.15,0,-.17,.04,.15,.085);
      this.wheels.push({group,spin,discMaterial});
    });
  }
  update(dt) {
    const car=this.car;this.root.position.set(car.x,.025,car.z);this.root.rotation.y=car.yaw;
    this.body.position.y=-car.heave;this.body.rotation.set(car.pitch,0,car.roll);
    this.steeringWheel.rotation.z=-car.steering*9;
    for(const material of this.brakeMaterials)material.emissiveIntensity=car.controls.brake>.05?3:.7;
    this.wheels.forEach((v,i)=>{v.group.rotation.y=car.wheels[i].steer;v.group.position.y=.35-(car.wheels[i].compression-.045);v.spin.rotation.x+=car.wheels[i].omega*dt;v.discMaterial.emissiveIntensity=Math.max(0,Math.min(1.6,(car.wheels[i].brakeTemp-480)/220));});
  }
  cockpit(active) { this.body.visible=!active; }
  setWheelAsset(template){
    for(const wheel of this.wheels){
      // Keep the existing brake-heat disc and caliper behind the new open spokes.
      for(const child of wheel.spin.children)child.visible=false;
      const detail=template.clone(true);detail.traverse(object=>{if(object.isMesh){object.castShadow=true;object.receiveShadow=true;}});
      wheel.spin.add(detail);
      const disc=mesh(new THREE.CylinderGeometry(.195,.195,.016,32),wheel.discMaterial,wheel.spin);disc.rotation.z=Math.PI/2;
    }
  }
  setColor(color) { this.paint.color.set(color);this.car.color=color; }
}
