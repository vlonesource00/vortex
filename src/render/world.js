import * as THREE from 'three';
import { Sky } from 'three/addons/objects/Sky.js';
import { random, clamp } from '../sim/math.js';
import { RivieraScenery } from './scenery.js';
import { surfaceMaps, LIGHTING, wetSurface } from './surfaces.js';
import { TracksideLife } from './trackside-life.js';
import { HarborScenery } from './harbor-scenery.js';

const mat=(color,roughness=.85,metalness=0)=>new THREE.MeshStandardMaterial({color,roughness,metalness});
function add(parent,geo,material,x=0,y=0,z=0){const m=new THREE.Mesh(geo,material);m.position.set(x,y,z);m.castShadow=true;m.receiveShadow=true;parent.add(m);return m;}
function box(p,m,x,y,z,w,h,d){return add(p,new THREE.BoxGeometry(w,h,d),m,x,y,z);}
function noiseTexture(size,base,range,seed=5){
  const canvas=document.createElement('canvas');canvas.width=canvas.height=size;const c=canvas.getContext('2d'),img=c.createImageData(size,size),rng=random(seed);
  for(let i=0;i<img.data.length;i+=4){const n=(rng()-.5)*range;for(let k=0;k<3;k++)img.data[i+k]=base[k]+n;img.data[i+3]=255;}c.putImageData(img,0,0);
  const map=new THREE.CanvasTexture(canvas);map.wrapS=map.wrapT=THREE.RepeatWrapping;map.colorSpace=THREE.SRGBColorSpace;map.anisotropy=8;return map;
}
export function ribbon(track,left,right,height=.02,steps=960){
  const pos=[],uv=[],idx=[];
  for(let i=0;i<=steps;i++){const s=i/steps*track.length;for(const side of [left,right]){const p=track.at(s,side);pos.push(p.x,height,p.z);uv.push((side-left)/4,s/4);}}
  for(let i=0;i<steps;i++){
    const a=i*2;
    if(right>left)idx.push(a,a+2,a+1,a+1,a+2,a+3);
    else idx.push(a,a+1,a+2,a+1,a+3,a+2);
  }
  const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(pos,3));g.setAttribute('uv',new THREE.Float32BufferAttribute(uv,2));g.setIndex(idx);g.computeVertexNormals();return g;
}
function signTexture(text,sub='',dark=true){
  const c=document.createElement('canvas');c.width=1024;c.height=256;const x=c.getContext('2d');
  x.fillStyle=dark?'#171e1f':'#e8e4d8';x.fillRect(0,0,1024,256);x.fillStyle='#ec502e';x.fillRect(0,0,16,256);
  x.fillStyle=dark?'#efeee5':'#202825';x.font='italic 800 112px Arial';x.textAlign='center';x.fillText(text,512,sub?133:166);
  if(sub){x.font='24px Arial';x.fillStyle='#a9b0a7';x.fillText(sub,512,205);}
  const map=new THREE.CanvasTexture(c);map.colorSpace=THREE.SRGBColorSpace;map.anisotropy=8;return new THREE.MeshStandardMaterial({map,roughness:.78,side:THREE.DoubleSide});
}

function foliageTexture(seed=57){
  const canvas=document.createElement('canvas');canvas.width=canvas.height=1024;
  const c=canvas.getContext('2d'),rng=random(seed);
  c.lineCap='round';
  c.strokeStyle='#554d37';c.lineWidth=23;c.beginPath();c.moveTo(512,1000);c.bezierCurveTo(498,850,520,580,494,350);c.stroke();
  const crowns=[];
  for(let i=0;i<32;i++){
    const a=rng()*Math.PI*2,rad=90+rng()*225,x=512+Math.cos(a)*rad,y=420+Math.sin(a)*rad*.83;
    c.lineWidth=4+rng()*8;c.strokeStyle='#514e36';c.beginPath();c.moveTo(511,780);c.quadraticCurveTo(510,550,x,y);c.stroke();crowns.push({x,y,r:95+rng()*54});
  }
  crowns.sort((a,b)=>b.y-a.y);
  for(const crown of crowns){
    for(let j=0;j<1050;j++){
      const a=rng()*Math.PI*2,r=Math.sqrt(rng())*crown.r,x=crown.x+Math.cos(a)*r,y=crown.y+Math.sin(a)*r*.8;
      const light=23+rng()*21+(1-r/crown.r)*5;
      c.fillStyle=`hsl(${69+rng()*24},${18+rng()*17}%,${light}%)`;
      c.beginPath();c.ellipse(x,y,2+rng()*6,2+rng()*3,rng()*3,0,Math.PI*2);c.fill();
    }
  }
  const map=new THREE.CanvasTexture(canvas);map.colorSpace=THREE.SRGBColorSpace;map.anisotropy=4;return map;
}

export class World {
  constructor(scene,renderer,track){
    this.scene=scene;this.renderer=renderer;this.track=track;this.root=new THREE.Group();scene.add(this.root);
    scene.fog=new THREE.FogExp2('#c6caba',.00065);
    this.sky=new Sky();this.sky.scale.setScalar(12000);scene.add(this.sky);
    const u=this.sky.material.uniforms;u.turbidity.value=3.8;u.rayleigh.value=1.5;u.mieCoefficient.value=.006;u.mieDirectionalG.value=.86;
    this.sun=new THREE.DirectionalLight('#fff0d2',3.8);this.sun.castShadow=true;this.sun.shadow.mapSize.set(2048,2048);
    Object.assign(this.sun.shadow.camera,{left:-52,right:52,top:52,bottom:-52,near:1,far:300});
    this.sun.shadow.bias=-.00015;this.sun.shadow.normalBias=.025;this.sun.shadow.radius=2;
    scene.add(this.sun,this.sun.target);this.hemi=new THREE.HemisphereLight('#c5dbe5','#746646',1.5);scene.add(this.hemi);
    this.pmrem=new THREE.PMREMGenerator(renderer);this.setLighting('golden');
    this.buildTrack();this.buildLandscape();this.buildPaddock();this.buildDetails();
    this.riviera=track.id==='harbor-ring'?new HarborScenery(this.root,track):new RivieraScenery(this.root,track);
    this.tracksideLife=new TracksideLife(this.root,track);
    // Circuit scenery is static. Material and rubber-buffer updates remain live.
    this.root.traverse(object=>{object.updateMatrix();object.matrixAutoUpdate=false;});
    this.centre=new THREE.Vector3();this.shadowRight=new THREE.Vector3();this.shadowUp=new THREE.Vector3();
  }
  setLighting(mode){
    this.lightMode=mode;
    const p=LIGHTING[mode]||LIGHTING.golden;
    this.sunDirection=new THREE.Vector3(.7,p.elevation,.65).normalize();
    this.sky.material.uniforms.sunPosition.value.copy(this.sunDirection);
    this.sky.material.uniforms.turbidity.value=p.turbidity;
    this.sky.material.uniforms.rayleigh.value=mode==='overcast'?.35:1.7;
    this.sun.intensity=p.intensity;this.hemi.intensity=p.fill;
    this.sun.color.set(p.sun);this.hemi.color.set(p.sky);this.hemi.groundColor.set(p.ground);
    this.scene.fog.color.set(p.fog);this.scene.fog.density=p.density;this.renderer.toneMappingExposure=p.exposure;
    const environmentScene=new THREE.Scene();const sky=this.sky.clone();environmentScene.add(sky);
    const env=this.pmrem.fromScene(environmentScene,0.03);this.envTarget?.dispose();this.envTarget=env;this.scene.environment=env.texture;this.scene.environmentIntensity=p.environment;
  }
  buildTrack(){
    const t=this.track;
    this.roadMaterial=new THREE.MeshPhysicalMaterial({...surfaceMaps('asphalt'),bumpScale:.004,roughness:1,metalness:0,clearcoat:0,clearcoatRoughness:.12});
    this.road=add(this.root,ribbon(t,-t.halfWidth,t.halfWidth,.018),this.roadMaterial);this.road.castShadow=false;
    for(const side of [-1,1]){
      const runoff=add(this.root,ribbon(t,side*(t.halfWidth+t.curbWidth),side*(t.barrierOffset-1),.005),new THREE.MeshStandardMaterial({...surfaceMaps('gravel'),bumpScale:.065,roughness:1,side:THREE.DoubleSide}));runoff.castShadow=false;
      add(this.root,ribbon(t,side*(t.halfWidth-.22),side*(t.halfWidth-.11),.028),mat('#e2e1d3')).castShadow=false;
    }
    const curbWhite=new THREE.MeshStandardMaterial({color:'#dddcd1',bumpMap:surfaceMaps('asphalt').bumpMap,bumpScale:.006,roughness:.83}),curbRed=curbWhite.clone();curbRed.color.set('#b83a28');
    const dummy=new THREE.Object3D(),count=Math.ceil(t.length/2.5);
    for(const side of [-1,1])for(let parity=0;parity<2;parity++){
      const mesh=new THREE.InstancedMesh(new THREE.BoxGeometry(t.curbWidth,.06,2.55),parity?curbRed:curbWhite,Math.ceil(count/2));let k=0;
      for(let i=parity;i<count;i+=2){const p=t.at(i/count*t.length,side*(t.halfWidth+t.curbWidth/2));dummy.position.set(p.x,.025,p.z);dummy.rotation.set(0,p.heading,0);dummy.updateMatrix();mesh.setMatrixAt(k++,dummy.matrix);}mesh.count=k;mesh.receiveShadow=true;this.root.add(mesh);
    }
    // The circuit starts clean; contact-patch trails are drawn by CarEffects.
    const positions=[],colors=[];
    for(let i=0;i<t.nodes.length;i++)for(let lane=0;lane<13;lane++){
      const left=lane*t.laneWidth-t.halfWidth,right=left+t.laneWidth;
      const a=t.at(t.nodes[i].s,left),b=t.at(t.nodes[i].s,right),next=t.nodes[(i+1)%t.nodes.length].s;
      const c=t.at(next,left),d=t.at(next,right);
      for(const p of [a,b,c,b,d,c]){positions.push(p.x,.036,p.z);colors.push(1,1,1);}
    }
    const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(positions,3));g.setAttribute('color',new THREE.Float32BufferAttribute(colors,3));
    this.rubberMesh=add(this.root,g,new THREE.MeshBasicMaterial({vertexColors:true,transparent:true,opacity:.26,blending:THREE.MultiplyBlending,depthWrite:false,side:THREE.DoubleSide}));this.rubberMesh.castShadow=false;
    // Start/finish is at race grid origin s=90.
    const p=t.at(t.finishS),start=new THREE.Group();start.position.set(p.x,.035,p.z);start.rotation.y=p.heading;this.root.add(start);
    for(let x=0;x<16;x++)for(let z=0;z<2;z++)box(start,mat((x+z)%2?'#252925':'#dddccf'),-t.halfWidth+(x+.5)*t.width/16,.002,z*.6,t.width/16,.01,.6).castShadow=false;
    for(let i=0;i<8;i++){const p=t.at(t.gridS-Math.floor(i/2)*(t.scenario?.start.rowSpacingM??9.5),(i%2?-1:1)*(t.scenario?.start.laneOffsetM??2.3));const g=new THREE.Group();g.position.set(p.x,.04,p.z);g.rotation.y=p.heading;this.root.add(g);for(const x of [-1.1,1.1])box(g,curbWhite,x,0,-.5,.08,.01,4.8);box(g,curbWhite,0,0,1.9,2.2,.01,.08);}
  }
  buildLandscape(){
    const rng=random(414),grassMaps={};
    for(const [key,value] of Object.entries(surfaceMaps('grass'))){grassMaps[key]=value.clone();grassMaps[key].repeat.set(460,460);}
    // Keep the terrain behind the millimetre-separated circuit layers even at
    // grazing camera angles where a conventional depth buffer loses precision.
    const groundMaterial=new THREE.MeshStandardMaterial({...grassMaps,bumpScale:.025,roughness:1,polygonOffset:true,polygonOffsetFactor:1,polygonOffsetUnits:4});
    groundMaterial.onBeforeCompile=shader=>{
      shader.vertexShader='varying vec3 terrainPosition;\n'+shader.vertexShader;
      shader.vertexShader=shader.vertexShader.replace('#include <worldpos_vertex>','#include <worldpos_vertex>\nterrainPosition=(modelMatrix*vec4(transformed,1.)).xyz;');
      shader.fragmentShader='varying vec3 terrainPosition;\n'+shader.fragmentShader;
      shader.fragmentShader=shader.fragmentShader.replace('#include <color_fragment>','#include <color_fragment>\nfloat terrainPatch=sin(terrainPosition.x*.013+sin(terrainPosition.z*.009)*2.)*sin(terrainPosition.z*.017);diffuseColor.rgb*=mix(vec3(.72,.8,.66),vec3(1.15,1.04,.86),terrainPatch*.5+.5);');
    };
    const ground=add(this.root,new THREE.PlaneGeometry(6500,6500),groundMaterial,0,-.055,0);ground.rotation.x=-Math.PI/2;ground.castShadow=false;
    // Distant ridge line with layered atmospheric silhouettes.
    for(let layer=0;layer<(this.track.id==='harbor-ring'?0:3);layer++){
      const v=[],idx=[];const radius=1350+layer*480;
      for(let i=0;i<=100;i++){const a=i/100*Math.PI*2;const h=100+Math.sin(a*5+layer)*50+Math.sin(a*11+layer)*28+rng()*28;v.push(Math.cos(a)*radius,-5,Math.sin(a)*radius,Math.cos(a)*radius,h+layer*55,Math.sin(a)*radius);}
      for(let i=0;i<100;i++)idx.push(i*2,i*2+1,i*2+2,i*2+1,i*2+3,i*2+2);
      const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(v,3));g.setIndex(idx);g.computeVertexNormals();add(this.root,g,new THREE.MeshStandardMaterial({color:['#778477','#8e9b90','#a2aca0'][layer],roughness:1,side:THREE.DoubleSide})).castShadow=false;
    }
    const treeMap=foliageTexture(),treeMaterial=new THREE.MeshStandardMaterial({map:treeMap,alphaTest:.45,side:THREE.DoubleSide,roughness:1,color:'#e0e5c8',emissive:'#24301b',emissiveIntensity:.25});
    const trees=new THREE.InstancedMesh(new THREE.PlaneGeometry(13,14),treeMaterial,2300),dummy=new THREE.Object3D();
    let k=0;
    for(let i=0;i<2400&&k<1150;i++){
      const x=(rng()-.5)*2500,z=(rng()-.5)*2200;
      if(this.track.id==='harbor-ring'&&(x>550||rng()<.75))continue;
      if(Math.abs(x)<650&&Math.abs(z)<520&&Math.abs(this.track.nearest(x,z).lateral)<52)continue;
      const scale=.65+rng()*1.1;
      dummy.position.set(x,7*scale,z);dummy.rotation.set(0,rng()*6.28,0);dummy.scale.set(scale,scale,scale);dummy.updateMatrix();trees.setMatrixAt(k*2,dummy.matrix);
      const tint=new THREE.Color().setHSL(.17+rng()*.04,.08,.70+rng()*.23);trees.setColorAt(k*2,tint);
      dummy.rotation.y+=Math.PI/2;dummy.updateMatrix();trees.setMatrixAt(k*2+1,dummy.matrix);trees.setColorAt(k*2+1,tint);k++;
    }
    trees.count=k*2;trees.castShadow=true;trees.receiveShadow=true;this.root.add(trees);
    // Low, varied verge grass is instanced separately from the tree canopy.
    const bladeGeometry=new THREE.BufferGeometry();bladeGeometry.setAttribute('position',new THREE.Float32BufferAttribute([-.035,0,0,.035,0,0,.02,.45,.03],3));bladeGeometry.computeVertexNormals();
    const blades=new THREE.InstancedMesh(bladeGeometry,new THREE.MeshStandardMaterial({color:'#979363',roughness:1,side:THREE.DoubleSide}),16000);
    for(let i=0;i<16000;i++){const p=this.track.at(rng()*this.track.length,(rng()<.5?-1:1)*(18+rng()*23));dummy.position.set(p.x,-.03,p.z);dummy.rotation.y=rng()*6.28;dummy.scale.setScalar(.5+rng()*1.2);dummy.updateMatrix();blades.setMatrixAt(i,dummy.matrix);blades.setColorAt(i,new THREE.Color().setHSL(.16+rng()*.04,.20+rng()*.15,.29+rng()*.14));}this.root.add(blades);
  }
  buildPaddock(){
    const t=this.track,p=t.at(210,-31),pits=new THREE.Group();pits.position.set(p.x,0,p.z);pits.rotation.y=p.heading;this.root.add(pits);
    const concrete=mat('#c6c5b8'),dark=mat('#28312e'),metal=mat('#8e9691',.4,.65),glass=mat('#4b7278',.2,.7);
    box(pits,concrete,0,.03,0,25,.08,186);
    box(pits,concrete,0,3.2,0,11,6.4,170);
    box(pits,dark,0,6.6,0,13,.35,176);
    box(pits,glass,0,5.0,0,11.1,2.25,169);
    for(let i=0;i<17;i++){
      box(pits,dark,5.57,1.62,-79+i*9.8,.04,3.1,8.5);
      box(pits,metal,5.63,1.65,-79+i*9.8,.06,2.9,8.15);
      const number=add(pits,new THREE.PlaneGeometry(1.7,.45),signTexture(String(i+1).padStart(2,'0')),5.69,3.55,-79+i*9.8);number.rotation.y=Math.PI/2;
      box(pits,concrete,5.75,2.0,-84+i*9.8,.25,4,.35);
    }
    const logo=add(pits,new THREE.PlaneGeometry(28,4),signTexture('ASTRA','M O T O R S P O R T'),5.62,5,2);logo.rotation.y=Math.PI/2;
    // Main grandstand opposite the pits, stepped seating and floating canopy.
    const q=t.at(165,32),stand=new THREE.Group();stand.position.set(q.x,0,q.z);stand.rotation.y=q.heading;this.root.add(stand);
    const seat=mat('#768c82'),roof=mat('#d9d7c8');
    for(let row=0;row<9;row++){box(stand,concrete,row*.95,row*.44+.2,0,.95,.44,96);box(stand,seat,row*.95,row*.44+.55,0,.45,.25,94);}
    box(stand,roof,3.9,8.8,0,14,.25,100).rotation.z=-.055;
    for(let z=-45;z<=45;z+=15)box(stand,metal,7,4.4,z,.25,8.8,.25);
    // Human silhouettes in a single instanced mesh.
    const spectators=new THREE.InstancedMesh(new THREE.SphereGeometry(.17,5,5),mat('#90876e'),500),dummy=new THREE.Object3D(),rng=random(71);
    for(let i=0;i<500;i++){const row=Math.floor(rng()*9);dummy.position.set(row*.95,row*.44+.93,(rng()-.5)*93);dummy.scale.set(1,2,1);dummy.updateMatrix();spectators.setMatrixAt(i,dummy.matrix);spectators.setColorAt(i,new THREE.Color().setHSL(rng(),.2,.2+rng()*.5));}stand.add(spectators);
    // Cantilever timing tower at the start line.
    const towerP=t.at(72,-24),tower=new THREE.Group();tower.position.set(towerP.x,0,towerP.z);tower.rotation.y=towerP.heading;this.root.add(tower);
    box(tower,concrete,0,6,0,5,12,7);box(tower,glass,0,11,0,8,3,10);box(tower,dark,0,12.7,0,9,.3,11);
    const g=t.at(100),gantry=new THREE.Group();gantry.position.set(g.x,0,g.z);gantry.rotation.y=g.heading;this.root.add(gantry);
    for(const x of [-8.8,8.8])box(gantry,metal,x,4.2,0,.4,8.4,.5);
    box(gantry,dark,0,7.75,0,18.5,1.5,.55);
    add(gantry,new THREE.PlaneGeometry(16,1.15),signTexture(t.id==='harbor-ring'?'HARBOR RING / ASTRA':'SOLENNE / ASTRA'),0,7.75,.29);
    this.startLights=[];
    for(let i=0;i<5;i++){const light=add(gantry,new THREE.CircleGeometry(.18,16),new THREE.MeshBasicMaterial({color:'#351e18'}),i*.55-1.1,6.7,-.32);light.rotation.y=Math.PI;this.startLights.push(light);}
  }
  buildDetails(){
    const t=this.track,dummy=new THREE.Object3D();
    const count=Math.ceil(t.length/4),steel=mat('#b8bab1',.45,.65);
    const barriers=new THREE.InstancedMesh(new THREE.BoxGeometry(.16,.65,4.04),steel,count*2);
    const posts=new THREE.InstancedMesh(new THREE.BoxGeometry(.10,2.5,.10),mat('#787e72',.6,.5),count*2);let k=0;
    for(const side of [-1,1])for(let i=0;i<count;i++){
      const p=t.at(i/count*t.length,side*(t.barrierOffset+1.1));dummy.position.set(p.x,.61,p.z);dummy.rotation.set(0,p.heading,0);dummy.scale.set(1,1,1);dummy.updateMatrix();barriers.setMatrixAt(k,dummy.matrix);
      dummy.position.y=1.25;dummy.updateMatrix();posts.setMatrixAt(k,dummy.matrix);k++;
    }
    barriers.castShadow=barriers.receiveShadow=true;posts.castShadow=true;this.root.add(barriers,posts);
    const fenceCanvas=document.createElement('canvas');fenceCanvas.width=fenceCanvas.height=64;const fc=fenceCanvas.getContext('2d');fc.strokeStyle='rgba(98,106,100,.7)';fc.lineWidth=1;fc.beginPath();fc.moveTo(0,0);fc.lineTo(64,64);fc.moveTo(64,0);fc.lineTo(0,64);fc.stroke();const map=new THREE.CanvasTexture(fenceCanvas);map.wrapS=map.wrapT=THREE.RepeatWrapping;
    const fm=new THREE.MeshStandardMaterial({map,transparent:true,alphaTest:.1,side:THREE.DoubleSide,roughness:.8});
    for(const side of [-1,1]){
      const pos=[],uv=[],ix=[];
      for(let i=0;i<=600;i++){const p=t.at(i/600*t.length,side*(t.barrierOffset+1.2));pos.push(p.x,.85,p.z,p.x,2.4,p.z);uv.push(i/600*t.length*2,0,i/600*t.length*2,3);if(i<600){const a=i*2;ix.push(a,a+1,a+2,a+1,a+3,a+2);}}
      const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(pos,3));g.setAttribute('uv',new THREE.Float32BufferAttribute(uv,2));g.setIndex(ix);g.computeVertexNormals();const f=add(this.root,g,fm);f.castShadow=false;
    }
    for(let i=0;i<18;i++){
      const p=t.at(i/18*t.length,19),board=new THREE.Group();board.position.set(p.x,1.0,p.z);board.rotation.y=p.heading-Math.PI/2;this.root.add(board);
      add(board,new THREE.PlaneGeometry(10,1.3),signTexture(i%3===0?'ASTRA':i%3===1?'SOLENNE':'DRIVE THE LIMIT',i%3===0?'M O T O R S P O R T':''));
    }
    for(const distance of [610,1240,1700,2110,2510])for(const n of [150,100,50]){
      const p=t.at(distance-n,-9),g=new THREE.Group();g.position.set(p.x,1,p.z);g.rotation.y=p.heading+Math.PI;this.root.add(g);
      add(g,new THREE.PlaneGeometry(1.05,1.3),signTexture(String(n),'',false));
    }
    for(let i=0;i<12;i++){
      const p=t.at(i*13+30,-18.5);const pole=add(this.root,new THREE.CylinderGeometry(.04,.04,7,8),steel,p.x,3.5,p.z);
      const flag=add(this.root,new THREE.PlaneGeometry(1.2,3.4),signTexture('A','',false),p.x+.6,5.1,p.z);flag.rotation.y=.3;flag.material.color.set(i%2?'#d7d4c8':'#df5539');
    }
  }
  update(car,time,countdown,aerial=false){
    this.riviera.update(time);
    const centre=this.centre.set(car.x,0,car.z);
    // Snap in light space so moving the shadow camera does not crawl across pixels.
    const extent=aerial?85:52;
    if(this.shadowExtent!==extent){Object.assign(this.sun.shadow.camera,{left:-extent,right:extent,top:extent,bottom:-extent});this.sun.shadow.camera.updateProjectionMatrix();this.shadowExtent=extent;}
    const texel=extent*2/this.sun.shadow.mapSize.x;
    this.shadowRight.crossVectors(this.sunDirection,THREE.Object3D.DEFAULT_UP).normalize();
    this.shadowUp.crossVectors(this.shadowRight,this.sunDirection).normalize();
    const sx=centre.dot(this.shadowRight),sy=centre.dot(this.shadowUp);
    centre.addScaledVector(this.shadowRight,Math.round(sx/texel)*texel-sx).addScaledVector(this.shadowUp,Math.round(sy/texel)*texel-sy);
    this.sun.position.copy(centre).addScaledVector(this.sunDirection,140);this.sun.target.position.copy(centre);this.sun.target.updateMatrixWorld();
    const wet=this.track.wetness||0;
    if(wet!==this.visualWetness){const surface=wetSurface(wet);if((this.roadMaterial.clearcoat>0)!==(surface.clearcoat>0))this.roadMaterial.needsUpdate=true;this.roadMaterial.roughness=surface.roughness;this.roadMaterial.clearcoat=surface.clearcoat;this.roadMaterial.color.setScalar(surface.darken);this.visualWetness=wet;}
    this.startLights.forEach((l,i)=>l.material.color.set(countdown>0&&countdown<4-i*.45?'#f94322':'#351e18'));
    if(Math.floor(time*2)!==this.rubberTick){
      this.rubberTick=Math.floor(time*2);const color=this.rubberMesh.geometry.attributes.color;
      for(let i=0;i<this.track.rubber.length;i++){const v=1-this.track.rubber[i]*.86;for(let j=0;j<6;j++)color.setXYZ(i*6+j,v,v,v);}color.needsUpdate=true;
    }
  }
}
