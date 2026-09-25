import * as THREE from 'three';
import { random } from '../sim/math.js';

// Industrial waterfront in world metres. Decoration never alters track nodes.
export class HarborScenery {
  constructor(parent,track){
    this.root=new THREE.Group();this.root.name='Harbor waterfront';parent.add(this.root);
    this.placements=[];const rng=random(73124),dummy=new THREE.Object3D();
    const material=(color,metalness=.2)=>new THREE.MeshStandardMaterial({color,metalness,roughness:.72});
    const batch=(color,count)=>{const mesh=new THREE.InstancedMesh(new THREE.BoxGeometry(1,1,1),material(color),count);mesh.count=0;mesh.castShadow=mesh.receiveShadow=true;this.root.add(mesh);return mesh;};
    const containers=batch('#bc5139',260),steel=batch('#bbc6c0',180),dark=batch('#243d43',150),concrete=batch('#afa99a',130);
    const place=(mesh,x,y,z,w,h,d,heading=0)=>{
      const radius=Math.hypot(w,d)/2;
      if(Math.abs(track.nearest(x,z).lateral)<track.barrierOffset+radius+2)return false;
      dummy.position.set(x,y,z);dummy.rotation.set(0,heading,0);dummy.scale.set(w,h,d);dummy.updateMatrix();mesh.setMatrixAt(mesh.count++,dummy.matrix);
      this.placements.push({x,z,radius});return true;
    };
    for(let i=0;i<150;i++){
      const s=track.length*(.26+rng()*.61),p=track.at(s,(i%2?1:-1)*(48+rng()*45)),level=i%3;
      if(place(containers,p.x,1.4+level*2.85,p.z,12,2.8,2.5,p.heading))containers.setColorAt(containers.count-1,new THREE.Color().setHSL([.02,.12,.48,.58][i%4],.35,.38));
    }
    // Warehouses and loading aprons beyond the race enclosure.
    for(let i=0;i<9;i++){
      const p=track.at(track.length*(.32+i*.065),-85);
      place(concrete,p.x,.1,p.z,42,.2,35,p.heading);
      place(dark,p.x,4,p.z,30,8,22,p.heading);
      place(steel,p.x,8.2,p.z,33,.45,25,p.heading);
    }
    // Container cranes and suspended spreaders along the eastern quay.
    for(let i=0;i<4;i++){
      const x=610+i*57,z=-180+i*110;
      for(const side of [-1,1])place(steel,x+side*11,18,z,1.4,36,1.4);
      place(containers,x,37,z,75,2,2.6);
      place(dark,x+15,24,z,.3,24,.3);place(containers,x+15,12,z,11,.5,2.8);
    }
    const waterMaterial=new THREE.MeshPhysicalMaterial({color:'#286879',roughness:.24,metalness:.35,clearcoat:1});
    this.waterMaterial=waterMaterial;this.time={value:0};
    waterMaterial.onBeforeCompile=shader=>{shader.uniforms.harborTime=this.time;shader.fragmentShader='uniform float harborTime;\n'+shader.fragmentShader;shader.fragmentShader=shader.fragmentShader.replace('#include <normal_fragment_maps>','#include <normal_fragment_maps>\nnormal=normalize(normal+vec3(sin(vViewPosition.x*.08+harborTime)*.045,0.,cos(vViewPosition.z*.1+harborTime*.7)*.045));');};
    const water=new THREE.Mesh(new THREE.PlaneGeometry(1250,1500),waterMaterial);water.rotation.x=-Math.PI/2;water.position.set(1375,-.02,0);water.receiveShadow=true;this.root.add(water);
    place(concrete,720,.05,0,60,.2,1300);
    // Moored freighter: dark hull, container deck and white bridge.
    place(dark,840,3,40,36,6,165);place(steel,840,11,-12,25,12,22);
    for(let i=0;i<24;i++)place(containers,829+(i%3)*11,7+Math.floor(i/12)*2.8,12+Math.floor(i/3)%4*15,9.8,2.6,12);
  }
  update(time){this.time.value=time;}
}
