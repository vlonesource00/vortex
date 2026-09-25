import * as THREE from 'three';
import { random } from '../sim/math.js';

export class RivieraScenery {
  constructor(root,track){
    const rng=random(921),dummy=new THREE.Object3D();this.time={value:0};
    const material=color=>new THREE.MeshStandardMaterial({color,roughness:.85});
    const batch=(geometry,mat,count)=>{const mesh=new THREE.InstancedMesh(geometry,mat,count);mesh.castShadow=true;mesh.receiveShadow=true;root.add(mesh);return mesh;};
    const place=(mesh,index,x,y,z,sx=1,sy=1,sz=1,rotation=0)=>{dummy.position.set(x,y,z);dummy.scale.set(sx,sy,sz);dummy.rotation.set(0,rotation,0);dummy.updateMatrix();mesh.setMatrixAt(index,dummy.matrix);};
    // Tall cypress groves frame the bends; flowers soften the verge beyond the
    // safety fence. All planting sits clear of the asphalt and runoff.
    const trunks=batch(new THREE.CylinderGeometry(.13,.2,6,6),material('#66563e'),140);
    const crownGeometry=new THREE.SphereGeometry(1.2,10,12).scale(.85,3.3,.85);
    const crowns=batch(crownGeometry,material('#aab59b'),100);
    for(let i=0;i<100;i++){
      const s=420+rng()*(track.length-500),p=track.at(s,(i%2?1:-1)*(25+rng()*15)),scale=.65+rng()*.65;
      place(trunks,i,p.x,3*scale,p.z,scale,scale,scale);place(crowns,i,p.x,6*scale,p.z,scale,scale,scale);
      crowns.setColorAt(i,new THREE.Color().setHSL(.23+rng()*.025,.22,.32+rng()*.10));
    }
    const frond=new THREE.BufferGeometry();
    frond.setAttribute('position',new THREE.Float32BufferAttribute([0,0,0,-.5,.22,1.4,.5,.22,1.4,0,-.65,3.4],3));
    frond.setIndex([0,1,2,1,3,2]);frond.computeVertexNormals();
    const leafMaterial=material('#58734a');leafMaterial.side=THREE.DoubleSide;
    const palms=batch(frond,leafMaterial,40*9);
    for(let i=0;i<40;i++){
      const p=track.at(440+i*57,(i%2?1:-1)*(24+(i%3)*5)),height=5+rng()*3;
      place(trunks,100+i,p.x,height/2,p.z,1,height/6,1);
      for(let j=0;j<9;j++)place(palms,i*9+j,p.x,height,p.z,1,1,1,j*Math.PI*2/9+i);
    }
    const flowers=batch(new THREE.IcosahedronGeometry(.19,0),material('#e6deea'),4200);flowers.castShadow=false;
    for(let i=0;i<4200;i++){
      const s=420+rng()*(track.length-520),p=track.at(s,(i%2?1:-1)*(21+rng()*4));
      place(flowers,i,p.x,.18+rng()*.12,p.z,.6,1+rng(),.6);
      flowers.setColorAt(i,new THREE.Color().setHSL(.72+rng()*.08,.24+rng()*.18,.35+rng()*.22));
    }
    const umbrellas=batch(new THREE.ConeGeometry(2.1,.65,8),material('#f1d8ad'),24);
    const poles=batch(new THREE.CylinderGeometry(.045,.045,2.6,6),material('#cbbca4'),24);
    const tables=batch(new THREE.CylinderGeometry(.6,.6,.08,12),material('#d1b68a'),24);
    for(let i=0;i<24;i++){
      const p=track.at(460+i*6,26+(i%3)*6);
      place(umbrellas,i,p.x,2.8,p.z);place(poles,i,p.x,1.3,p.z);place(tables,i,p.x,.8,p.z);
      umbrellas.setColorAt(i,new THREE.Color(['#da6848','#d4c39c','#498d91'][i%3]));
    }
    // Wind moves fabric in the vertex shader: no per-flag scene traversal.
    const fabric=material('#df6646');fabric.side=THREE.DoubleSide;
    fabric.onBeforeCompile=shader=>{
      shader.uniforms.windTime=this.time;
      shader.vertexShader='uniform float windTime;\n'+shader.vertexShader;
      shader.vertexShader=shader.vertexShader.replace('#include <begin_vertex>','#include <begin_vertex>\ntransformed.z += sin(position.x*3. + windTime*2.4)*.18*smoothstep(-1.,1.,position.x);');
    };
    const flags=batch(new THREE.PlaneGeometry(2,1,10,2),fabric,32);
    const flagpoles=batch(new THREE.CylinderGeometry(.045,.045,5,6),material('#b8b7a8'),32);
    for(let i=0;i<32;i++){
      const p=track.at(350+i*72,(i%2?1:-1)*22);
      place(flagpoles,i,p.x,2.5,p.z);place(flags,i,p.x+1,4.3,p.z);
      flags.setColorAt(i,new THREE.Color(i%2?'#fff3d1':'#71c1c3'));
    }
    // A sheltered turquoise lake beyond the east end of the circuit.
    const waterMaterial=new THREE.MeshPhysicalMaterial({color:'#326e78',metalness:.15,roughness:.19,clearcoat:1,clearcoatRoughness:.13});
    waterMaterial.onBeforeCompile=shader=>{
      shader.uniforms.waterTime=this.time;
      shader.fragmentShader='uniform float waterTime;\n'+shader.fragmentShader;
      shader.fragmentShader=shader.fragmentShader.replace('#include <normal_fragment_maps>','#include <normal_fragment_maps>\nnormal=normalize(normal+vec3(sin(vViewPosition.x*.16+waterTime*.7)*.035,cos(vViewPosition.z*.12+waterTime*.5)*.035,0.));');
    };
    const water=new THREE.Mesh(new THREE.CircleGeometry(1,96),waterMaterial);
    water.rotation.x=-Math.PI/2;water.scale.set(155,290,1);water.position.set(800,-.025,0);water.receiveShadow=true;root.add(water);
    const shore=new THREE.Mesh(new THREE.RingGeometry(1,1.045,96),material('#c6b894'));
    shore.rotation.x=-Math.PI/2;shore.scale.set(155,290,1);shore.position.set(800,-.02,0);root.add(shore);
  }
  update(time){this.time.value=time;}
}
