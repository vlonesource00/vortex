import * as THREE from 'three';
import { random } from '../sim/math.js';

// Distinct venues along the circuit, in a small number of shared draw batches.
export class TracksideLife {
  constructor(root,track){
    const rng=random(641),dummy=new THREE.Object3D();this.root=new THREE.Group();root.add(this.root);
    const batches=new Map();this.placements=[];
    const add=(kind,color,x,y,z,sx,sy,sz,heading=0)=>{
      // Reject even distant placements if another circuit section runs nearby.
      if(Math.abs(track.nearest(x,z).lateral)<20+Math.hypot(sx,sz)/2)return;
      if(!batches.has(kind))batches.set(kind,[]);
      batches.get(kind).push({color,x,y,z,sx,sy,sz,heading});
      this.placements.push({x,z,radius:Math.hypot(sx,sz)/2});
    };
    const at=(s,offset)=>track.at(s,offset);
    // Viewing villages: terraced seats, awnings, people and parked service vans.
    for(const [s,side] of [[470,1],[850,-1],[1330,1],[1810,-1],[2300,1],[2670,-1]]){
      for(let row=0;row<3;row++)for(let j=0;j<10;j++){
        const p=at(s+j*2.1,side*(25+row*2));
        add('box','#b1ad98',p.x,.3+row*.3,p.z,1.8,.6+row*.6,1.8,p.heading);
        if(j%3!==0){add('person',['#bf6543','#799aaf','#d2bd8c'][j%3],p.x,1.05+row*.6,p.z,.38,.85,.38);}
      }
      for(let j=0;j<4;j++){
        const p=at(s+j*8,side*40);
        add('box','#d0c4a7',p.x,1.3,p.z,4,2.6,4,p.heading);
        add('roof',j%2?'#bc6448':'#d3bea0',p.x,3,p.z,5,1.1,5,p.heading+Math.PI/4);
        const van=at(s+j*8+3,side*50);
        add('box',j%2?'#456d71':'#dfd6bd',van.x,1,van.z,2.1,1.8,4.5,van.heading);
        add('box','#374952',van.x,1.6,van.z,2.15,.55,2.7,van.heading);
        for(const side of [-1,1])for(const axle of [-1,1]){
          const x=van.x+Math.cos(van.heading)*side*1.05+Math.sin(van.heading)*axle*1.4;
          const z=van.z-Math.sin(van.heading)*side*1.05+Math.cos(van.heading)*axle*1.4;
          add('box','#252b2b',x,.35,z,.25,.7,.7,van.heading);
        }
      }
    }
    // Marshal cabins and shelters form repeated, readable trackside landmarks.
    for(let s=390;s<track.length;s+=210){
      const p=at(s,-23);
      add('box','#eee0c8',p.x,1.4,p.z,2.4,2.8,2.4,p.heading);
      add('box','#cf6439',p.x,2.95,p.z,3,.25,3,p.heading);
      const marshal=at(s+3,-24);add('person','#ee7b28',marshal.x,.8,marshal.z,.5,1.6,.5);
    }
    // Orderly olive groves and vineyard rows give the infield an agricultural identity.
    for(let section=0;section<7;section++)for(let row=0;row<5;row++)for(let j=0;j<12;j++){
      const p=at(400+section*360+j*5,(section%2?-1:1)*(65+row*7));
      if(section%2){
        add('box','#697a44',p.x,.8,p.z,1.2,1.4,3.8,p.heading);
      }else{
        add('box','#746044',p.x,1.1,p.z,.3,2.2,.3);
        add('tree','#84916a',p.x,2.9,p.z,3.6,3.3,3.5);
      }
    }
    // Small hillside settlement beyond the run-off; avoids the lake to the east.
    for(let i=0;i<55;i++){
      const x=-730+rng()*260,z=-380+rng()*570,w=7+rng()*7,d=7+rng()*7,h=5+rng()*8;
      add('box',['#d7c6a6','#c3b397','#ddd2b9'][i%3],x,h/2,z,w,h,d);
      add('roof','#9b6350',x,h+1,z,w*1.18,2.8,d*1.18,Math.PI/4);
      for(const side of [-1,1])for(let window=0;window<3;window++)add('box','#586768',x-w*.3+window*w*.3,h*.6,z+side*(d*.5+.03),1,1.5,.06);
    }
    // Shrub clusters at the bases of existing trees, rather than a uniform scatter.
    for(let i=0;i<500;i++){
      const p=at(400+rng()*(track.length-430),(i%2?-1:1)*(25+rng()*20));
      add('tree',i%3?'#6e7c51':'#938363',p.x,.65,p.z,1+rng()*1.5,1.3,1+rng()*1.5);
    }
    const geometry={box:new THREE.BoxGeometry(1,1,1),roof:new THREE.ConeGeometry(.72,1,4),tree:new THREE.IcosahedronGeometry(.5,1),person:new THREE.CapsuleGeometry(.18,.45,2,5)};
    for(const [kind,items] of batches){
      const mesh=new THREE.InstancedMesh(geometry[kind],new THREE.MeshStandardMaterial({color:'#ffffff',roughness:.9}),items.length);
      for(let i=0;i<items.length;i++){
        const p=items[i];dummy.position.set(p.x,p.y,p.z);dummy.rotation.set(0,p.heading,0);dummy.scale.set(p.sx,p.sy,p.sz);dummy.updateMatrix();mesh.setMatrixAt(i,dummy.matrix);mesh.setColorAt(i,new THREE.Color(p.color));
      }
      mesh.castShadow=kind!=='person';mesh.receiveShadow=true;this.root.add(mesh);
    }
  }
}
