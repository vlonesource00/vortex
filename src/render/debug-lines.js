import * as THREE from 'three';

// Persistent style batches: separate paths become segments, never connecting
// the end of one prediction to the start of another.
export class DebugLines {
  constructor(root){this.root=root;this.batches=new Map();}
  clear(){for(const batch of this.batches.values()){batch.count=0;batch.mesh.visible=false;}}
  add(points,color,opacity=1){
    if(points.length<2)return;
    const key=`${color}/${opacity}`;
    let batch=this.batches.get(key);
    if(!batch){
      const material=new THREE.LineBasicMaterial({color,transparent:true,opacity,depthTest:false,depthWrite:false});
      const mesh=new THREE.LineSegments(new THREE.BufferGeometry(),material);
      mesh.frustumCulled=false;mesh.renderOrder=30;this.root.add(mesh);
      batch={mesh,count:0,capacity:0};this.batches.set(key,batch);
    }
    const required=batch.count+(points.length-1)*2;
    if(required>batch.capacity){
      const capacity=2**Math.ceil(Math.log2(Math.max(256,required)));
      const array=new Float32Array(capacity*3);
      const old=batch.mesh.geometry;
      if(batch.count)array.set(old.attributes.position.array.subarray(0,batch.count*3));
      const geometry=new THREE.BufferGeometry();
      geometry.setAttribute('position',new THREE.BufferAttribute(array,3).setUsage(THREE.DynamicDrawUsage));
      batch.mesh.geometry=geometry;batch.capacity=capacity;old.dispose();
    }
    const array=batch.mesh.geometry.attributes.position.array;
    for(let i=1;i<points.length;i++)for(const p of [points[i-1],points[i]]){
      const j=batch.count++*3;array[j]=p.x;array[j+1]=p.y??.11;array[j+2]=p.z;
    }
  }
  flush(){
    for(const {mesh,count} of this.batches.values()){
      mesh.visible=count>0;mesh.geometry.setDrawRange(0,count);
      if(!count)continue;
      const position=mesh.geometry.attributes.position;
      position.clearUpdateRanges();position.addUpdateRange(0,count*3);position.needsUpdate=true;
    }
  }
}
