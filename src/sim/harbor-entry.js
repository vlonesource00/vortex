import { clamp, lerp } from './math.js';

// Driving arc only: the pinned road geometry is never modified.
export class HarborEntry {
  constructor(track){
    this.start=840;this.end=900;this.step=.5;this.points=[];
    const a=track.at(this.start),b=track.at(this.end),length=Math.hypot(b.x-a.x,b.z-a.z);
    const point=u=>{const h0=2*u**3-3*u*u+1,h1=u**3-2*u*u+u,h2=-2*u**3+3*u*u,h3=u**3-u*u;
      return {x:h0*a.x+h1*length*a.tx+h2*b.x+h3*length*b.tx,z:h0*a.z+h1*length*a.tz+h2*b.z+h3*length*b.tz};};
    for(let s=this.start;s<=this.end;s+=this.step){
      const f=track.at(s);let lo=0,hi=1;
      for(let j=0;j<32;j++){const u=(lo+hi)/2,p=point(u);if((p.x-f.x)*f.tx+(p.z-f.z)*f.tz<0)lo=u;else hi=u;}
      const p=point((lo+hi)/2);p.offset=(p.x-f.x)*f.nx+(p.z-f.z)*f.nz;
      if(Math.abs(p.offset)>track.halfWidth-1.2)throw new Error('Harbor entry exceeds usable asphalt');
      this.points.push(p);
    }
  }
  at(s,frame,original){
    if(s<this.start-35||s>this.end+35)return null;
    if(s<this.start||s>this.end){const u=clamp((s<this.start?this.start-s:s-this.end)/35,0,1),offset=original*u*u*(3-2*u);
      return {x:frame.x+frame.nx*offset,z:frame.z+frame.nz*offset,offset};}
    const index=clamp((s-this.start)/this.step,0,this.points.length-1),i=Math.floor(index),a=this.points[i],b=this.points[Math.min(i+1,this.points.length-1)],t=index-i;
    return {x:lerp(a.x,b.x,t),z:lerp(a.z,b.z,t),offset:lerp(a.offset,b.offset,t)};
  }
}
