import { clamp, wrap } from './math.js';
import { pathCurvature } from './path-geometry.js';

// Shared circuit-only cost-to-go. Previous/current lateral lanes encode entry
// direction, so a good exit cannot be claimed by an instantaneous lane change.
export class RaceContinuation {
  constructor(line){
    this.line=line;this.lanes=[-2.6,0,2.6];this.n=Math.ceil(line.track.length/20);this.ds=line.track.length/this.n;
    const points=Array.from({length:this.n},(_,i)=>this.lanes.map(o=>line.at(i*this.ds,o)));
    const edges=Array.from({length:this.n},()=>new Float64Array(27).fill(Infinity));
    for(let i=0;i<this.n;i++)for(let a=0;a<3;a++)for(let b=0;b<3;b++)for(let c=0;c<3;c++){
      if(Math.abs(a-b)>1||Math.abs(b-c)>1)continue;
      const p=points[wrap(i-1,this.n)][a],q=points[i][b],r=points[(i+1)%this.n][c],k=Math.abs(pathCurvature(p,q,r));
      let v=line.globalPace.at(i*this.ds);
      for(let j=0;j<8&&v*v*k>line.globalPace.lateral(v);j++)v*=.94;
      edges[i][a*9+b*3+c]=Math.hypot(r.x-q.x,r.z-q.z)/Math.max(5,v)+Math.abs(c-2*b+a)*.025;
    }
    let values=Array.from({length:this.n},()=>new Float64Array(9));
    for(let horizon=0;horizon<24;horizon++){
      const next=Array.from({length:this.n},()=>new Float64Array(9).fill(Infinity));
      for(let i=0;i<this.n;i++)for(let a=0;a<3;a++)for(let b=0;b<3;b++)for(let c=0;c<3;c++)
        next[i][a*3+b]=Math.min(next[i][a*3+b],edges[i][a*9+b*3+c]+values[(i+1)%this.n][b*3+c]);
      values=next;
    }
    this.values=values;this.points=points;this.horizon=24*this.ds;
  }
  cost(s,offset,slope){
    const i=Math.floor(wrap(s,this.line.track.length)/this.ds);let best=Infinity,base=Infinity;
    for(let a=0;a<3;a++)for(let b=0;b<3;b++){
      const v=this.values[i][a*3+b];base=Math.min(base,v);
      const rate=(this.lanes[b]-this.lanes[a])/this.ds;
      best=Math.min(best,v+Math.abs(this.points[i][b].offset-offset)*.12+(slope-rate)**2*12);
    }
    return clamp(best-base,0,3);
  }
}
