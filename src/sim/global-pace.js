import { clamp, wrap } from './math.js';
import { pathCurvature } from './path-geometry.js';

// A periodic, class-specific speed envelope. Uses physical power, drag and
// downforce, then couples every braking zone to the following acceleration.
// Terminal recovery is measured in seconds, not a reward for raw exit speed.
export class GlobalPace {
  constructor(line,spec){
    this.line=line;this.spec=spec;const nodes=line.track.nodes,n=nodes.length;
    this.speed=new Float64Array(n);this.ds=new Float64Array(n);
    for(let i=0;i<n;i++){
      const s=nodes[i].s,a=line.at(s-5),b=line.at(s),c=line.at(s+5);
      const h1=Math.atan2(b.x-a.x,b.z-a.z),h2=Math.atan2(c.x-b.x,c.z-b.z);
      const k=line.track.id==='harbor-ring'?Math.abs(pathCurvature(a,b,c)):Math.abs(Math.atan2(Math.sin(h2-h1),Math.cos(h2-h1)))/5;
      let lo=5,hi=86;
      for(let j=0;j<16;j++){const v=(lo+hi)/2;if(v*v*k<=this.lateral(v))lo=v;else hi=v;}
      this.speed[i]=lo;this.ds[i]=wrap(nodes[(i+1)%n].s-s,line.track.length);
    }
    for(let pass=0;pass<6;pass++){
      for(let i=n-1;i>=0;i--)this.speed[i]=Math.min(this.speed[i],Math.sqrt(this.speed[(i+1)%n]**2+2*this.brake()*this.ds[i]));
      for(let i=0;i<n;i++){const j=wrap(i-1,n);this.speed[i]=Math.min(this.speed[i],Math.sqrt(Math.max(1,this.speed[j]**2+2*this.drive(this.speed[j])*this.ds[j])));}
    }
    this.lapTime=nodes.reduce((t,p,i)=>t+2*this.ds[i]/(this.speed[i]+this.speed[(i+1)%n]),0);
  }
  lateral(v){const s=this.spec;return 1.12*s.tyreGrip*(9.81+.5*1.225*v*v*s.area*s.cl/(s.mass+26.25))*.80;}
  brake(){return 7.5*this.spec.tyreGrip;}
  drive(v){const s=this.spec;let gear=1;while(gear<6&&v/s.radius*s.gears[gear]*s.finalDrive*9.5493>7450)gear++;
    const ratio=s.gears[gear]*s.finalDrive,rpm=v/s.radius*ratio*9.5493;
    const force=s.maxTorque*clamp(1-((rpm-5500)/6700)**2,.45,1)*ratio*.91/s.radius;
    return clamp((force-.5*1.225*v*v*s.area*s.cd)/(s.mass+26.25)-.13,-2,this.lateral(v)*(s.drive==='front'?.40:.48));}
  at(s){const p=this.line.track.at(s),i=p.index,j=(i+1)%this.speed.length,t=wrap(s-this.line.track.nodes[i].s,this.line.track.length)/this.ds[i];return this.speed[i]+(this.speed[j]-this.speed[i])*clamp(t,0,1);}
  recoveryCost(s,speed,offset){
    // Compare actual terminal speed to the same nominal continuation, ending
    // once it catches the reference or after 450 m. No free speed reset.
    let v=Math.max(1,speed),loss=0;
    for(let d=10;d<=450;d+=10){const target=this.at(s+d),previous=v;
      v=Math.min(target,Math.sqrt(Math.max(1,v*v+2*this.drive(v)*10)));
      loss+=Math.max(0,20/(previous+v)-10/Math.max(1,target));
      if(v>=target-.05)break;
    }
    const lineOffset=this.line.offsetAt(s),shift=Math.abs(offset-lineOffset);
    return loss+shift*shift/(Math.max(10,speed)*8);
  }
}
