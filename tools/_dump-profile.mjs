import { readFileSync } from 'node:fs';
const r = JSON.parse(readFileSync('src/ai/vortex/atlas/oracle-data.json','utf8'));
const n=r.n, sp=r.spacing;
const at = s => Math.floor(s/sp)%n;
console.log('lapTime', r.lapTime, 'n', n, 'spacing', sp.toFixed(3), 'grip', r.gripScale);
console.log(' s      q      v      |k|    R');
for(let s=780; s<=920; s+=5){
  const i=at(s);
  console.log(String(s).padStart(5),
    r.q[i].toFixed(2).padStart(7),
    r.v[i].toFixed(2).padStart(7),
    Math.abs(r.kappa[i]).toFixed(4).padStart(8),
    (1/Math.max(1e-4,Math.abs(r.kappa[i]))).toFixed(2).padStart(8));
}
