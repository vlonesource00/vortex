import * as THREE from 'three';

// Periodic noise keeps every map seamless, including the low-frequency mottling.
function noise(x,y,grid,seed){
  const hash=(a,b)=>{const n=Math.sin(((a%grid+grid)%grid)*127.1+((b%grid+grid)%grid)*311.7+seed*73.3)*43758.5453;return n-Math.floor(n);};
  const ix=Math.floor(x),iy=Math.floor(y),u=x-ix,v=y-iy,s=u*u*(3-2*u),t=v*v*(3-2*v);
  return THREE.MathUtils.lerp(THREE.MathUtils.lerp(hash(ix,iy),hash(ix+1,iy),s),THREE.MathUtils.lerp(hash(ix,iy+1),hash(ix+1,iy+1),s),t);
}

export function surfacePixels(kind,size=256){
  const color=new Uint8Array(size*size*4),height=new Uint8Array(size*size*4),roughness=new Uint8Array(size*size*4);
  for(let y=0;y<size;y++)for(let x=0;x<size;x++){
    const i=(y*size+x)*4,n=noise(x/size*8,y/size*8,8,7),m=noise(x/size*3,y/size*3,3,19),grain=noise(x/size*128,y/size*128,128,3);
    let rgb,h,r;
    if(kind==='asphalt'){
      const chip=grain>.64?(grain-.64)*38:0,v=52+n*3+m*3+grain*4+chip;
      rgb=[v,v+1,v+3];h=grain*160+n*40;r=185+grain*55;
    }else if(kind==='gravel'){
      const stone=noise(x/size*70,y/size*70,70,13),v=110+stone*64+m*19;
      rgb=[v+15,v+7,v-12];h=stone*220;r=215+stone*35;
    }else if(kind==='grass'){
      const dry=m*.65+n*.35;
      rgb=[84+dry*20+grain*12,95+dry*18+grain*12,52+dry*14+grain*8];h=grain*150;r=245;
    }else{
      const weave=((Math.floor(x/8)+Math.floor(y/8))%2?x:y)%8/8,v=16+Math.sin(weave*Math.PI)*15;
      rgb=[v,v+2,v+3];h=v*5;r=120+weave*35;
    }
    for(let c=0;c<3;c++){color[i+c]=rgb[c];height[i+c]=h;roughness[i+c]=r;}
    color[i+3]=height[i+3]=roughness[i+3]=255;
  }
  return {color,height,roughness,size};
}

const cache=new Map();
export function surfaceMaps(kind){
  if(cache.has(kind))return cache.get(kind);
  const pixels=surfacePixels(kind),make=(data,srgb=false)=>{
    const map=new THREE.DataTexture(data,pixels.size,pixels.size,THREE.RGBAFormat);
    map.wrapS=map.wrapT=THREE.RepeatWrapping;map.magFilter=THREE.LinearFilter;map.minFilter=THREE.LinearMipmapLinearFilter;
    map.generateMipmaps=true;map.anisotropy=4;map.colorSpace=srgb?THREE.SRGBColorSpace:THREE.NoColorSpace;map.needsUpdate=true;return map;
  };
  const maps={map:make(pixels.color,true),bumpMap:make(pixels.height),roughnessMap:make(pixels.roughness)};
  const repeat=kind==='asphalt'?4:kind==='carbon'?6:1;
  for(const map of Object.values(maps))map.repeat.setScalar(repeat);
  cache.set(kind,maps);return maps;
}

export const LIGHTING={
  golden:{sun:'#ffe0bc',sky:'#cbdfea',ground:'#a69a80',fog:'#d4ccba',intensity:3.1,fill:1.05,environment:.65,exposure:1.02,elevation:.42,turbidity:3.1,density:.00055},
  day:{sun:'#fff5e9',sky:'#c4e2ff',ground:'#9b987e',fog:'#c1d6df',intensity:3.5,fill:1.25,environment:.75,exposure:.95,elevation:.95,turbidity:2.2,density:.0004},
  overcast:{sun:'#e6edf5',sky:'#dbe5ef',ground:'#898e85',fog:'#bac8cc',intensity:.7,fill:1.8,environment:.85,exposure:1.05,elevation:.65,turbidity:14,density:.00085}
};

export function wetSurface(wetness){
  const wet=THREE.MathUtils.clamp(Number.isFinite(wetness)?wetness:0,0,1);
  return {roughness:1-wet*.8,clearcoat:wet*.85,darken:1-wet*.32};
}
