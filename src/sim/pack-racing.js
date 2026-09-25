import { clamp } from './math.js';

// Front progress and rear exposure are independent of the committed steering
// side. Inputs are observed motion, never rival controls.
export function packExitCost(car,observations,exit,predict,attackValue=1){
  let frontCost=0,rearCost=0;
  for(const other of observations){
    if(Math.abs(other.distance)>65||Math.abs(other.distance)<5)continue;
    const future=predict(other,exit.time),gap=future.distance-exit.distance;
    const delta=(gap-other.distance)/Math.max(12,car.speed);
    if(other.distance>0){
      const relevance=clamp((65-other.distance)/45,0,1);
      frontCost+=relevance*clamp(delta,-.6,1.5)*.4*attackValue;
    }else{
      const pressure=clamp((32+other.distance+Math.max(0,other.speed-car.speed)*2)/28,0,1);
      rearCost+=pressure*clamp(delta,0,1.5)*.5;
      if(gap>-8&&gap<12)rearCost+=pressure*clamp((future.speed-exit.speed)/8,0,1)*.3;
    }
  }
  return {frontCost,rearCost,total:frontCost+rearCost};
}
