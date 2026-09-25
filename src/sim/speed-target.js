import { relativeDistance } from './perception.js';

// A plan is held between tactical solves; braking distances must be measured
// from the car's current progress, including across the start/finish seam.
export function brakingTarget(plan,currentS,trackLength,skill,brake,ceiling=Infinity){
  const travelled=relativeDistance(currentS,plan.startS??currentS,trackLength);
  const physicalTravelled=plan.pathDistanceAt?.(travelled)??travelled;
  let target=ceiling;
  for(const p of plan.points){
    const remaining=(p.travelDistance??p.distance)-physicalTravelled;
    if(remaining<3)continue;
    target=Math.min(target,Math.sqrt(Math.max(0,Math.min(p.trafficSpeed??Infinity,p.speedLimit)*skill)**2+2*brake*Math.max(0,remaining-4)));
  }
  return target;
}
