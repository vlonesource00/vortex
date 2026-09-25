// React to observed lost momentum, never another driver's hidden controls.
export function flowOpportunity(car,observation){
  if(car.speed<12)return null;
  return observation.observations.filter(other=>other.distance>7&&other.distance<60&&
    Math.abs(other.lateral-observation.origin.lateral)<2.6&&car.speed-other.speed>5&&
    ((other.acceleration< -2&&other.acceleration<(car.ax??0)-3)||other.speed<car.speed*.65||Math.abs(other.headingError)>.25))
    .sort((a,b)=>a.distance-b.distance)[0]??null;
}
