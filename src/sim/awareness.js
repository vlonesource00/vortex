// Relative motion and physical footprints for simultaneous battle awareness.
// A lateral neighbour counts even when it is fractionally ahead or behind.
export function raceAwareness(car,observation){
  const origin=observation.origin;
  const rivals=observation.observations.map(o=>{
    const closing=o.distance>=0?car.speed-o.speed:o.speed-car.speed;
    const gap=Math.max(0,Math.abs(o.distance)-o.halfLength-2.3);
    return {...o,closing,gap,ttc:closing>.1?gap/closing:Infinity,
      sideClearance:Math.abs(o.lateral-origin.lateral)-o.halfWidth-.99,
      overlap:Math.abs(o.distance)<o.halfLength+2.3};
  });
  const front=rivals.filter(o=>o.distance>=0).sort((a,b)=>a.distance-b.distance)[0]??null;
  const rear=rivals.filter(o=>o.distance<0).sort((a,b)=>b.distance-a.distance)[0]??null;
  // A fast second pursuer can be the real threat while the nearest car drops
  // back. This is a constant-velocity warning, not permission to change lanes.
  const rearThreat=rivals.filter(o=>o.distance<0&&((o.closing>.5&&o.ttc<4)||(o.gap<6&&o.closing>-.5)))
    .sort((a,b)=>a.ttc-b.ttc||b.distance-a.distance)[0]??null;
  const rearThreatSeparateLane=!!rearThreat&&rivals.filter(o=>o.distance<0&&o.distance>rearThreat.distance)
    .every(o=>Math.abs(rearThreat.lateral-o.lateral)>rearThreat.halfWidth+o.halfWidth+.15);
  const neighbours=rivals.filter(o=>Math.abs(o.distance)<12);
  const sideClearance=side=>neighbours.filter(o=>Math.sign(o.lateral-origin.lateral)===side)
    .reduce((gap,o)=>Math.min(gap,o.sideClearance),Infinity);
  return {front,rear,rearThreat,rearThreatSeparateLane,rivals,left:sideClearance(-1),right:sideClearance(1)};
}
