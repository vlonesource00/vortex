// Driver policy only: no changes to engine power, tyre forces or collision
// response. On the local road cross-section, a centred symmetric car retains
// half its footprint when its centre reaches the track boundary.
export const RACECRAFT={trafficBlend:.45,edgeInset:1.1,rubOverlap:.22,rubClosing:2.5,rubSideSpeed:1};
export const raceOffsetLimit=track=>track.halfWidth-RACECRAFT.edgeInset;
export const halfCarInside=(lateral,halfWidth)=>Math.abs(lateral)<=halfWidth;
export const gentleRub=(overlap,closing,sideSpeed)=>overlap<=RACECRAFT.rubOverlap&&closing<=RACECRAFT.rubClosing&&sideSpeed<=RACECRAFT.rubSideSpeed;

// A following-speed cap belongs behind a car, not beside its doors. Keep a
// small clearance buffer, but allow settled side-by-side running and light rubs.
export function needsFollowing({gap,lateralGap,halfWidth,halfLength,closing,sideSpeed,alongside=false}){
  if(gap<=0||gap>=40||lateralGap>=2.15)return false;
  const overlap=Math.max(0,halfWidth+.98-lateralGap);
  return !(alongside&&gap<halfLength+2.3&&gentleRub(overlap,closing,sideSpeed));
}
