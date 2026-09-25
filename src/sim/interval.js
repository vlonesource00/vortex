// Elapsed time since the leader crossed the follower's current race distance.
// History uses monotonic race progress; braking speed is not a denominator.
export function raceInterval(history,progress,now,liveProgress=null){
  if(!history?.length||progress<history[0].progress)return null;
  const last=history.at(-1);
  if(progress>last.progress){
    if(liveProgress===null||liveProgress<progress)return null;
    const crossing=last.time+(now-last.time)*(progress-last.progress)/Math.max(.000001,liveProgress-last.progress);
    return Math.max(0,now-crossing);
  }
  let lo=0,hi=history.length-1;
  while(lo<hi){const mid=(lo+hi)>>1;if(history[mid].progress<progress)lo=mid+1;else hi=mid;}
  const b=history[lo],a=history[Math.max(0,lo-1)];
  const blend=(progress-a.progress)/Math.max(.000001,b.progress-a.progress);
  return Math.max(0,now-(a.time+(b.time-a.time)*blend));
}
