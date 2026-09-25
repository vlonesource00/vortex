export function showcaseOptions(search) {
  const q = new URLSearchParams(search);
  const choice = (key, values, fallback) => values.includes(q.get(key)) ? q.get(key) : fallback;
  return { enabled: q.get('showcase') === '1', laps: Number(choice('laps', ['3','5','10'], '3')),
    field: Number(choice('field', ['4','6','8'], '6')), camera: choice('camera', ['chase','bonnet','cockpit','trackside','engineer'], 'engineer'),
    objective: choice('pace', ['race','qualifying'], 'race'),
    track:choice('track',['solenne','harbor-ring'],'solenne'),
    classId:choice('class',['gt','touring','prototype'],'gt'),mixed:q.get('grid')==='mixed' };
}

export function showcaseRestartAt(phase, now, deadline) {
  if (phase !== 'finished') return null;
  return deadline ?? now + 12000;
}
