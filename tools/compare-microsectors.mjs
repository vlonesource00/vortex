import { readFile } from 'node:fs/promises';
const [beforePath, afterPath] = process.argv.slice(2);
if (!beforePath || !afterPath) { console.log('Usage: node tools/compare-microsectors.mjs before.json after.json'); process.exit(0); }
const [before, after] = await Promise.all([beforePath, afterPath].map(path => readFile(path, 'utf8').then(JSON.parse)));
const count = Math.min(before.samples.length, after.samples.length);
const rows = Array.from({ length: count }, (_, i) => ({ cell: i,
  qDelta: Number((after.samples[i].residualQ - before.samples[i].residualQ).toFixed(4)),
  speedDelta: Number((after.samples[i].residualSpeed - before.samples[i].residualSpeed).toFixed(4)),
  samples: after.samples[i].count }));
console.log(JSON.stringify({ cells: count, changedCells: rows.filter(row => row.qDelta || row.speedDelta).length, rows }, null, 2));
