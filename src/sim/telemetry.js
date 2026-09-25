// Optional benchmark/engineer instrumentation. Does not participate in driving.
export class DrivingTelemetry {
  constructor(){this.time=0;this.energy=[0,0];this.peakCore=0;this.steerTravel=0;this.previousSteer=0;this.samples=[];this.sectors=[];this.sector=0;this.sectorStart=0;}
  sample(car,dt,aiMs) {
    this.time+=dt;
    for(let i=0;i<4;i++) {
      this.energy[i<2?0:1]+=car.wheels[i].tyre.slipPower*dt;
      this.peakCore=Math.max(this.peakCore,car.wheels[i].tyre.core);
    }
    this.steerTravel+=Math.abs(car.controls.steer-this.previousSteer);this.previousSteer=car.controls.steer;
    if(Number.isFinite(aiMs))this.samples.push(aiMs);
    if(car.race?.sector>this.sector){this.sectors.push(this.time-this.sectorStart);this.sectorStart=this.time;this.sector=car.race.sector;}
  }
  summary(){
    const sorted=this.samples.slice().sort((a,b)=>a-b),percentile=p=>sorted[Math.min(sorted.length-1,Math.floor(sorted.length*p))]??0;
    return {slipEnergyMJ:this.energy.map(v=>+(v/1e6).toFixed(3)),peakCore:+this.peakCore.toFixed(1),steerTravel:+this.steerTravel.toFixed(2),
      sectors:this.sectors.map(v=>+v.toFixed(3)),stepMs:{median:+percentile(.5).toFixed(3),p95:+percentile(.95).toFixed(3),p99:+percentile(.99).toFixed(3)}};
  }
}
