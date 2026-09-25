import { Session } from './sim/session.js';
import { VortexDriver } from './ai/vortex/vortex-driver.js';

/** Thin host adapter: canonical Session continues to own reset, timing, physics and contacts. */
export class VortexSession extends Session {
  constructor(track, options = {}) {
    super(track, options);
    this.vortexOptions = options.vortex ?? { physical: options.physical, oracle: options.oracle };
  }
  reset() {
    super.reset();
    if (this.cars && this.lineFor) {
      this.drivers = this.cars.map(car => new VortexDriver(car.id, this.lineFor(car), {
        aggression: this.aggression, seed: 0x564f5254, ...this.vortexOptions,
      }));
    }
  }
}
