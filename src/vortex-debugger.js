import * as THREE from 'three';

export class VortexDebugger {
  constructor(scene, session, onCamera = () => {}) {
    this.scene = scene; this.session = session; this.root = new THREE.Group();
    this.root.renderOrder = 30; this.root.visible = false; scene.add(this.root);
    this.enabled = false; this.keepPaths = false; this.focus = 0; this.last = 0; this.frozen = false;
    this.panel = document.createElement('section'); this.panel.id = 'ai-debug'; this.panel.hidden = true;
    this.panel.innerHTML = `<div class="debug-header"><span>VORTEX / RACE ENGINEER</span><button id="debug-close" aria-label="Close VORTEX debugger">×</button></div>
      <div class="debug-select"><label>OBSERVE <select id="debug-car">${session.cars.map(car => `<option value="${car.id}">${car.id === 0 ? 'VORTEX · 07' : car.name}</option>`).join('')}</select></label><button id="debug-freeze">FREEZE PATHS</button></div>
      <div id="debug-status"></div><div id="debug-metrics"></div><div class="debug-legend"><span style="--c:#b8ff64">SELECTED</span><span style="--c:#6badcb">CANDIDATES</span><span style="--c:#ffbc52">RIVALS</span></div><p id="debug-reason"></p>
      <div class="debug-foot">120 HZ EXECUTION · 60 HZ DYNAMICS · 27 HZ OPPORTUNITY<br><kbd>B</kbd> TOGGLE · <kbd>P</kbd> AI DEMO</div>`;
    document.querySelector('#app').append(this.panel);
    this.panel.querySelector('#debug-close').onclick = () => this.toggle(false);
    this.panel.querySelector('#debug-car').onchange = event => { this.focus = Number(event.target.value); this.last = 0; };
    this.panel.querySelector('#debug-freeze').onclick = event => {
      this.frozen = !this.frozen; event.currentTarget.textContent = this.frozen ? 'LIVE PATHS' : 'FREEZE PATHS';
    };
    const camera = document.createElement('button'); camera.className = 'debug-camera'; camera.textContent = 'TACTICAL / DRIVER CAMERA'; camera.onclick = onCamera;
    this.panel.querySelector('.debug-foot').prepend(camera);
    this.battleOverlay = document.createElement('aside'); this.battleOverlay.id = 'battle-overlay'; this.battleOverlay.hidden = true;
    this.battleOverlay.innerHTML = '<span class="attack">MULTI-CORNER PLAN</span><span class="defence">WORLD MODEL</span><small>Selected trajectory and evaluated corridor candidates</small>';
    document.querySelector('#app').append(this.battleOverlay);
  }
  get pathsEnabled() { return this.enabled || this.keepPaths; }
  toggle(value = !this.enabled) {
    this.enabled = value; this.root.visible = this.pathsEnabled; this.panel.hidden = !value;
    this.battleOverlay.hidden = !this.pathsEnabled; return value;
  }
  clear() {
    for (const child of [...this.root.children]) {
      child.geometry?.dispose();
      if (Array.isArray(child.material)) child.material.forEach(material => material.dispose()); else child.material?.dispose();
      this.root.remove(child);
    }
  }
  line(points, color, opacity = 1, width = 1) {
    if (!points || points.length < 2) return;
    const geometry = new THREE.BufferGeometry().setFromPoints(points.map(point => new THREE.Vector3(point.x, point.y ?? .14, point.z)));
    const material = new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity, linewidth: width, depthTest: false });
    const line = new THREE.Line(geometry, material); line.renderOrder = 31; this.root.add(line);
  }
  update(now) {
    this.battleOverlay.hidden = !this.pathsEnabled;
    if (!this.pathsEnabled || now - this.last < 120) return;
    this.last = now;
    const driver = this.session.drivers[this.focus], car = this.session.cars[this.focus];
    if (!driver || !car) return;
    if (!this.frozen) { this.clear();
      for (const candidate of driver.debug.candidates ?? []) if (candidate.id !== driver.plan?.id)
        this.line(candidate.points, '#6badcb', .19);
      this.line(driver.selectedTrajectory?.points, '#b8ff64', 1, 2);
      for (const rival of driver.lastOpponents ?? []) {
        const point = this.session.track.at(rival.s, rival.lateral);
        const future = this.session.track.at(rival.s + rival.longitudinalSpeed * 2.2, rival.lateral + rival.lateralSpeed * 1.1);
        this.line([point, future], '#ffbc52', .78);
      }
    }
    const d = driver.diagnostics, telemetry = d.telemetry;
    this.panel.querySelector('#debug-status').innerHTML = `<b>${d.state}</b><span>${car.name}</span>`;
    this.panel.querySelector('#debug-metrics').innerHTML = [
      ['SELECTED CORRIDOR', d.selected ?? 'waiting'], ['TARGET SPEED', `${(d.targetSpeed * 3.6).toFixed(0)} km/h`],
      ['CANDIDATES', d.candidates], ['BRAKE EVENT', d.brakeReason], ['SOLVER', `${d.telemetry.meanSolverMs.toFixed(2)} ms mean / ${d.telemetry.solverMaxMs.toFixed(2)} ms max`],
      ['SAFETY', telemetry.safetyInterventions], ['FALLBACKS', telemetry.controllerFallbacks],
      ['COMBAT TIME', `${telemetry.meaningfulInteractionTime.toFixed(2)} s`], ['RETAINED PASSES', telemetry.retainedPasses],
      ['CLEAN LEARNING', `${telemetry.cleanLearnerCells} microsectors`], ['GRIP / BRAKE', `${d.envelope.mu.toFixed(2)} μ / ${d.envelope.brake.toFixed(1)} m/s²`],
    ].map(([key, value]) => `<span>${key}<b>${value}</b></span>`).join('');
    this.panel.querySelector('#debug-reason').textContent = `Space-time optimization · ${driver.strategy.reason} · ${driver.planner.brakeEvents.events.length} station-coordinate brake events`;
    this.battleOverlay.querySelector('.attack').textContent = `PLAN · ${driver.plan?.id ?? 'WAITING'}`;
    this.battleOverlay.querySelector('.defence').textContent = `${driver.lastOpponents.length} opponent hypotheses`;
  }
}
