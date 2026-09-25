import * as THREE from 'three';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { UnrealBloomPass } from 'three/addons/postprocessing/UnrealBloomPass.js';
import { OutputPass } from 'three/addons/postprocessing/OutputPass.js';
import { ShaderPass } from 'three/addons/postprocessing/ShaderPass.js';
import { FXAAShader } from 'three/addons/shaders/FXAAShader.js';

export class VisualFinish {
  constructor(renderer,scene,camera){
    this.renderer=renderer;this.scene=scene;this.camera=camera;this.mode='performance';renderer.info.autoReset=false;
  }
  setQuality(mode){
    this.mode=mode;
    if(mode!=='performance'&&!this.composer){
      const w = Math.max(1, Math.floor(window.innerWidth || 1280));
      const h = Math.max(1, Math.floor(window.innerHeight || 720));
      // In WebGL2 on Firefox/ANGLE, multisampled renderbuffers with HalfFloatType (samples: 4)
      // cause GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT (0x8cd6) "Attachment has no width or height".
      // Since FXAAShader is used for antialiasing, samples should be 0 (texture-based render target).
      const target=new THREE.WebGLRenderTarget(w, h, {
        type: THREE.HalfFloatType,
        minFilter: THREE.LinearFilter,
        magFilter: THREE.LinearFilter,
        format: THREE.RGBAFormat,
        samples: 0
      });
      this.composer=new EffectComposer(this.renderer,target);
      this.composer.addPass(new RenderPass(this.scene,this.camera));
      this.bloom=new UnrealBloomPass(new THREE.Vector2(Math.max(64, Math.floor(w / 2)), Math.max(64, Math.floor(h / 2))),.13,.45,1.6);
      this.composer.addPass(this.bloom);
      this.composer.addPass(new OutputPass());
      // The upstream shader's -100 bias exceeds ANGLE's supported range.
      this.fxaa=new ShaderPass({...FXAAShader,fragmentShader:FXAAShader.fragmentShader.replaceAll('-100.0','-16.0')});
      this.composer.addPass(this.fxaa);
    }
    if(this.bloom)this.bloom.strength=mode==='ultra'?.2:.12;
    const w = Math.max(1, Math.floor(window.innerWidth || 1280));
    const h = Math.max(1, Math.floor(window.innerHeight || 720));
    this.resize(w, h);
  }
  resize(width,height){
    if(!this.composer)return;
    const w = Math.max(1, Math.floor(width || window.innerWidth || 1));
    const h = Math.max(1, Math.floor(height || window.innerHeight || 1));
    const ratio=this.renderer.getPixelRatio() || 1;
    this.composer.setPixelRatio(ratio);
    this.composer.setSize(w, h);
    if(this.fxaa?.material?.uniforms?.resolution){
      this.fxaa.material.uniforms.resolution.value.set(1/(w*ratio), 1/(h*ratio));
    }
  }
  render(){
    this.renderer.info.reset();
    const w = this.renderer.domElement?.width ?? 0;
    const h = this.renderer.domElement?.height ?? 0;
    if (w <= 0 || h <= 0) return;
    if(this.mode==='performance') {
      this.renderer.render(this.scene,this.camera);
    } else {
      try {
        this.composer.render();
      } catch (err) {
        this.renderer.render(this.scene, this.camera);
      }
    }
  }
}
