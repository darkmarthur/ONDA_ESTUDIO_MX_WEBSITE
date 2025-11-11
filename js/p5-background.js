// js/flow-boids-reynolds.js
(function () {
  const CFG = {
    // Appearance
    bgMode: 'transparent',    // 'transparent' or 'solid'
    bgColor: [0, 0, 0, 1],    // only used if bgMode === 'solid'
    boidHue: 210,             // blue
    boidSat: 85,
    boidBright: 80,
    boidAlpha: 0.95,
    showTrails: false,
    trailAlpha: 0.04,

    // Rendering: 'triangle' (original), 'dot' (circle), 'polygon' (n-sided)
    renderMode: 'dot',    // change to 'triangle' or 'dot' as needed
    polygonSides: 6,
    polygonFill: true,
    polygonStrokeWeight: 0.6,
    dotSizeMultiplier: 1.0,

    // Population control (maxBoids will be adjusted per device)
    areaPerBoid: 500,
    minBoids: 40,
    maxBoids: 1200,         // default (may be lowered for mobile)
    forceFixedCount: 1000,  // if set, forces this exact count (still honored)
    
    // Flocking parameters
    perceptionRadius: 50,
    cohesionRadius: 50,
    separationRadius: 30,
    maxSpeed: 3,
    maxForce: 0.2,
    alignStrength: 1.0,
    cohesionStrength: 0.8,
    separationStrength: 1.5,

    // Boid size & variety
    boidSize: 3,
    sizeJitter: 0.1,
    sizeVariety: 0.25,
    speedVariety: 0.25,

    // Noise/time
    noiseDrift: 0.12,

    // Mouse "void" agent
    mouseRadius: 500,      // radius in pixels
    mouseStrength: 10,     // multiplier for repulsion force
    mouseActiveWindowMs: 60, // consider mouse "active" for this many ms after movement

    // Misc
    frameRate: 50,
    pxRatioMax: 2
  };

  // Responsive adjustment: if mobile-ish, reduce maxBoids
  function adjustMaxBoidsForDevice() {
    // consider mobile if UA contains mobile tokens OR viewport narrow
    const ua = (navigator && navigator.userAgent) ? navigator.userAgent : '';
    const mobileUA = /Mobi|Android|iPhone|iPad|iPod|Windows Phone/i.test(ua);
    const narrow = (window.innerWidth || document.documentElement.clientWidth) < 900; // <= tablet widths considered narrow
    // Treat iPad as 'desktop/tablet' with higher budget — only phones narrow and mobile UA reduce more
    // final heuristic: if mobileUA AND narrow => mobile phone
    const isMobilePhone = mobileUA && narrow;
    CFG.maxBoids = isMobilePhone ? 300 : 1200;
    CFG.boidSize = isMobilePhone ? 2 : 3;
    // guard lower bound not to be below minBoids
    CFG.maxBoids = Math.max(CFG.maxBoids, CFG.minBoids);
  }

  function computeBoidCount(W, H) {
    if (Number.isInteger(CFG.forceFixedCount) && CFG.forceFixedCount > 0) {
      return Math.min(CFG.maxBoids, Math.max(CFG.minBoids, CFG.forceFixedCount));
    }
    const area = Math.max(40000, W * H);
    const n = Math.round(area / CFG.areaPerBoid);
    return Math.max(CFG.minBoids, Math.min(CFG.maxBoids, n));
  }

  // track mouse globally (canvas has pointer-events:none so use window listener)
  const mousePos = { x: -9999, y: -9999 };
  let mouseMovedAt = 0;
  window.addEventListener('mousemove', (e) => {
    mousePos.x = e.clientX;
    mousePos.y = e.clientY;
    mouseMovedAt = Date.now();
  });

  class Boid {
    constructor(p, x, y) {
      this.p = p;
      this.pos = p.createVector(x, y);
      const baseSpeed = CFG.maxSpeed * (1 - CFG.speedVariety + p.random() * CFG.speedVariety * 2);
      this.vel = p.createVector(p.random(-1, 1), p.random(-1, 1));
      this.vel.setMag(baseSpeed || 1);
      this.acc = p.createVector(0, 0);
      this.maxSpeed = CFG.maxSpeed * (0.9 + p.random() * 0.2);
      this.maxForce = CFG.maxForce * (0.8 + p.random() * 0.4);
      const jitter = 1 + (p.random() - 0.5) * CFG.sizeJitter * 2;
      this.size = CFG.boidSize * (1 - CFG.sizeVariety + p.random() * CFG.sizeVariety * 2) * jitter;
      this.hue = CFG.boidHue + p.random(-8, 8);
    }

    edges(p) {
      if (this.pos.x > p.width) this.pos.x = 0;
      if (this.pos.x < 0) this.pos.x = p.width;
      if (this.pos.y > p.height) this.pos.y = 0;
      if (this.pos.y < 0) this.pos.y = p.height;
    }

    applyForce(f) {
      this.acc.add(f);
    }

    flock(p, boids, time) {
      const alignR = CFG.perceptionRadius;
      const cohR = CFG.cohesionRadius;
      const sepR = CFG.separationRadius;

      let steerAlign = p.createVector(0, 0);
      let steerCoh = p.createVector(0, 0);
      let steerSep = p.createVector(0, 0);
      let totalAlign = 0, totalCoh = 0, totalSep = 0;

      for (let other of boids) {
        if (other === this) continue;
        const d = p.dist(this.pos.x, this.pos.y, other.pos.x, other.pos.y);

        if (d < alignR) {
          steerAlign.add(other.vel);
          totalAlign++;
        }
        if (d < cohR) {
          steerCoh.add(other.pos);
          totalCoh++;
        }
        if (d > 0 && d < sepR) {
          const diff = p.createVector(this.pos.x - other.pos.x, this.pos.y - other.pos.y);
          diff.normalize();
          diff.div(d);
          steerSep.add(diff);
          totalSep++;
        }
      }

      if (totalAlign > 0) {
        steerAlign.div(totalAlign);
        steerAlign.setMag(this.maxSpeed);
        steerAlign.sub(this.vel);
        steerAlign.limit(this.maxForce);
        steerAlign.mult(CFG.alignStrength);
      }

      if (totalCoh > 0) {
        steerCoh.div(totalCoh);
        steerCoh.sub(this.pos);
        steerCoh.setMag(this.maxSpeed);
        steerCoh.sub(this.vel);
        steerCoh.limit(this.maxForce);
        steerCoh.mult(CFG.cohesionStrength);
      }

      if (totalSep > 0) {
        steerSep.div(totalSep);
        steerSep.setMag(this.maxSpeed);
        steerSep.sub(this.vel);
        steerSep.limit(this.maxForce * 1.6);
        steerSep.mult(CFG.separationStrength);
      }

      // mouse void repulsion: only active shortly after mouse moved
      const now = Date.now();
      let mouseForce = p.createVector(0, 0);
      if (now - mouseMovedAt < CFG.mouseActiveWindowMs) {
        // compute distance to mouse
        const mx = mousePos.x;
        const my = mousePos.y;
        const dMouse = p.dist(this.pos.x, this.pos.y, mx, my);
        if (dMouse > 0 && dMouse < CFG.mouseRadius) {
          // push away inversely proportional to distance
          const diffM = p.createVector(this.pos.x - mx, this.pos.y - my);
          diffM.normalize();
          // stronger when closer
          const strength = (1 - dMouse / CFG.mouseRadius) * CFG.mouseStrength;
          mouseForce = diffM.mult(this.maxSpeed * 0.5 * strength); // scale to feel natural
          // limit to a reasonable force
          mouseForce.limit(this.maxForce * 6);
        }
      }

      // subtle global noise to add organic movement (no mouse required)
      const noiseAngle = (p.noise(this.pos.x * 0.0012, this.pos.y * 0.0012, time * CFG.noiseDrift) - 0.5) * Math.PI * 1.6;
      const noiseForce = p.createVector(Math.cos(noiseAngle), Math.sin(noiseAngle)).mult(0.002);

      // apply
      this.applyForce(steerAlign);
      this.applyForce(steerCoh);
      this.applyForce(steerSep);
      this.applyForce(mouseForce);
      this.applyForce(noiseForce);
    }

    update(p) {
      this.vel.add(this.acc);
      this.vel.limit(this.maxSpeed);
      this.pos.add(this.vel);
      this.acc.mult(0);
    }

    draw(p) {
      p.push();
      p.translate(this.pos.x, this.pos.y);
      const angle = Math.atan2(this.vel.y, this.vel.x);
      p.rotate(angle + Math.PI / 2);

      p.noStroke();
      p.fill(this.hue, CFG.boidSat, CFG.boidBright, CFG.boidAlpha);

      const s = this.size;

      if (CFG.renderMode === 'dot') {
        const ds = s * CFG.dotSizeMultiplier;
        p.ellipse(0, 0, ds, ds);
      } else if (CFG.renderMode === 'polygon') {
        const sides = Math.max(3, Math.round(CFG.polygonSides || 6));
        const radius = s * 1.4;
        if (!CFG.polygonFill) {
          p.noFill();
          p.stroke(this.hue, CFG.boidSat, CFG.boidBright, CFG.boidAlpha);
          p.strokeWeight(CFG.polygonStrokeWeight);
        } else {
          p.noStroke();
        }
        p.beginShape();
        for (let k = 0; k < sides; k++) {
          const a = -Math.PI / 2 + (k / sides) * Math.PI * 2;
          const vx = Math.cos(a) * radius;
          const vy = Math.sin(a) * radius;
          p.vertex(vx, vy);
        }
        p.endShape(p.CLOSE);
        if (!CFG.polygonFill) p.noStroke();
      } else {
        p.beginShape();
        p.vertex(0, -s * 1.6);
        p.vertex(-s * 0.8, s * 1.1);
        p.vertex(s * 0.8, s * 1.1);
        p.endShape(p.CLOSE);
      }

      p.pop();
    }
  }

  function sketch(p) {
    let canvas;
    let boids = [];
    let boidCount = 0;
    let tStart = 0;

    p.setup = function () {
      // first adjust maxBoids for current device/viewport
      adjustMaxBoidsForDevice();

      p.pixelDensity(Math.min(window.devicePixelRatio || 1, CFG.pxRatioMax));
      canvas = p.createCanvas(window.innerWidth, window.innerHeight);
      canvas.id('p5-bg-canvas');

      // keep canvas behind page content and allow pointer events through
      canvas.elt.style.position = 'fixed';
      canvas.elt.style.top = '0';
      canvas.elt.style.left = '0';
      canvas.elt.style.zIndex = '-1';
      canvas.elt.style.pointerEvents = 'none';

      p.noStroke();
      p.frameRate(CFG.frameRate);
      p.clear();

      p.colorMode(p.HSB, 360, 100, 100, 1);

      boidCount = computeBoidCount(p.width, p.height);
      boids = [];
      for (let i = 0; i < boidCount; i++) {
        boids.push(new Boid(p, p.random(p.width), p.random(p.height)));
      }

      tStart = p.millis();
      console.log('[flow-boids-reynolds] boids:', boidCount, 'mode:', CFG.renderMode, 'maxBoids:', CFG.maxBoids);

      // listen for resize changes and re-evaluate device type and boid counts
      window.addEventListener('resize', () => {
        // adjust responsive cap, then resize canvas and reconcile boid list in p.windowResized
        adjustMaxBoidsForDevice();
      });
    };

    p.windowResized = function () {
      p.resizeCanvas(window.innerWidth, window.innerHeight);

      // recompute target count after any responsive cap changes
      const newCount = computeBoidCount(p.width, p.height);
      if (newCount > boids.length) {
        for (let i = 0; i < newCount - boids.length; i++) {
          boids.push(new Boid(p, p.random(p.width), p.random(p.height)));
        }
      } else if (newCount < boids.length) {
        boids.splice(0, boids.length - newCount);
      }
    };

    p.draw = function () {
      if (CFG.bgMode === 'solid') {
        p.clear();
        p.background(CFG.bgColor[0], CFG.bgColor[1], CFG.bgColor[2], CFG.bgColor[3]);
      } else {
        if (CFG.showTrails && CFG.trailAlpha > 0) {
          p.fill(0, 0, 0, CFG.trailAlpha);
          p.noStroke();
          p.rect(0, 0, p.width, p.height);
        } else {
          p.clear();
        }
      }

      const time = (p.millis() - tStart) * 0.001;

      for (let b of boids) {
        b.flock(p, boids, time);
        b.update(p);
        b.edges(p);
        b.draw(p);
      }
    };
  }

  if (typeof p5 === 'undefined') {
    console.error('p5 not found - include p5 before this script.');
  } else {
    new p5(sketch);
  }
})();
