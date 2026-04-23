/* ============================================================
   GRANDROOT — main.js
   Scroll reveal · Counter animations · Nav · Particles · Misc
   ============================================================ */

'use strict';

// ── Utility ──────────────────────────────────────────────────
const $ = (sel, ctx = document) => ctx.querySelector(sel);
const $$ = (sel, ctx = document) => [...ctx.querySelectorAll(sel)];

// ── Nav scroll behaviour ────────────────────────────────────
(function initNav() {
  const nav = $('#nav');
  const hamburger = $('#hamburger');
  const mobileMenu = $('#mobileMenu');
  const mobileLinks = $$('.mobile-link');

  const onScroll = () => {
    nav.classList.toggle('scrolled', window.scrollY > 40);
  };
  window.addEventListener('scroll', onScroll, { passive: true });
  onScroll();

  hamburger.addEventListener('click', () => {
    hamburger.classList.toggle('active');
    mobileMenu.classList.toggle('open');
  });

  mobileLinks.forEach(link => {
    link.addEventListener('click', () => {
      hamburger.classList.remove('active');
      mobileMenu.classList.remove('open');
    });
  });
})();

// ── Scroll reveal ────────────────────────────────────────────
(function initReveal() {
  const elements = $$('.reveal');

  const observer = new IntersectionObserver((entries) => {
    entries.forEach(entry => {
      if (entry.isIntersecting) {
        entry.target.classList.add('visible');
        observer.unobserve(entry.target);
      }
    });
  }, { threshold: 0.12, rootMargin: '0px 0px -40px 0px' });

  elements.forEach(el => observer.observe(el));
})();

// ── Animated counters ─────────────────────────────────────────
(function initCounters() {
  const metrics = $$('.metric');

  const easeOut = t => 1 - Math.pow(1 - t, 3);

  const animateCounter = (el) => {
    const valueEl = el.querySelector('.metric__value');
    if (!valueEl) return;
    const target = parseFloat(valueEl.dataset.target);
    const duration = 2000;
    const start = performance.now();

    const tick = (now) => {
      const elapsed = now - start;
      const progress = Math.min(elapsed / duration, 1);
      const value = Math.round(easeOut(progress) * target);
      valueEl.textContent = value;
      if (progress < 1) requestAnimationFrame(tick);
    };
    requestAnimationFrame(tick);
  };

  const observer = new IntersectionObserver((entries) => {
    entries.forEach(entry => {
      if (entry.isIntersecting) {
        animateCounter(entry.target);
        observer.unobserve(entry.target);
      }
    });
  }, { threshold: 0.3 });

  metrics.forEach(m => observer.observe(m));
})();

// ── Hero particles ───────────────────────────────────────────
(function initParticles() {
  const container = $('#particles');
  if (!container) return;

  const count = window.matchMedia('(prefers-reduced-motion: reduce)').matches ? 0 : 40;

  for (let i = 0; i < count; i++) {
    const p = document.createElement('div');
    p.className = 'particle';
    const left = Math.random() * 60; // keep in left half (robot is right)
    const dur  = 6 + Math.random() * 10;
    const delay = Math.random() * 8;
    const drift = (Math.random() - 0.5) * 80;
    p.style.cssText = `left:${left}%;bottom:${-5 + Math.random() * 20}%;--dur:${dur}s;--delay:${delay}s;--drift:${drift}px;`;
    container.appendChild(p);
  }
})();

// ── CTA section particles ─────────────────────────────────────
(function initCtaParticles() {
  const container = $('#ctaParticles');
  if (!container) return;

  const count = window.matchMedia('(prefers-reduced-motion: reduce)').matches ? 0 : 30;

  for (let i = 0; i < count; i++) {
    const p = document.createElement('div');
    p.className = 'particle';
    const left = Math.random() * 100;
    const dur  = 8 + Math.random() * 12;
    const delay = Math.random() * 10;
    const drift = (Math.random() - 0.5) * 60;
    p.style.cssText = `left:${left}%;bottom:${-5 + Math.random() * 15}%;--dur:${dur}s;--delay:${delay}s;--drift:${drift}px;`;
    container.appendChild(p);
  }
})();

// ── Smooth anchor scrolling (for browsers without CSS support) ──
(function initSmoothScroll() {
  $$('a[href^="#"]').forEach(anchor => {
    anchor.addEventListener('click', e => {
      const target = $(anchor.getAttribute('href'));
      if (!target) return;
      e.preventDefault();
      target.scrollIntoView({ behavior: 'smooth', block: 'start' });
    });
  });
})();

// ── Parallax on hero robot ────────────────────────────────────
(function initParallax() {
  if (window.matchMedia('(prefers-reduced-motion: reduce)').matches) return;

  const heroRobot = $('.hero__robot-visual');
  if (!heroRobot) return;

  let ticking = false;
  const onScroll = () => {
    if (!ticking) {
      requestAnimationFrame(() => {
        const scrolled = window.scrollY;
        const viewH = window.innerHeight;
        if (scrolled < viewH) {
          const pct = scrolled / viewH;
          heroRobot.style.transform = `translateY(calc(-50% + ${pct * 40}px))`;
        }
        ticking = false;
      });
      ticking = true;
    }
  };
  window.addEventListener('scroll', onScroll, { passive: true });
})();

// ── Micro-interaction: button ripple ────────────────────────
(function initRipple() {
  $$('.btn').forEach(btn => {
    btn.addEventListener('click', function(e) {
      const rect = this.getBoundingClientRect();
      const x = e.clientX - rect.left;
      const y = e.clientY - rect.top;

      const ripple = document.createElement('span');
      ripple.style.cssText = `
        position:absolute;
        left:${x}px;top:${y}px;
        width:0;height:0;
        background:rgba(255,255,255,0.15);
        border-radius:50%;
        transform:translate(-50%,-50%);
        pointer-events:none;
        animation:ripple-anim 0.6s ease-out forwards;
      `;

      if (!document.getElementById('ripple-style')) {
        const style = document.createElement('style');
        style.id = 'ripple-style';
        style.textContent = `
          @keyframes ripple-anim {
            to { width:200px; height:200px; opacity:0; }
          }
        `;
        document.head.appendChild(style);
      }

      this.appendChild(ripple);
      setTimeout(() => ripple.remove(), 650);
    });
  });
})();

// ── Spec bar animate on visible ──────────────────────────────
(function initSpecBars() {
  const bars = $$('.spec-card__bar-fill');

  const observer = new IntersectionObserver((entries) => {
    entries.forEach(entry => {
      if (entry.isIntersecting) {
        // Force reflow to trigger CSS transition
        entry.target.style.width = '0';
        requestAnimationFrame(() => {
          requestAnimationFrame(() => {
            entry.target.style.width = entry.target.style.getPropertyValue('--pct') ||
              getComputedStyle(entry.target).getPropertyValue('--pct');
          });
        });
        observer.unobserve(entry.target);
      }
    });
  }, { threshold: 0.5 });

  bars.forEach(b => observer.observe(b));
})();

// ── Active nav link highlight ────────────────────────────────
(function initActiveNav() {
  const sections = $$('section[id]');
  const navLinks = $$('.nav__links a');

  const observer = new IntersectionObserver((entries) => {
    entries.forEach(entry => {
      if (entry.isIntersecting) {
        const id = entry.target.id;
        navLinks.forEach(link => {
          link.style.color = link.getAttribute('href') === `#${id}`
            ? 'var(--white)'
            : '';
        });
      }
    });
  }, { threshold: 0.4 });

  sections.forEach(s => observer.observe(s));
})();

// ── Keyboard accessibility: close mobile menu on Escape ──────
document.addEventListener('keydown', e => {
  if (e.key === 'Escape') {
    $('#hamburger').classList.remove('active');
    $('#mobileMenu').classList.remove('open');
  }
});
