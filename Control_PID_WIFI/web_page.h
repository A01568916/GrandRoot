#pragma once
#include <pgmspace.h>

// Interfaz web embebida en Flash del ESP32.
// Se define en .h separado para evitar que el preprocesador de Arduino IDE
// rompa el raw string literal R"HTML(...)HTML".

const char HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Control PID — Motores Derecho e Izquierdo</title>
<link href="https://cdn.jsdelivr.net/npm/bootstrap-icons@1.11.3/font/bootstrap-icons.min.css" rel="stylesheet">
<style>
:root{--bg:#0d1117;--surface:#161b22;--surface2:#1c2128;--border:#30363d;--accent:#58a6ff;--green:#3fb950;--red:#f85149;--orange:#f0883e;--text:#e6edf3;--muted:#8b949e}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font-family:'Segoe UI',system-ui,sans-serif;min-height:100vh;display:flex;flex-direction:column}
header{background:var(--surface);border-bottom:1px solid var(--border);padding:12px 20px;display:flex;align-items:center;justify-content:space-between}
.brand{display:flex;align-items:center;gap:10px}
.brand-icon{font-size:22px;color:var(--accent)}
.brand-title{font-size:16px;font-weight:700}
.brand-sub{font-size:11px;color:var(--muted)}
#badge{font-size:12px;font-weight:600;padding:4px 12px;border-radius:20px;background:#21262d;color:var(--green);border:1px solid var(--border);display:flex;align-items:center;gap:6px}
#badge::before{content:'';width:8px;height:8px;border-radius:50%;background:currentColor;display:inline-block}
main{flex:1;padding:20px;display:grid;grid-template-columns:280px 1fr;gap:16px}
.col-left{display:flex;flex-direction:column;gap:12px}
.card{background:var(--surface);border:1px solid var(--border);border-radius:10px;padding:14px}
.card-title{font-size:10px;font-weight:700;letter-spacing:.08em;color:var(--muted);text-transform:uppercase;margin-bottom:10px;display:flex;align-items:center;gap:6px}
.estado-badge{text-align:center;padding:8px 10px;border-radius:6px;font-size:13px;font-weight:700;margin-bottom:10px}
.estado-detenido{background:#3d1f1e;color:var(--red);border:1px solid #da3633}
.estado-activo{background:#1a2f1a;color:var(--green);border:1px solid #238636}
.btn-row{display:flex;gap:8px}
.btn-enable{flex:1;padding:10px;border:none;border-radius:8px;font-size:13px;font-weight:700;cursor:pointer;transition:.15s;letter-spacing:.04em}
.btn-activar{background:#238636;color:#fff}.btn-activar:hover{background:#2ea043}
.btn-paro{background:#da3633;color:#fff}.btn-paro:hover{background:#f85149}
.tele-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.tele-item{background:var(--surface2);border-radius:8px;padding:10px 12px;border-left:3px solid var(--border)}
.tele-item.pul{border-left-color:var(--accent)}
.tele-item.err{border-left-color:var(--orange)}
.tele-item.esf{border-left-color:var(--green)}
.tele-item.ref{border-left-color:#a371f7}
.tele-lbl{font-size:10px;color:var(--muted);margin-bottom:3px}
.tele-val{font-size:22px;font-weight:700}
.pid-grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px}
.pid-item{background:var(--surface2);border-radius:8px;padding:8px 10px;text-align:center}
.pid-lbl{font-size:10px;color:var(--muted)}
.pid-val{font-size:15px;font-weight:700;color:var(--accent)}
.col-right{display:flex;flex-direction:column;gap:12px}
.ref-section{display:flex;flex-direction:column;align-items:center;gap:20px;padding:20px}
.ref-display{font-size:72px;font-weight:900;color:var(--accent);line-height:1;text-align:center}
.ref-unit{font-size:16px;color:var(--muted);margin-top:4px}
.slider-wrap{width:100%;max-width:400px}
.slider-label{display:flex;justify-content:space-between;font-size:11px;color:var(--muted);margin-bottom:6px}
input[type=range]{width:100%;height:10px;accent-color:var(--accent);cursor:pointer}
.send-btn{padding:12px 40px;background:var(--accent);color:#0d1117;border:none;border-radius:8px;font-size:14px;font-weight:700;cursor:pointer;transition:.15s;letter-spacing:.04em}
.send-btn:hover{background:#79c0ff}
.send-btn:active{transform:scale(.97)}
.bar-wrap{width:100%;max-width:400px}
.bar-row{display:flex;align-items:center;gap:10px;margin-bottom:8px}
.bar-label{font-size:11px;color:var(--muted);width:80px;text-align:right}
.bar-bg{flex:1;height:14px;background:var(--surface2);border-radius:7px;overflow:hidden;border:1px solid var(--border)}
.bar-fill{height:100%;border-radius:7px;transition:width .3s}
.bar-ref{background:var(--accent)}
.bar-pul{background:var(--green)}
.bar-esf{background:var(--orange)}
.bar-val{font-size:12px;font-weight:700;min-width:32px}
.dir-pad{display:grid;grid-template-columns:56px 56px 56px;grid-template-rows:56px 56px 56px;gap:6px;margin:20px auto}
.dir-btn{background:var(--surface2);border:1px solid var(--border);border-radius:8px;color:var(--text);font-size:24px;cursor:pointer;display:flex;align-items:center;justify-content:center;transition:.1s;user-select:none}
.dir-btn:active,.dir-btn.pressed{background:var(--accent);color:#0d1117}
.speed-row{display:flex;align-items:center;gap:10px;margin:12px 0}
.speed-row label{font-size:11px;color:var(--muted);white-space:nowrap}
input[type=range]{flex:1;accent-color:var(--accent)}
#speedVal{font-size:13px;font-weight:700;min-width:30px;text-align:right}
.tele-cols{display:grid;grid-template-columns:1fr 1fr;gap:12px}
</style>
</head>
<body>
<header>
  <div class="brand">
    <i class="bi bi-cpu brand-icon"></i>
    <div>
      <div class="brand-title">Control PID — Motor Derecho e Izquierdo</div>
      <div class="brand-sub">Planta Llanta — ESP32 &bull; 192.168.4.1</div>
    </div>
  </div>
  <div id="badge"><span>GrandRoot</span></div>
</header>

<main>
  <!-- Columna izquierda -->
  <div class="col-left">

    <!-- Motor enable - Control único -->
    <div class="card">
      <div class="card-title"><i class="bi bi-power"></i> Motor</div>
      <div id="estado-badge" class="estado-badge estado-detenido">⛔ MOTOR DETENIDO</div>
      <div class="btn-row">
        <button class="btn-enable btn-activar" onclick="setEnable(true)">
          <i class="bi bi-play-fill"></i> ACTIVAR
        </button>
        <button class="btn-enable btn-paro" onclick="setEnable(false)">
          <i class="bi bi-stop-fill"></i> PARO
        </button>
      </div>
    </div>

    <!-- Velocidad máxima -->
    <div class="card">
      <div class="card-title" style="justify-content:center"><i class="bi bi-speedometer2"></i> Velocidad</div>
      <div style="padding:10px">
        <div style="font-size:36px;font-weight:900;color:var(--accent);text-align:center;margin-bottom:10px" id="speedDisplay">0</div>
        <div class="speed-row">
          <span style="font-size:11px">0</span>
          <input type="range" id="speedSlider" min="0" max="100" value="0"
            oninput="updateSpeed(this.value)">
          <span style="font-size:11px">100</span>
        </div>
        <div style="text-align:center;font-size:12px;color:var(--muted);margin-top:8px">pul/s</div>
      </div>
    </div>

    <!-- Control de dirección -->
    <div class="card">
      <div class="card-title" style="justify-content:center"><i class="bi bi-arrows-move"></i> Dirección</div>
      <div class="dir-pad">
        <div></div>
        <div class="dir-btn" id="btn-up" onmousedown="dpad('up',true)" onmouseup="dpad('up',false)" ontouchstart="dpad('up',true)" ontouchend="dpad('up',false)"><i class="bi bi-arrow-up"></i></div>
        <div></div>
        <div class="dir-btn" id="btn-left" onmousedown="dpad('left',true)" onmouseup="dpad('left',false)" ontouchstart="dpad('left',true)" ontouchend="dpad('left',false)"><i class="bi bi-arrow-left"></i></div>
        <div class="dir-btn" id="btn-stop" onclick="parar()" style="background:var(--red);font-size:20px"><i class="bi bi-stop-fill"></i></div>
        <div class="dir-btn" id="btn-right" onmousedown="dpad('right',true)" onmouseup="dpad('right',false)" ontouchstart="dpad('right',true)" ontouchend="dpad('right',false)"><i class="bi bi-arrow-right"></i></div>
        <div></div>
        <div class="dir-btn" id="btn-down" onmousedown="dpad('down',true)" onmouseup="dpad('down',false)" ontouchstart="dpad('down',true)" ontouchend="dpad('down',false)"><i class="bi bi-arrow-down"></i></div>
        <div></div>
      </div>
    </div>

  </div>

  <!-- Telemetría de ambos motores -->
  <div class="tele-cols">
    <div class="card">
      <div class="card-title"><i class="bi bi-speedometer2"></i> Telemetría (Derecho)</div>
      <div class="tele-grid">
        <div class="tele-item ref">
          <div class="tele-lbl">Referencia</div>
          <div class="tele-val" id="tRef_d">0</div>
        </div>
        <div class="tele-item pul">
          <div class="tele-lbl">Pulsos</div>
          <div class="tele-val" id="tPul_d">—</div>
        </div>
        <div class="tele-item err">
          <div class="tele-lbl">Error</div>
          <div class="tele-val" id="tErr_d">—</div>
        </div>
        <div class="tele-item esf">
          <div class="tele-lbl">Esfuerzo</div>
          <div class="tele-val" id="tEsf_d">—</div>
        </div>
      </div>
    </div>
    <div class="card">
      <div class="card-title"><i class="bi bi-speedometer2"></i> Telemetría (Izquierdo)</div>
      <div class="tele-grid">
        <div class="tele-item ref">
          <div class="tele-lbl">Referencia</div>
          <div class="tele-val" id="tRef_i">0</div>
        </div>
        <div class="tele-item pul">
          <div class="tele-lbl">Pulsos</div>
          <div class="tele-val" id="tPul_i">—</div>
        </div>
        <div class="tele-item err">
          <div class="tele-lbl">Error</div>
          <div class="tele-val" id="tErr_i">—</div>
        </div>
        <div class="tele-item esf">
          <div class="tele-lbl">Esfuerzo</div>
          <div class="tele-val" id="tEsf_i">—</div>
        </div>
      </div>
    </div>
  </div>
</main>

<script>
let motoresActivos = false;
let dpadState = {up: false, down: false, left: false, right: false};
let dpadTimer = null;

function updateSpeed(val) {
  document.getElementById('speedDisplay').textContent = val;
  enviarMovimiento();
}

async function setEnable(on) {
  try {
    const r = await fetch('/api/enable', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ on })
    });
    const j = await r.json();
    motoresActivos = j.enabled;
    const badge = document.getElementById('estado-badge');
    if (motoresActivos) {
      badge.className = 'estado-badge estado-activo';
      badge.textContent = '\u2705 MOTOR ACTIVO';
    } else {
      badge.className = 'estado-badge estado-detenido';
      badge.textContent = '\u26d4 MOTOR DETENIDO';
    }
    if (!motoresActivos) parar();
  } catch(e) { console.error(e); }
}

async function enviarMovimiento() {
  if (!motoresActivos) return;
  const spd = parseInt(document.getElementById('speedSlider').value) / 100;
  let vx = 0, vy = 0;
  
  if (dpadState.up)    vx = spd;
  if (dpadState.down)  vx = -spd;
  if (dpadState.left)  vy = -spd;
  if (dpadState.right) vy = spd;
  
  try {
    const r = await fetch('/api/mover', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ vx, vy })
    });
    const j = await r.json();
    if (j.ok) {
      document.getElementById('tRef_d').textContent = j.ref_der;
      document.getElementById('tRef_i').textContent = j.ref_izq;
    }
  } catch(e) { console.error(e); }
}

function parar() {
  dpadState = {up: false, down: false, left: false, right: false};
  document.getElementById('btn-up').classList.remove('pressed');
  document.getElementById('btn-down').classList.remove('pressed');
  document.getElementById('btn-left').classList.remove('pressed');
  document.getElementById('btn-right').classList.remove('pressed');
  clearInterval(dpadTimer);
  enviarMovimiento();
}

function dpad(dir, pressed) {
  dpadState[dir] = pressed;
  document.getElementById('btn-' + dir).classList.toggle('pressed', pressed);
  clearInterval(dpadTimer);
  
  if (Object.values(dpadState).some(Boolean)) {
    enviarMovimiento();
    dpadTimer = setInterval(enviarMovimiento, 100);
  } else {
    parar();
  }
}

const keyMap = {ArrowUp: 'up', ArrowDown: 'down', ArrowLeft: 'left', ArrowRight: 'right'};
document.addEventListener('keydown', e => {
  if (keyMap[e.key] && !dpadState[keyMap[e.key]]) {
    e.preventDefault();
    dpad(keyMap[e.key], true);
  }
  if (e.key === ' ') {
    e.preventDefault();
    parar();
  }
});
document.addEventListener('keyup', e => {
  if (keyMap[e.key]) {
    e.preventDefault();
    dpad(keyMap[e.key], false);
  }
});

setInterval(() => {
  fetch('/api/tele?motor=derecho').then(r => r.json()).then(d => {
    document.getElementById('tPul_d').textContent = d.pulsos;
    document.getElementById('tErr_d').textContent = d.error;
    document.getElementById('tEsf_d').textContent = d.esfuerzo;
  }).catch(() => {});
  
  fetch('/api/tele?motor=izquierdo').then(r => r.json()).then(d => {
    document.getElementById('tPul_i').textContent = d.pulsos;
    document.getElementById('tErr_i').textContent = d.error;
    document.getElementById('tEsf_i').textContent = d.esfuerzo;
  }).catch(() => {});
}, 1000);
</script>
</body>
</html>)HTML";
