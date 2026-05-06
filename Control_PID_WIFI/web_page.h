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
<title>Control PID — Motor Derecho</title>
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
main{flex:1;padding:20px;display:grid;grid-template-columns:280px 1fr;gap:16px;align-items:start}
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
</style>
</head>
<body>
<header>
  <div class="brand">
    <i class="bi bi-cpu brand-icon"></i>
    <div>
      <div class="brand-title">Control PID — Motor Derecho</div>
      <div class="brand-sub">Planta Llanta — ESP32 &bull; 192.168.4.1</div>
    </div>
  </div>
  <div id="badge"><span>GrandRoot</span></div>
</header>

<main>
  <!-- Columna izquierda -->
  <div class="col-left">

    <!-- Motor enable -->
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

    <!-- Telemetría -->
    <div class="card">
      <div class="card-title"><i class="bi bi-speedometer2"></i> Telemetría (cada 1 s)</div>
      <div class="tele-grid">
        <div class="tele-item ref">
          <div class="tele-lbl">Referencia (pul)</div>
          <div class="tele-val" id="tRef">0</div>
        </div>
        <div class="tele-item pul">
          <div class="tele-lbl">Pulsos medidos</div>
          <div class="tele-val" id="tPul">—</div>
        </div>
        <div class="tele-item err">
          <div class="tele-lbl">Error</div>
          <div class="tele-val" id="tErr">—</div>
        </div>
        <div class="tele-item esf">
          <div class="tele-lbl">Esfuerzo (DAC)</div>
          <div class="tele-val" id="tEsf">—</div>
        </div>
      </div>
    </div>

    <!-- Ganancias PID -->
    <div class="card">
      <div class="card-title"><i class="bi bi-sliders"></i> Ganancias PID</div>
      <div class="pid-grid">
        <div class="pid-item"><div class="pid-lbl">Kp</div><div class="pid-val">0.800</div></div>
        <div class="pid-item"><div class="pid-lbl">Ki</div><div class="pid-val">0.400</div></div>
        <div class="pid-item"><div class="pid-lbl">Kd</div><div class="pid-val">0.040</div></div>
      </div>
    </div>

  </div>

  <!-- Columna derecha -->
  <div class="col-right">

    <!-- Control de referencia -->
    <div class="card">
      <div class="card-title" style="justify-content:center"><i class="bi bi-bullseye"></i> Referencia de velocidad</div>
      <div class="ref-section">
        <div>
          <div class="ref-display" id="refDisplay">0</div>
          <div class="ref-unit" style="text-align:center">pulsos / segundo</div>
        </div>
        <div class="slider-wrap">
          <div class="slider-label"><span>0</span><span>25</span><span>50</span><span>75</span><span>100</span></div>
          <input type="range" id="refSlider" min="0" max="100" value="0"
            oninput="document.getElementById('refDisplay').textContent=this.value">
        </div>
        <button class="send-btn" onclick="enviarRef()">
          <i class="bi bi-send-fill"></i> Enviar referencia
        </button>
      </div>
    </div>

    <!-- Barras de estado -->
    <div class="card">
      <div class="card-title"><i class="bi bi-bar-chart-fill"></i> Estado visual</div>
      <div style="padding:8px 0">
        <div class="bar-row">
          <span class="bar-label">Referencia</span>
          <div class="bar-bg"><div class="bar-fill bar-ref" id="barRef" style="width:0%"></div></div>
          <span class="bar-val" id="bvRef">0</span>
        </div>
        <div class="bar-row">
          <span class="bar-label">Pulsos</span>
          <div class="bar-bg"><div class="bar-fill bar-pul" id="barPul" style="width:0%"></div></div>
          <span class="bar-val" id="bvPul">0</span>
        </div>
        <div class="bar-row">
          <span class="bar-label">Esfuerzo</span>
          <div class="bar-bg"><div class="bar-fill bar-esf" id="barEsf" style="width:0%"></div></div>
          <span class="bar-val" id="bvEsf">0</span>
        </div>
      </div>
    </div>

  </div>
</main>

<script>
async function setEnable(on) {
  try {
    const r = await fetch('/api/enable', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ on })
    });
    const j = await r.json();
    const badge = document.getElementById('estado-badge');
    if (j.enabled) {
      badge.className = 'estado-badge estado-activo';
      badge.textContent = '\u2705 MOTOR ACTIVO';
    } else {
      badge.className = 'estado-badge estado-detenido';
      badge.textContent = '\u26d4 MOTOR DETENIDO';
    }
  } catch(e) { console.error(e); }
}

async function enviarRef() {
  const ref = parseInt(document.getElementById('refSlider').value);
  try {
    await fetch('/api/setref', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ ref })
    });
  } catch(e) { console.error(e); }
}

function setBar(barId, valId, value, max) {
  const pct = Math.min(100, Math.round(value / max * 100));
  document.getElementById(barId).style.width = pct + '%';
  document.getElementById(valId).textContent = value;
}

setInterval(() => {
  fetch('/api/tele').then(r => r.json()).then(d => {
    document.getElementById('tRef').textContent  = d.referencia;
    document.getElementById('tPul').textContent  = d.pulsos;
    document.getElementById('tErr').textContent  = d.error;
    document.getElementById('tEsf').textContent  = d.esfuerzo;
    setBar('barRef', 'bvRef', d.referencia, 100);
    setBar('barPul', 'bvPul', d.pulsos,     100);
    setBar('barEsf', 'bvEsf', d.esfuerzo,  255);
    const badge = document.getElementById('estado-badge');
    if (d.activo) {
      badge.className = 'estado-badge estado-activo';
      badge.textContent = '\u2705 MOTOR ACTIVO';
    } else {
      badge.className = 'estado-badge estado-detenido';
      badge.textContent = '\u26d4 MOTOR DETENIDO';
    }
  }).catch(() => {});
}, 1000);
</script>
</body>
</html>)HTML";
