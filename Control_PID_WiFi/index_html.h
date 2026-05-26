// index_html.h
// Página web embebida — Control PID WiFi (AP mode)
// La comunicación Serial fue reemplazada por WebSocket ws://192.168.4.1/ws

#pragma once

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Control PID — WiFi</title>

<link href="https://cdn.jsdelivr.net/npm/bootstrap-icons@1.11.3/font/bootstrap-icons.min.css" rel="stylesheet">
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.3/dist/chart.umd.min.js"></script>

<style>
:root{
--bg:#0d1117;
--surface:#161b22;
--surface2:#1c2128;
--border:#30363d;
--accent:#58a6ff;
--green:#3fb950;
--red:#f85149;
--orange:#f0883e;
--purple:#a371f7;
--text:#e6edf3;
--muted:#8b949e
}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font-family:'Segoe UI',system-ui,sans-serif;min-height:100vh;display:flex;flex-direction:column}
header{background:var(--surface);border-bottom:1px solid var(--border);padding:12px 20px;display:flex;align-items:center;justify-content:space-between}
.brand{display:flex;align-items:center;gap:10px}
.brand-icon{font-size:22px;color:var(--accent)}
.brand-title{font-size:16px;font-weight:700}
.brand-sub{font-size:11px;color:var(--muted)}
#badge{font-size:12px;font-weight:600;padding:4px 12px;border-radius:20px;background:#21262d;color:var(--red);border:1px solid var(--border);display:flex;align-items:center;gap:6px}
#badge.connected{color:var(--green)}
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
.btn-activar{background:#238636;color:#fff}
.btn-activar:hover{background:#2ea043}
.btn-paro{background:#da3633;color:#fff}
.btn-paro:hover{background:#f85149}
.tele-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.tele-item{background:var(--surface2);border-radius:8px;padding:10px 12px;border-left:3px solid var(--border)}
.tele-item.pul{border-left-color:var(--accent)}
.tele-item.err{border-left-color:var(--orange)}
.tele-item.esf{border-left-color:var(--green)}
.tele-item.ref{border-left-color:#a371f7}
.tele-lbl{font-size:10px;color:var(--muted);margin-bottom:3px}
.tele-val{font-size:22px;font-weight:700}
.dir-pad{display:grid;grid-template-columns:56px 56px 56px;grid-template-rows:56px 56px 56px;gap:6px;margin:20px auto}
.dir-btn{background:var(--surface2);border:1px solid var(--border);border-radius:8px;color:var(--text);font-size:24px;cursor:pointer;display:flex;align-items:center;justify-content:center;transition:.1s;user-select:none}
.dir-btn:active,.dir-btn.pressed{background:var(--accent);color:#0d1117}
.speed-row{display:flex;align-items:center;gap:10px;margin:12px 0}
.speed-row label{font-size:11px;color:var(--muted);white-space:nowrap}
input[type=range]{flex:1;accent-color:var(--accent)}
.tele-cols{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.charts{display:flex;flex-direction:column;gap:14px;margin-top:20px}
.chart-card{background:var(--surface);border:1px solid var(--border);border-radius:10px;padding:18px 22px}
.chart-card .ch-header{display:flex;align-items:center;gap:9px;margin-bottom:14px}
.chart-card .ch-title{font-size:10px;font-weight:700;letter-spacing:.08em;color:var(--muted);text-transform:uppercase}
.chart-card canvas{max-height:140px}
.wifi-btn{width:100%;padding:12px;border:none;border-radius:8px;background:var(--accent);color:#0d1117;font-weight:800;font-size:13px;cursor:pointer;margin-bottom:6px}
.wifi-btn:hover{background:#79c0ff}
.wifi-btn:disabled{background:#21262d;color:var(--muted);cursor:not-allowed}
.ws-info{font-size:11px;color:var(--muted);text-align:center;margin-top:4px}

/* ——— Botón Admin ——— */
.btn-admin{
  display:flex;align-items:center;justify-content:center;gap:6px;
  width:100%;padding:10px;margin-top:8px;
  border:1px solid var(--purple);border-radius:8px;
  background:transparent;color:var(--purple);
  font-weight:700;font-size:13px;cursor:pointer;transition:.15s;
}
.btn-admin:hover{background:rgba(163,113,247,.15)}

/* ——— Modal Admin ——— */
.modal-overlay{
  display:none;position:fixed;inset:0;z-index:9000;
  background:rgba(0,0,0,.7);backdrop-filter:blur(4px);
  align-items:center;justify-content:center;
}
.modal-overlay.open{display:flex}
.modal{
  background:var(--surface);border:1px solid var(--border);
  border-radius:14px;padding:24px;width:min(480px,95vw);
  max-height:90vh;overflow-y:auto;
  box-shadow:0 8px 40px rgba(0,0,0,.6);
}
.modal-header{display:flex;align-items:center;justify-content:space-between;margin-bottom:18px}
.modal-title{font-size:15px;font-weight:800;display:flex;align-items:center;gap:8px;color:var(--purple)}
.modal-close{background:none;border:none;color:var(--muted);font-size:22px;cursor:pointer;line-height:1;padding:2px 6px;border-radius:4px}
.modal-close:hover{color:var(--text);background:var(--surface2)}
.param-group{display:flex;flex-direction:column;gap:14px}
.param-row{display:grid;grid-template-columns:1fr auto auto;align-items:center;gap:8px;
  background:var(--surface2);border:1px solid var(--border);border-radius:8px;padding:10px 14px}
.param-label{font-size:12px;font-weight:700;color:var(--text)}
.param-sub{font-size:10px;color:var(--muted);margin-top:2px}
.param-input{
  width:90px;background:var(--bg);border:1px solid var(--border);
  border-radius:6px;color:var(--text);font-size:14px;font-weight:700;
  padding:6px 8px;text-align:right;outline:none;
}
.param-input:focus{border-color:var(--accent)}
.param-send{
  background:var(--accent);color:#0d1117;border:none;border-radius:6px;
  font-size:12px;font-weight:800;padding:7px 12px;cursor:pointer;
  white-space:nowrap;transition:.1s;
}
.param-send:hover{background:#79c0ff}
.param-send.ok{background:var(--green)!important;color:#fff}
.param-send.err{background:var(--red)!important;color:#fff}
.modal-footer{margin-top:20px;display:flex;justify-content:flex-end}
.btn-close-modal{padding:10px 22px;border:1px solid var(--border);border-radius:8px;
  background:transparent;color:var(--muted);font-size:13px;font-weight:700;cursor:pointer}
.btn-close-modal:hover{color:var(--text);border-color:var(--text)}
</style>
</head>
<body>

<header>
  <div class="brand">
    <i class="bi bi-cpu brand-icon"></i>
    <div>
      <div class="brand-title">Control PID — Motor Derecho e Izquierdo</div>
      <div class="brand-sub">ESP32 WiFi AP · ws://192.168.4.1/ws</div>
    </div>
  </div>
  <div id="badge">
    <span id="wsStatus">WiFi Desconectado</span>
  </div>
</header>

<main>

<div class="col-left">

<div class="card">
  <div class="card-title"><i class="bi bi-wifi"></i>Conexión WiFi</div>
  <button class="wifi-btn" id="btn-conectar" onclick="connectWS()">
    <i class="bi bi-plug-fill"></i> CONECTAR
  </button>
  <button class="wifi-btn" id="btn-desconectar" onclick="disconnectWS()" style="background:#da3633;color:#fff;display:none">
    <i class="bi bi-plug"></i> DESCONECTAR
  </button>
  <div class="ws-info">Conéctate primero a la red <b>ESP32-Robot</b></div>
  <!-- Botón Admin -->
  <button class="btn-admin" onclick="abrirAdmin()">
    <i class="bi bi-sliders"></i> PANEL ADMIN
  </button>
</div>

<div class="card">
  <div class="card-title"><i class="bi bi-power"></i>Motor</div>
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

<div class="card">
  <div class="card-title" style="justify-content:center"><i class="bi bi-speedometer2"></i>Velocidad</div>
  <div style="padding:10px">
    <div style="font-size:36px;font-weight:900;color:var(--accent);text-align:center;margin-bottom:10px" id="speedDisplay">0</div>
    <div class="speed-row">
      <span style="font-size:11px">0</span>
      <input type="range" id="speedSlider" min="0" max="22" value="0" oninput="updateSpeed(this.value)">
      <span style="font-size:11px">22</span>
    </div>
    <div style="text-align:center;font-size:12px;color:var(--muted);margin-top:8px">pul/s</div>
  </div>
</div>

<div class="card">
  <div class="card-title" style="justify-content:center"><i class="bi bi-arrows-move"></i>Dirección</div>
  <div class="dir-pad">
    <div></div>
    <div class="dir-btn" id="btn-up"
      onmousedown="dpad('up',true)" onmouseup="dpad('up',false)"
      ontouchstart="dpad('up',true)" ontouchend="dpad('up',false)">
      <i class="bi bi-arrow-up"></i>
    </div>
    <div></div>
    <div class="dir-btn" id="btn-left"
      onmousedown="dpad('left',true)" onmouseup="dpad('left',false)"
      ontouchstart="dpad('left',true)" ontouchend="dpad('left',false)">
      <i class="bi bi-arrow-left"></i>
    </div>
    <div class="dir-btn" id="btn-stop" onclick="parar()" style="background:var(--red);font-size:20px">
      <i class="bi bi-stop-fill"></i>
    </div>
    <div class="dir-btn" id="btn-right"
      onmousedown="dpad('right',true)" onmouseup="dpad('right',false)"
      ontouchstart="dpad('right',true)" ontouchend="dpad('right',false)">
      <i class="bi bi-arrow-right"></i>
    </div>
    <div></div>
    <div class="dir-btn" id="btn-down"
      onmousedown="dpad('down',true)" onmouseup="dpad('down',false)"
      ontouchstart="dpad('down',true)" ontouchend="dpad('down',false)">
      <i class="bi bi-arrow-down"></i>
    </div>
    <div></div>
  </div>
</div>

</div><!-- col-left -->

<div>

<div class="tele-cols">

<div class="card">
  <div class="card-title"><i class="bi bi-speedometer2"></i>Telemetría (Derecho)</div>
  <div class="tele-grid">
    <div class="tele-item ref"><div class="tele-lbl">Referencia</div><div class="tele-val" id="tRef_d">0</div></div>
    <div class="tele-item pul"><div class="tele-lbl">Pulsos</div><div class="tele-val" id="tPul_d">—</div></div>
    <div class="tele-item err"><div class="tele-lbl">Error</div><div class="tele-val" id="tErr_d">—</div></div>
    <div class="tele-item esf"><div class="tele-lbl">Esfuerzo</div><div class="tele-val" id="tEsf_d">—</div></div>
  </div>
</div>

<div class="card">
  <div class="card-title"><i class="bi bi-speedometer2"></i>Telemetría (Izquierdo)</div>
  <div class="tele-grid">
    <div class="tele-item ref"><div class="tele-lbl">Referencia</div><div class="tele-val" id="tRef_i">0</div></div>
    <div class="tele-item pul"><div class="tele-lbl">Pulsos</div><div class="tele-val" id="tPul_i">—</div></div>
    <div class="tele-item err"><div class="tele-lbl">Error</div><div class="tele-val" id="tErr_i">—</div></div>
    <div class="tele-item esf"><div class="tele-lbl">Esfuerzo</div><div class="tele-val" id="tEsf_i">—</div></div>
  </div>
</div>

</div><!-- tele-cols -->
</div><!-- col-right -->

</main>

<div style="padding:20px 20px 40px;max-width:1200px;margin:0 auto;width:100%">
<div class="charts">

<div class="chart-card">
  <div class="ch-header">
    <i class="bi bi-graph-up-arrow" style="color:var(--accent);font-size:1.1rem"></i>
    <span class="ch-title">Referencia vs Medida (Derecho)</span>
  </div>
  <canvas id="cRef_d"></canvas>
</div>

<div class="chart-card">
  <div class="ch-header">
    <i class="bi bi-graph-up-arrow" style="color:var(--accent);font-size:1.1rem"></i>
    <span class="ch-title">Referencia vs Medida (Izquierdo)</span>
  </div>
  <canvas id="cRef_i"></canvas>
</div>

<div class="chart-card">
  <div class="ch-header">
    <i class="bi bi-activity" style="color:var(--green);font-size:1.1rem"></i>
    <span class="ch-title">Error</span>
  </div>
  <canvas id="cErr"></canvas>
</div>

<div class="chart-card">
  <div class="ch-header">
    <i class="bi bi-lightning-fill" style="color:var(--orange);font-size:1.1rem"></i>
    <span class="ch-title">Esfuerzo (DAC)</span>
  </div>
  <canvas id="cEsf"></canvas>
</div>

</div>
</div>

<!-- =====================================================
     MODAL ADMIN
     ===================================================== -->
<div class="modal-overlay" id="adminModal">
  <div class="modal">
    <div class="modal-header">
      <div class="modal-title">
        <i class="bi bi-sliders"></i> Panel de Administrador
      </div>
      <button class="modal-close" onclick="cerrarAdmin()">×</button>
    </div>

    <div class="param-group">

      <div class="param-row">
        <div>
          <div class="param-label">Pulsos Max</div>
          <div class="param-sub">Escala de referencia (PULSOS_MAX)</div>
        </div>
        <input class="param-input" id="p_pulsos_max" type="number" step="1" min="1" value="22">
        <button class="param-send" onclick="enviarParam('PULSOS_MAX', 'p_pulsos_max', this)">Aplicar</button>
      </div>

      <div class="param-row">
        <div>
          <div class="param-label">VMAX (m/s)</div>
          <div class="param-sub">Velocidad lineal máxima</div>
        </div>
        <input class="param-input" id="p_vmax" type="number" step="0.1" min="0.1" value="4.0">
        <button class="param-send" onclick="enviarParam('VMAX', 'p_vmax', this)">Aplicar</button>
      </div>

      <div class="param-row">
        <div>
          <div class="param-label">WMAX (rad/s)</div>
          <div class="param-sub">Velocidad angular máxima</div>
        </div>
        <input class="param-input" id="p_wmax" type="number" step="0.1" min="0.1" value="2.0">
        <button class="param-send" onclick="enviarParam('WMAX', 'p_wmax', this)">Aplicar</button>
      </div>

      <div class="param-row">
        <div>
          <div class="param-label">Kp</div>
          <div class="param-sub">Ganancia proporcional</div>
        </div>
        <input class="param-input" id="p_kp" type="number" step="0.5" min="0" value="6.0">
        <button class="param-send" onclick="enviarParam('Kp', 'p_kp', this)">Aplicar</button>
      </div>

      <div class="param-row">
        <div>
          <div class="param-label">Ki</div>
          <div class="param-sub">Ganancia integral (actualiza anti-windup)</div>
        </div>
        <input class="param-input" id="p_ki" type="number" step="0.5" min="0" value="3.0">
        <button class="param-send" onclick="enviarParam('Ki', 'p_ki', this)">Aplicar</button>
      </div>

      <div class="param-row">
        <div>
          <div class="param-label">Ref Min Giro</div>
          <div class="param-sub">Referencia mínima en giros (pulsos)</div>
        </div>
        <input class="param-input" id="p_ref_min_gir" type="number" step="1" min="0" value="10">
        <button class="param-send" onclick="enviarParam('ref_min_gir', 'p_ref_min_gir', this)">Aplicar</button>
      </div>

      <div class="param-row">
        <div>
          <div class="param-label">DAC Min Arranque</div>
          <div class="param-sub">Feedforward zona muerta (0–255)</div>
        </div>
        <input class="param-input" id="p_dac_min" type="number" step="1" min="0" max="255" value="60">
        <button class="param-send" onclick="enviarParam('dac_min_arranque', 'p_dac_min', this)">Aplicar</button>
      </div>

    </div>

    <div class="modal-footer">
      <button class="btn-close-modal" onclick="cerrarAdmin()">Cerrar</button>
    </div>
  </div>
</div>

<script>
// =====================================================
// WEBSOCKET
// =====================================================

let ws = null;
let wsReconnectTimer = null;
let motoresActivos = false;

function setWsUI(conectado) {
  document.getElementById('btn-conectar').style.display    = conectado ? 'none'  : 'block';
  document.getElementById('btn-desconectar').style.display = conectado ? 'block' : 'none';
  const badge = document.getElementById('badge');
  badge.classList.toggle('connected', conectado);
  document.getElementById('wsStatus').textContent = conectado ? 'WiFi Conectado' : 'WiFi Desconectado';
}

function connectWS() {
  if (ws && ws.readyState === WebSocket.OPEN) return;

  ws = new WebSocket('ws://192.168.4.1/ws');

  ws.onopen = () => {
    setWsUI(true);
    clearTimeout(wsReconnectTimer);
  };

  ws.onclose = () => {
    setWsUI(false);
    ws = null;
    if (motoresActivos) {
      wsReconnectTimer = setTimeout(connectWS, 3000);
    }
  };

  ws.onerror = (e) => {
    console.error('WS error', e);
  };

  ws.onmessage = (evt) => {
    parseMensaje(evt.data);
  };
}

function disconnectWS() {
  clearTimeout(wsReconnectTimer);
  if (ws) {
    sendMsg('ENABLE,false');
    ws.close();
    ws = null;
  }
  setWsUI(false);
  const badge = document.getElementById('estado-badge');
  badge.className   = 'estado-badge estado-detenido';
  badge.textContent = '⛔ MOTOR DETENIDO';
  motoresActivos = false;
}

function sendMsg(txt) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(txt + '\n');
  }
}

// =====================================================
// PARSER MENSAJES
// =====================================================

function parseMensaje(data) {
  const lines = data.split('\n');
  lines.forEach(line => {
    line = line.trim();

    // Confirmación de parámetro
    if (line.startsWith('PARAM_OK,')) {
      console.log('[Admin]', line);
      return;
    }

    if (!line.startsWith('TEL,')) return;
    const p = line.split(',');
    if (p.length < 9) return;

    const safeNum = v => { const n = Number(v); return isFinite(n) ? n : 0; };

    const pi   = safeNum(p[1]);
    const pd   = safeNum(p[2]);
    const ri   = safeNum(p[3]);
    const rd   = safeNum(p[4]);
    const ei   = safeNum(p[5]);
    const ed   = safeNum(p[6]);
    const daci = safeNum(p[7]);
    const dacd = safeNum(p[8]);

    document.getElementById('tPul_i').textContent = pi;
    document.getElementById('tPul_d').textContent = pd;
    document.getElementById('tRef_i').textContent = ri;
    document.getElementById('tRef_d').textContent = rd;
    document.getElementById('tErr_i').textContent = ei.toFixed(1);
    document.getElementById('tErr_d').textContent = ed.toFixed(1);
    document.getElementById('tEsf_i').textContent = daci;
    document.getElementById('tEsf_d').textContent = dacd;

    push(cRef_d, rd, pd);
    push(cRef_i, ri, pi);
    push(cErr,  ed, ei);
    push(cEsf, dacd, daci);
  });
}

// =====================================================
// CONTROL DIRECCIÓN
// =====================================================

let dpadState = { up:false, down:false, left:false, right:false };
let dpadTimer = null;

function updateSpeed(val) {
  document.getElementById('speedDisplay').textContent = val;
  enviarMovimiento();
}

function setEnable(on) {
  motoresActivos = on;
  sendMsg('ENABLE,' + (on ? 'true' : 'false'));

  const badge = document.getElementById('estado-badge');
  if (on) {
    badge.className   = 'estado-badge estado-activo';
    badge.textContent = '✅ MOTOR ACTIVO';
  } else {
    badge.className   = 'estado-badge estado-detenido';
    badge.textContent = '⛔ MOTOR DETENIDO';
    parar();
  }
}

function enviarMovimiento() {
  if (!motoresActivos) return;
  const spd = parseInt(document.getElementById('speedSlider').value) / 22;
  let vx = 0, vy = 0;
  if (dpadState.up)    vx =  spd;
  if (dpadState.down)  vx = -spd;
  if (dpadState.left)  vy = -spd;
  if (dpadState.right) vy =  spd;
  sendMsg(`MOVE,${vx.toFixed(4)},${vy.toFixed(4)}`);
}

function parar() {
  dpadState = { up:false, down:false, left:false, right:false };
  document.querySelectorAll('.dir-btn').forEach(btn => btn.classList.remove('pressed'));
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

const keyMap = { ArrowUp:'up', ArrowDown:'down', ArrowLeft:'left', ArrowRight:'right' };

document.addEventListener('keydown', e => {
  if (keyMap[e.key] && !dpadState[keyMap[e.key]]) { e.preventDefault(); dpad(keyMap[e.key], true); }
  if (e.key === ' ') { e.preventDefault(); parar(); }
});
document.addEventListener('keyup', e => {
  if (keyMap[e.key]) { e.preventDefault(); dpad(keyMap[e.key], false); }
});

// =====================================================
// PANEL ADMIN
// =====================================================

function abrirAdmin() {
  document.getElementById('adminModal').classList.add('open');
}

function cerrarAdmin() {
  document.getElementById('adminModal').classList.remove('open');
}

// Cerrar al hacer click fuera del modal — se registra tras carga del DOM
window.addEventListener('load', function() {
  var overlay = document.getElementById('adminModal');
  if (overlay) {
    overlay.addEventListener('click', function(e) {
      if (e.target === this) cerrarAdmin();
    });
  }
});

function enviarParam(nombre, inputId, btn) {
  const val = parseFloat(document.getElementById(inputId).value);
  if (isNaN(val)) {
    flashBtn(btn, 'err');
    return;
  }
  if (!ws || ws.readyState !== WebSocket.OPEN) {
    flashBtn(btn, 'err');
    alert('No hay conexión WebSocket activa.');
    return;
  }
  sendMsg(`PARAM,${nombre},${val}`);
  flashBtn(btn, 'ok');
}

function flashBtn(btn, cls) {
  btn.classList.add(cls);
  setTimeout(() => btn.classList.remove(cls), 1200);
}

// =====================================================
// GRÁFICAS
// =====================================================

const BUFFER = 150;
Chart.defaults.color = '#8b949e';
Chart.defaults.borderColor = '#21262d';

function mkChart(id, datasets) {
  // Arrancamos con arrays vacíos — Chart.js 4.x no dibuja nada con null[]
  datasets.forEach(ds => { ds.data = []; });
  return new Chart(document.getElementById(id), {
    type: 'line',
    data: { labels: [], datasets },
    options: {
      animation: false, responsive: true, maintainAspectRatio: true,
      interaction: { mode: 'index', intersect: false },
      plugins: { legend: { position: 'top', labels: { boxWidth: 10, padding: 12, usePointStyle: true, font: { size: 11 } } } },
      scales: {
        x: { display: false },
        y: { grid: { color: '#21262d' }, ticks: { color: '#8b949e', font: { size: 10 } } }
      },
      elements: { point: { radius: 0 }, line: { borderWidth: 1.5, tension: 0.3, spanGaps: true } }
    }
  });
}

const cRef_d = mkChart('cRef_d',[
  { label:'Referencia', borderColor:'#a371f7', backgroundColor:'rgba(163,113,247,.07)', fill:true, borderDash:[6,4], borderWidth:1 },
  { label:'Medida',     borderColor:'#f0883e', backgroundColor:'rgba(240,136,62,.07)',  fill:true }
]);
const cRef_i = mkChart('cRef_i',[
  { label:'Referencia', borderColor:'#a371f7', backgroundColor:'rgba(163,113,247,.07)', fill:true, borderDash:[6,4], borderWidth:1 },
  { label:'Medida',     borderColor:'#79c0ff', backgroundColor:'rgba(121,192,255,.07)', fill:true }
]);
const cErr = mkChart('cErr',[
  { label:'Derecho',   borderColor:'#f0883e', backgroundColor:'rgba(240,136,62,.07)',  fill:true },
  { label:'Izquierdo', borderColor:'#79c0ff', backgroundColor:'rgba(121,192,255,.07)', fill:true }
]);
const cEsf = mkChart('cEsf',[
  { label:'Derecho',   borderColor:'#f0883e', backgroundColor:'rgba(240,136,62,.07)',  fill:true },
  { label:'Izquierdo', borderColor:'#79c0ff', backgroundColor:'rgba(121,192,255,.07)', fill:true }
]);

function push(chart, ...vals) {
  vals.forEach((v, i) => {
    chart.data.datasets[i].data.push(v);
    if (chart.data.datasets[i].data.length > BUFFER) chart.data.datasets[i].data.shift();
  });
  chart.data.labels.push('');
  if (chart.data.labels.length > BUFFER) chart.data.labels.shift();
  chart.update('none');
}
</script>
</body>
</html>
)rawliteral";
