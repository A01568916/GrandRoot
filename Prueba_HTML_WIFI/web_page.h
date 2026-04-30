#pragma once
#include <pgmspace.h>

// HTML de la interfaz de control remoto, embebido en Flash del ESP32.
// Se define aquí (en .h) para evitar que el preprocesador de Arduino IDE
// rompa el raw string literal R"HTML(...)HTML".

const char HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Control Remoto — Planta Llanta</title>
<link href="https://cdn.jsdelivr.net/npm/bootstrap-icons@1.11.3/font/bootstrap-icons.min.css" rel="stylesheet">
<style>
:root{--bg:#0d1117;--surface:#161b22;--surface2:#1c2128;--border:#30363d;--accent:#58a6ff;--green:#3fb950;--red:#f85149;--text:#e6edf3;--muted:#8b949e}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font-family:'Segoe UI',system-ui,sans-serif;min-height:100vh;display:flex;flex-direction:column}
header{background:var(--surface);border-bottom:1px solid var(--border);padding:12px 20px;display:flex;align-items:center;justify-content:space-between}
.brand{display:flex;align-items:center;gap:10px}
.brand-icon{font-size:22px;color:var(--accent)}
.brand-title{font-size:16px;font-weight:700}
.brand-sub{font-size:11px;color:var(--muted)}
#badge{font-size:12px;font-weight:600;padding:4px 12px;border-radius:20px;background:#21262d;color:var(--green);border:1px solid var(--border);display:flex;align-items:center;gap:6px}
#badge::before{content:'';width:8px;height:8px;border-radius:50%;background:currentColor;display:inline-block}
main{flex:1;padding:20px;display:grid;grid-template-columns:280px 1fr;grid-template-rows:auto 1fr;gap:16px}
.panel-left{grid-row:1/3;display:flex;flex-direction:column;gap:12px}
.card{background:var(--surface);border:1px solid var(--border);border-radius:10px;padding:14px}
.card-title{font-size:10px;font-weight:700;letter-spacing:.08em;color:var(--muted);text-transform:uppercase;margin-bottom:10px;display:flex;align-items:center;gap:6px}
.tele-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.tele-item{background:var(--surface2);border-radius:8px;padding:8px 10px;border-left:3px solid var(--border)}
.tele-item.izq{border-left-color:var(--accent)}
.tele-item.der{border-left-color:#f0883e}
.tele-lbl{font-size:10px;color:var(--muted);margin-bottom:2px}
.tele-val{font-size:18px;font-weight:700}
.refs-row{display:flex;gap:8px}
.ref-box{flex:1;background:var(--surface2);border-radius:8px;padding:8px 10px;text-align:center}
.ref-box .lbl{font-size:10px;color:var(--muted)}
.ref-box .val{font-size:20px;font-weight:700}
.ref-box.izq .val{color:var(--accent)}
.ref-box.der .val{color:#f0883e}
.joystick-wrap{display:flex;flex-direction:column;align-items:center;gap:16px}
#joystick-canvas{cursor:crosshair;touch-action:none;border-radius:50%}
.dir-pad{display:grid;grid-template-columns:56px 56px 56px;grid-template-rows:56px 56px 56px;gap:6px}
.dir-btn{background:var(--surface2);border:1px solid var(--border);border-radius:8px;color:var(--text);font-size:20px;cursor:pointer;display:flex;align-items:center;justify-content:center;transition:.1s;user-select:none}
.dir-btn:active,.dir-btn.pressed{background:var(--accent);color:#0d1117}
.speed-row{display:flex;align-items:center;gap:10px;width:100%}
.speed-row label{font-size:12px;color:var(--muted);white-space:nowrap}
input[type=range]{flex:1;accent-color:var(--accent)}
#speedVal{font-size:13px;font-weight:700;min-width:30px;text-align:right}
.btn-enable{width:100%;padding:10px;border:none;border-radius:8px;font-size:13px;font-weight:700;cursor:pointer;transition:.15s;letter-spacing:.04em}
.btn-activar{background:#238636;color:#fff}.btn-activar:hover{background:#2ea043}
.btn-paro{background:#da3633;color:#fff}.btn-paro:hover{background:#f85149}
.estado-badge{text-align:center;padding:7px 10px;border-radius:6px;font-size:12px;font-weight:700;margin-bottom:10px}
.estado-detenido{background:#3d1f1e;color:var(--red);border:1px solid #da3633}
.estado-activo{background:#1a2f1a;color:var(--green);border:1px solid #238636}
</style>
</head>
<body>
<header>
  <div class="brand">
    <i class="bi bi-joystick brand-icon"></i>
    <div><div class="brand-title">Control Remoto</div><div class="brand-sub">Planta Llanta — ESP32</div></div>
  </div>
  <div id="badge"><span>GrandRoot &bull; 192.168.4.1</span></div>
</header>
<main>
  <div class="panel-left">
    <div class="card">
      <div class="card-title"><i class="bi bi-power"></i> Motores</div>
      <div id="estado-badge" class="estado-badge estado-detenido">⛔ MOTORES DETENIDOS</div>
      <div style="display:flex;gap:8px">
        <button class="btn-enable btn-activar" onclick="setEnable(true)"><i class="bi bi-play-fill"></i> ACTIVAR</button>
        <button class="btn-enable btn-paro"    onclick="setEnable(false)"><i class="bi bi-stop-fill"></i> PARO</button>
      </div>
    </div>
    <div class="card">
      <div class="card-title"><i class="bi bi-arrow-left-right"></i> Referencias (cinemática)</div>
      <div class="refs-row">
        <div class="ref-box izq"><div class="lbl">Izquierda</div><div class="val" id="rIzq">0</div></div>
        <div class="ref-box der"><div class="lbl">Derecha</div><div class="val" id="rDer">0</div></div>
      </div>
    </div>
    <div class="card">
      <div class="card-title"><i class="bi bi-speedometer2"></i> Telemetría (pulsos / 1000 ms)</div>
      <div class="tele-grid">
        <div class="tele-item izq"><div class="tele-lbl">Pulsos Izq</div><div class="tele-val" id="vPI">—</div></div>
        <div class="tele-item der"><div class="tele-lbl">Pulsos Der</div><div class="tele-val" id="vPD">—</div></div>
        <div class="tele-item izq"><div class="tele-lbl">DAC Izq</div><div class="tele-val" id="vDI">—</div></div>
        <div class="tele-item der"><div class="tele-lbl">DAC Der</div><div class="tele-val" id="vDD">—</div></div>
      </div>
    </div>
  </div>
  <div class="card" style="display:flex;flex-direction:column;align-items:center;justify-content:center;gap:24px">
    <div class="card-title" style="margin:0"><i class="bi bi-joystick"></i> Joystick</div>
    <div class="joystick-wrap">
      <canvas id="joystick-canvas" width="240" height="240"></canvas>
      <div class="speed-row" style="max-width:260px">
        <label>Vel. máx.</label>
        <input type="range" id="speedSlider" min="10" max="98" value="60" oninput="document.getElementById('speedVal').textContent=this.value">
        <span id="speedVal">60</span>
      </div>
    </div>
    <div>
      <div class="card-title" style="justify-content:center;margin-bottom:10px"><i class="bi bi-arrows-move"></i> D-Pad</div>
      <div class="dir-pad">
        <div></div>
        <div class="dir-btn" id="btn-up"    onmousedown="dpad('up',true)"    onmouseup="dpad('up',false)"    ontouchstart="dpad('up',true)"    ontouchend="dpad('up',false)"><i class="bi bi-arrow-up"></i></div>
        <div></div>
        <div class="dir-btn" id="btn-left"  onmousedown="dpad('left',true)"  onmouseup="dpad('left',false)"  ontouchstart="dpad('left',true)"  ontouchend="dpad('left',false)"><i class="bi bi-arrow-left"></i></div>
        <div class="dir-btn" id="btn-stop"  onclick="parar()"><i class="bi bi-stop-fill"></i></div>
        <div class="dir-btn" id="btn-right" onmousedown="dpad('right',true)" onmouseup="dpad('right',false)" ontouchstart="dpad('right',true)" ontouchend="dpad('right',false)"><i class="bi bi-arrow-right"></i></div>
        <div></div>
        <div class="dir-btn" id="btn-down"  onmousedown="dpad('down',true)"  onmouseup="dpad('down',false)"  ontouchstart="dpad('down',true)"  ontouchend="dpad('down',false)"><i class="bi bi-arrow-down"></i></div>
        <div></div>
      </div>
    </div>
  </div>
</main>
<script>
let sendTimer = null;
let motoresActivos = false;

async function setEnable(on) {
  const r = await fetch('/api/enable', {
    method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({on})
  });
  const j = await r.json();
  motoresActivos = j.enabled;
  const badge = document.getElementById('estado-badge');
  badge.className = 'estado-badge ' + (motoresActivos ? 'estado-activo' : 'estado-detenido');
  badge.textContent = motoresActivos ? '\u2705 MOTORES ACTIVOS' : '\u26d4 MOTORES DETENIDOS';
  if (!motoresActivos) { vxActual=0; vyActual=0; resetJoystick(); }
}

async function enviarMovimiento(vx, vy) {
  if (!motoresActivos) return;
  const r = await fetch('/api/mover', {
    method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({ vx, vy })
  });
  const j = await r.json();
  if (j.ok) {
    document.getElementById('rIzq').textContent = j.ref_izq;
    document.getElementById('rDer').textContent = j.ref_der;
  }
}
function parar() { vxActual=0; vyActual=0; enviarMovimiento(0,0); resetJoystick(); }
setInterval(() => {
  fetch('/api/tele').then(r => r.json()).then(d => {
    document.getElementById('vPI').textContent = d.pul_izq;
    document.getElementById('vDI').textContent = d.dac_izq;
    document.getElementById('vPD').textContent = d.pul_der;
    document.getElementById('vDD').textContent = d.dac_der;
  }).catch(() => {});
}, 1000);
const canvas = document.getElementById('joystick-canvas');
const ctx = canvas.getContext('2d');
const CX=canvas.width/2, CY=canvas.height/2, R_BASE=110, R_STICK=28;
let stickX=0, stickY=0, dragging=false, vxActual=0, vyActual=0;
function drawJoystick() {
  ctx.clearRect(0,0,canvas.width,canvas.height);
  ctx.beginPath(); ctx.arc(CX,CY,R_BASE,0,Math.PI*2); ctx.fillStyle='#1c2128'; ctx.fill();
  ctx.strokeStyle='#30363d'; ctx.lineWidth=2; ctx.stroke();
  ctx.lineWidth=1;
  ctx.beginPath(); ctx.moveTo(CX-R_BASE,CY); ctx.lineTo(CX+R_BASE,CY); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(CX,CY-R_BASE); ctx.lineTo(CX,CY+R_BASE); ctx.stroke();
  const sx=CX+stickX, sy=CY+stickY;
  ctx.beginPath(); ctx.arc(sx,sy,R_STICK,0,Math.PI*2);
  const g=ctx.createRadialGradient(sx-4,sy-4,2,sx,sy,R_STICK);
  g.addColorStop(0,'#58a6ff'); g.addColorStop(1,'#2E5F7A');
  ctx.fillStyle=g; ctx.fill(); ctx.strokeStyle='#79c0ff'; ctx.lineWidth=2; ctx.stroke();
}
function clampStick(x,y) {
  const d=Math.sqrt(x*x+y*y), lim=R_BASE-R_STICK;
  return d>lim ? [x*lim/d, y*lim/d] : [x,y];
}
function onMove(px,py) {
  const rect=canvas.getBoundingClientRect();
  [stickX,stickY]=clampStick(px-rect.left-CX, py-rect.top-CY);
  const m=R_BASE-R_STICK; vxActual=-stickY/m; vyActual=stickX/m;
  drawJoystick();
  clearTimeout(sendTimer); sendTimer=setTimeout(()=>enviarMovimiento(vxActual,vyActual),40);
}
function resetJoystick() { stickX=0; stickY=0; drawJoystick(); }
canvas.addEventListener('mousedown',  e=>{dragging=true; onMove(e.clientX,e.clientY)});
canvas.addEventListener('mousemove',  e=>{if(dragging) onMove(e.clientX,e.clientY)});
canvas.addEventListener('mouseup',    ()=>{dragging=false; parar()});
canvas.addEventListener('mouseleave', ()=>{if(dragging){dragging=false; parar()}});
canvas.addEventListener('touchstart', e=>{e.preventDefault(); dragging=true;  onMove(e.touches[0].clientX,e.touches[0].clientY)});
canvas.addEventListener('touchmove',  e=>{e.preventDefault(); if(dragging) onMove(e.touches[0].clientX,e.touches[0].clientY)});
canvas.addEventListener('touchend',   e=>{e.preventDefault(); dragging=false; parar()});
drawJoystick();
const dpadState={up:false,down:false,left:false,right:false};
let dpadTimer=null;
function dpad(dir,pressed) {
  dpadState[dir]=pressed;
  document.getElementById('btn-'+dir).classList.toggle('pressed',pressed);
  clearInterval(dpadTimer);
  if(Object.values(dpadState).some(Boolean)){dpadTick(); dpadTimer=setInterval(dpadTick,100);}
  else parar();
}
function dpadTick() {
  let vx=0, vy=0;
  const spd=parseInt(document.getElementById('speedSlider').value)/98;
  if(dpadState.up)    vx= spd;
  if(dpadState.down)  vx=-spd;
  if(dpadState.left)  vy=-spd;
  if(dpadState.right) vy= spd;
  enviarMovimiento(vx,vy);
}
const keyMap={ArrowUp:'up',ArrowDown:'down',ArrowLeft:'left',ArrowRight:'right'};
document.addEventListener('keydown', e=>{
  if(keyMap[e.key]&&!dpadState[keyMap[e.key]]){e.preventDefault(); dpad(keyMap[e.key],true);}
  if(e.key===' '){e.preventDefault(); parar();}
});
document.addEventListener('keyup', e=>{if(keyMap[e.key]){e.preventDefault(); dpad(keyMap[e.key],false);}});
</script>
</body>
</html>)HTML";
