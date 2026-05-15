# gui_control_web.py — Dashboard Web: Control Unificado (On/Off | P | PI | PID)
# Requiere: pip install flask pyserial
# Uso: python gui_control_web.py  →  abre http://localhost:5000 automáticamente

from flask import Flask, Response, jsonify, request, render_template_string
import serial
import serial.tools.list_ports
import threading
import queue
import json
import webbrowser
import time

app = Flask(__name__)

BAUD          = 115200
conexion      = None
conn_lock     = threading.Lock()
ref_valor     = 0.0
modo_valor    = 0          # 0=On/Off  1=P  2=PI  3=PID
modo_activo   = 0          # confirmado por el ESP32 (viene en la trama)
clientes      = []
clientes_lock = threading.Lock()

# ── Lectura serial ─────────────────────────────────────────────
def leer_serial():
    global conexion, modo_activo
    while True:
        with conn_lock:
            ser = conexion
        if ser is None or not ser.is_open:
            time.sleep(0.05)
            continue
        try:
            linea  = ser.readline().decode(errors='ignore').strip()
            partes = linea.split(',')
            if len(partes) == 4:          # nuevo formato con modo confirmado
                d = {
                    'pul_der': float(partes[0]),
                    'err_der': float(partes[1]),
                    'esf_der': int(float(partes[2])),
                    'modo':    int(float(partes[3])),
                }
                modo_activo = d['modo']
            elif len(partes) == 3:        # compatibilidad con firmware antiguo
                d = {
                    'pul_der': float(partes[0]),
                    'err_der': float(partes[1]),
                    'esf_der': int(float(partes[2])),
                    'modo':    modo_activo,
                }
            else:
                continue

            # El selector solo se habilita cuando el motor está detenido
            d['cambio_ok'] = (ref_valor == 0.0 and d['esf_der'] == 0)

            with clientes_lock:
                for q in list(clientes):
                    try:
                        q.put_nowait(d)
                    except queue.Full:
                        pass
        except Exception:
            pass

threading.Thread(target=leer_serial, daemon=True).start()

# ── API ────────────────────────────────────────────────────────
@app.route('/api/ports')
def api_ports():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    return jsonify(ports)

@app.route('/api/connect', methods=['POST'])
def api_connect():
    global conexion
    data     = request.get_json(silent=True) or {}
    conectar = bool(data.get('conectar'))
    puerto   = str(data.get('puerto', ''))
    with conn_lock:
        if conectar:
            try:
                if conexion and conexion.is_open:
                    conexion.close()
                conexion = serial.Serial(puerto, BAUD, timeout=1)
                return jsonify({'ok': True})
            except Exception as e:
                conexion = None
                return jsonify({'ok': False, 'error': str(e)}), 400
        else:
            if conexion and conexion.is_open:
                conexion.close()
            conexion = None
            return jsonify({'ok': True})

@app.route('/api/reset', methods=['POST'])
def api_reset():
    with conn_lock:
        if conexion and conexion.is_open:
            conexion.write(b"R\n")
    return jsonify({'ok': True})

@app.route('/api/send', methods=['POST'])
def api_send():
    global ref_valor, modo_valor
    data = request.get_json(silent=True) or {}
    try:
        ref_valor  = max(0.0, min(110.0, float(data.get('ref',  0))))
        modo_valor = max(0,   min(3,     int(  data.get('modo', 0))))
    except (ValueError, TypeError):
        return jsonify({'ok': False, 'error': 'valor inválido'}), 400

    with conn_lock:
        if conexion and conexion.is_open:
            # Protocolo: "ref,modo\n"
            conexion.write(f"{ref_valor},{modo_valor}\n".encode())
    return jsonify({'ok': True})

@app.route('/stream')
def stream():
    def generate():
        q = queue.Queue(maxsize=60)
        with clientes_lock:
            clientes.append(q)
        try:
            while True:
                try:
                    data = q.get(timeout=4)
                    yield f"data: {json.dumps(data)}\n\n"
                except queue.Empty:
                    yield ": keepalive\n\n"
        finally:
            with clientes_lock:
                if q in clientes:
                    clientes.remove(q)
    return Response(generate(), mimetype='text/event-stream',
                    headers={'Cache-Control': 'no-cache', 'X-Accel-Buffering': 'no'})

# ── HTML ───────────────────────────────────────────────────────
HTML = r"""<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Control Unificado — Motor Derecho</title>
<link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.3/dist/css/bootstrap.min.css" rel="stylesheet">
<link href="https://cdn.jsdelivr.net/npm/bootstrap-icons@1.11.3/font/bootstrap-icons.min.css" rel="stylesheet">
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.3/dist/chart.umd.min.js"></script>
<style>
:root{
  --bg:#0d1117; --surface:#161b22; --surface2:#1c2128;
  --border:#30363d; --accent:#58a6ff; --accent2:#79c0ff;
  --green:#3fb950; --green2:#56d364;
  --yellow:#d29922; --yellow2:#e3b341;
  --red:#f85149; --text:#e6edf3; --muted:#8b949e;
}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font-family:'Segoe UI',system-ui,sans-serif;min-height:100vh}

header{
  background:var(--surface);border-bottom:1px solid var(--border);
  padding:14px 28px;display:flex;align-items:center;gap:14px;
}
header .logo{font-size:1.6rem;color:var(--accent)}
header h1{font-size:1.15rem;font-weight:700}
header .sub{font-size:.78rem;color:var(--muted);margin-top:1px}
.badge-status{
  display:inline-flex;align-items:center;gap:7px;
  font-size:.78rem;font-weight:600;padding:5px 12px;
  border-radius:20px;margin-left:auto;
  background:rgba(248,81,73,.12);color:var(--red);border:1px solid rgba(248,81,73,.3);
  transition:all .3s;
}
.badge-status.on{background:rgba(63,185,80,.12);color:var(--green);border-color:rgba(63,185,80,.3)}
.dot{width:7px;height:7px;border-radius:50%;background:currentColor;animation:pulse 1.5s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.25}}

.ctrl{
  background:var(--surface);border-bottom:1px solid var(--border);
  padding:16px 28px;display:flex;align-items:flex-end;gap:20px;flex-wrap:wrap;
}
.ctrl-group{display:flex;flex-direction:column;gap:6px}
.ctrl-group label{font-size:.7rem;color:var(--muted);text-transform:uppercase;letter-spacing:.07em}
.ctrl-group select{
  background:var(--bg);border:1px solid var(--border);color:var(--text);
  border-radius:8px;padding:9px 12px;font-size:.9rem;outline:none;
  transition:border-color .2s,opacity .2s;
}
.ctrl-group select:focus{border-color:var(--accent);box-shadow:0 0 0 3px rgba(88,166,255,.18)}
.ctrl-group select:disabled{opacity:.45;cursor:not-allowed}

/* Modo selector — resalta modo activo confirmado */
.modo-badge{
  font-size:.65rem;font-weight:700;padding:2px 7px;border-radius:10px;
  background:rgba(88,166,255,.15);color:var(--accent);border:1px solid rgba(88,166,255,.3);
  margin-left:6px;vertical-align:middle;
}

.aviso-modo{
  font-size:.72rem;color:var(--yellow);margin-top:4px;
  display:none;
}
.aviso-modo.visible{display:block}

.divider-v{width:1px;background:var(--border);height:52px;align-self:flex-end}
.ref-input{
  background:var(--bg);border:2px solid var(--border);color:var(--text);
  border-radius:10px;padding:8px 14px;font-size:1.7rem;font-weight:700;
  text-align:center;width:120px;outline:none;transition:border-color .2s;
}
.ref-input:focus{border-color:var(--accent);box-shadow:0 0 0 3px rgba(88,166,255,.18)}

.btn-conn{
  background:var(--accent);color:#0d1117;border:none;border-radius:9px;
  padding:10px 22px;font-size:.9rem;font-weight:700;cursor:pointer;transition:.2s;
}
.btn-conn:hover{filter:brightness(1.12)}
.btn-conn:active{transform:scale(.96)}
.btn-conn.on{background:var(--red);color:#fff}
.btn-send{
  background:var(--green);color:#0d1117;border:none;border-radius:9px;
  padding:10px 28px;font-size:1.1rem;font-weight:700;cursor:pointer;transition:.2s;
  display:flex;align-items:center;gap:8px;
}
.btn-send:hover{filter:brightness(1.12)}
.btn-send:active{transform:scale(.96)}

.metrics{display:flex;gap:12px;padding:20px 28px 4px;flex-wrap:wrap}
.mcard{
  background:var(--surface);border:1px solid var(--border);border-radius:14px;
  padding:14px 20px;flex:1;min-width:130px;position:relative;overflow:hidden;
  transition:border-color .3s;
}
.mcard:hover{border-color:var(--accent)}
.mcard::before{
  content:'';position:absolute;top:0;left:0;right:0;height:3px;
  background:var(--card-accent,var(--accent));border-radius:14px 14px 0 0;
}
.mcard .lbl{font-size:.68rem;color:var(--muted);text-transform:uppercase;letter-spacing:.07em}
.mcard .val{font-size:2rem;font-weight:800;line-height:1.1;margin-top:4px;color:var(--card-accent,var(--text))}
.mcard .unit{font-size:.72rem;color:var(--muted);margin-top:3px}

.charts{display:flex;flex-direction:column;gap:14px;padding:16px 28px 28px}
.chart-card{
  background:var(--surface);border:1px solid var(--border);
  border-radius:14px;padding:18px 22px;
}
.chart-card .ch-header{display:flex;align-items:center;gap:9px;margin-bottom:14px}
.chart-card .ch-title{font-size:.82rem;font-weight:700;color:var(--muted);text-transform:uppercase;letter-spacing:.07em}
.chart-card canvas{max-height:165px}

.btn-reset{
  background:transparent;color:var(--red);border:1px solid rgba(248,81,73,.4);
  border-radius:8px;padding:7px 14px;font-size:.8rem;font-weight:600;
  cursor:pointer;transition:.2s;display:flex;align-items:center;gap:6px;
  margin-left:auto;align-self:flex-end;
}
.btn-reset:hover{background:rgba(248,81,73,.12);border-color:var(--red)}
.btn-reset:active{transform:scale(.95)}
.btn-reset:disabled{opacity:.35;cursor:not-allowed;pointer-events:none}
  position:fixed;bottom:22px;right:22px;
  background:var(--surface2);border:1px solid var(--border);color:var(--text);
  border-radius:10px;padding:12px 18px;font-size:.85rem;
  transform:translateY(80px);opacity:0;transition:.3s;pointer-events:none;z-index:99;
}
.toast-box.show{transform:translateY(0);opacity:1}
.toast-box.error{border-color:var(--red);color:var(--red)}
.toast-box.warn{border-color:var(--yellow);color:var(--yellow)}
</style>
</head>
<body>

<!-- Header -->
<header>
  <i class="bi bi-cpu-fill logo"></i>
  <div>
    <h1>Control Unificado <span id="modoLabel" class="modo-badge">On/Off</span></h1>
    <div class="sub">Motor Derecho — ESP32</div>
  </div>
  <span id="badge" class="badge-status">
    <span class="dot"></span>
    <span id="badgeTxt">Desconectado</span>
  </span>
</header>

<!-- Panel de control -->
<div class="ctrl">
  <!-- Puerto -->
  <div class="ctrl-group">
    <label><i class="bi bi-usb-symbol"></i> Puerto Serial</label>
    <select id="selPuerto"><option value="">— seleccionar —</option></select>
  </div>
  <button id="btnConn" class="btn-conn" onclick="toggleConexion()">
    <i class="bi bi-plug-fill"></i> Conectar
  </button>

  <div class="divider-v"></div>

  <!-- Selector de modo -->
  <div class="ctrl-group">
    <label><i class="bi bi-sliders"></i> Tipo de control</label>
    <select id="selModo" onchange="onModoChange()">
      <option value="0">On / Off</option>
      <option value="1">P — Proporcional</option>
      <option value="2">PI — Proporcional-Integral</option>
      <option value="3">PID — Proporcional-Integral-Derivativo</option>
    </select>
    <div id="avisoModo" class="aviso-modo">
      <i class="bi bi-exclamation-triangle-fill"></i>
      Lleva la referencia a 0 y espera que el motor se detenga para cambiar el modo.
    </div>
  </div>

  <div class="divider-v"></div>

  <!-- Referencia -->
  <div class="ctrl-group">
    <label><i class="bi bi-speedometer2"></i> Referencia (pulsos / 1000 ms)</label>
    <input id="inpRef" class="ref-input" type="number" value="0" min="0" max="110"
           onkeydown="if(event.key==='Enter') enviar()">
  </div>
  <button class="btn-send" onclick="enviar()">
    <i class="bi bi-send-fill"></i> Enviar
  </button>

  <button id="btnReset" class="btn-reset" onclick="resetMotor()" title="Forzar DAC a 0">
    <i class="bi bi-slash-circle"></i> Reset
  </button>
</div>

<!-- Métricas -->
<div class="metrics">
  <div class="mcard" style="--card-accent:var(--accent2)">
    <div class="lbl"><i class="bi bi-circle-half"></i> Pulsos Der</div>
    <div class="val" id="vPD">0</div>
    <div class="unit">por 1000 ms</div>
  </div>
  <div class="mcard" style="--card-accent:var(--green2)">
    <div class="lbl"><i class="bi bi-activity"></i> Error Der</div>
    <div class="val" id="vED">0</div>
    <div class="unit">% de referencia</div>
  </div>
  <div class="mcard" style="--card-accent:var(--yellow2)">
    <div class="lbl"><i class="bi bi-lightning-charge-fill"></i> DAC Der</div>
    <div class="val" id="vDD">0</div>
    <div class="unit">0 – 255</div>
  </div>
  <div class="mcard" style="--card-accent:var(--accent)">
    <div class="lbl"><i class="bi bi-gear-fill"></i> Modo Confirmado</div>
    <div class="val" id="vModo" style="font-size:1.2rem;padding-top:6px">On/Off</div>
    <div class="unit">enviado por ESP32</div>
  </div>
</div>

<!-- Gráficas -->
<div class="charts">
  <div class="chart-card">
    <div class="ch-header">
      <i class="bi bi-graph-up-arrow" style="color:var(--accent);font-size:1.1rem"></i>
      <span class="ch-title">Pulsos vs Referencia</span>
    </div>
    <canvas id="cRef"></canvas>
  </div>
  <div class="chart-card">
    <div class="ch-header">
      <i class="bi bi-activity" style="color:var(--green);font-size:1.1rem"></i>
      <span class="ch-title">Error (%)</span>
    </div>
    <canvas id="cErr"></canvas>
  </div>
  <div class="chart-card">
    <div class="ch-header">
      <i class="bi bi-lightning-fill" style="color:var(--yellow);font-size:1.1rem"></i>
      <span class="ch-title">Esfuerzo (DAC)</span>
    </div>
    <canvas id="cEsf"></canvas>
  </div>
</div>

<div id="toast" class="toast-box"></div>

<script>
const BUFFER   = 150;
const MODOS    = ['On/Off','P','PI','PID'];
const emptyArr = () => Array(BUFFER).fill(null);

Chart.defaults.color       = '#8b949e';
Chart.defaults.borderColor = '#21262d';

function mkChart(id, datasets) {
  return new Chart(document.getElementById(id), {
    type: 'line',
    data: { labels: emptyArr(), datasets },
    options: {
      animation: false, responsive: true, maintainAspectRatio: true,
      interaction: { mode: 'index', intersect: false },
      plugins: {
        legend: { position: 'top', labels: { boxWidth: 10, padding: 18, usePointStyle: true } },
        tooltip: {
          backgroundColor: '#1c2128', borderColor: '#30363d', borderWidth: 1,
          titleColor: '#8b949e', bodyColor: '#e6edf3',
        }
      },
      scales: {
        x: { display: false },
        y: { grid: { color: '#21262d' }, ticks: { color: '#8b949e' } }
      },
      elements: { point: { radius: 0 }, line: { borderWidth: 2, tension: 0.3 } }
    }
  });
}

const cRef = mkChart('cRef', [
  { label: 'Pulsos Der', data: emptyArr(), borderColor: '#58a6ff', backgroundColor: 'rgba(88,166,255,.07)', fill: true },
  { label: 'Referencia', data: emptyArr(), borderColor: '#f85149', borderDash: [6,4], backgroundColor: 'transparent', borderWidth: 1.5 },
]);
const cErr = mkChart('cErr', [
  { label: 'Error Der', data: emptyArr(), borderColor: '#3fb950', backgroundColor: 'rgba(63,185,80,.07)', fill: true },
]);
const cEsf = mkChart('cEsf', [
  { label: 'DAC Der', data: emptyArr(), borderColor: '#d29922', backgroundColor: 'rgba(210,153,34,.07)', fill: true },
]);

function push(chart, ...vals) {
  vals.forEach((v, i) => {
    chart.data.datasets[i].data.push(v);
    if (chart.data.datasets[i].data.length > BUFFER)
      chart.data.datasets[i].data.shift();
  });
  chart.data.labels.push('');
  if (chart.data.labels.length > BUFFER) chart.data.labels.shift();
  chart.update('none');
}

// ── Estado local ──────────────────────────────────────────────
let refActual    = 0;
let modoPendiente= 0;   // lo que el usuario eligió en el select
let modoConfirmado = 0; // lo que confirmó el ESP32
let cambioOk     = true; // el ESP dice si se puede cambiar

// ── SSE ───────────────────────────────────────────────────────
const sse = new EventSource('/stream');
sse.onmessage = e => {
  const d = JSON.parse(e.data);

  document.getElementById('vPD').textContent = d.pul_der;
  document.getElementById('vED').textContent = d.err_der;
  document.getElementById('vDD').textContent = d.esf_der;

  // Modo confirmado por el ESP
  if (d.modo !== undefined) {
    modoConfirmado = d.modo;
    document.getElementById('vModo').textContent = MODOS[d.modo] || '?';
    document.getElementById('modoLabel').textContent = MODOS[d.modo] || '?';
  }

  // Habilitar/deshabilitar selector según condición de seguridad
  cambioOk = d.cambio_ok;
  const sel = document.getElementById('selModo');
  const aviso = document.getElementById('avisoModo');
  if (cambioOk) {
    sel.disabled = false;
    aviso.classList.remove('visible');
  } else {
    sel.disabled = true;
    aviso.classList.add('visible');
  }

  push(cRef, d.pul_der, refActual);
  push(cErr, d.err_der);
  push(cEsf, d.esf_der);
};

// ── Cambio de modo en el selector ────────────────────────────
function onModoChange() {
  modoPendiente = parseInt(document.getElementById('selModo').value);
  if (!cambioOk) {
    toast('⚠ Baja la referencia a 0 y espera que el motor se detenga antes de cambiar el modo.', 'warn');
  }
}

// ── Puertos ───────────────────────────────────────────────────
async function cargarPuertos() {
  const r     = await fetch('/api/ports');
  const ports = await r.json();
  const sel   = document.getElementById('selPuerto');
  sel.innerHTML = '<option value="">— seleccionar —</option>' +
    ports.map(p => `<option value="${p}">${p}</option>`).join('');
}
cargarPuertos();

// ── Conexión ──────────────────────────────────────────────────
let conectado = false;
async function toggleConexion() {
  const puerto = document.getElementById('selPuerto').value;
  if (!conectado && !puerto) { toast('Selecciona un puerto primero', 'error'); return; }
  const r = await fetch('/api/connect', {
    method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({ puerto, conectar: !conectado })
  });
  const j = await r.json();
  if (!j.ok) { toast('Error: ' + (j.error || '?'), 'error'); return; }
  conectado = !conectado;
  const btn   = document.getElementById('btnConn');
  const badge = document.getElementById('badge');
  const btxt  = document.getElementById('badgeTxt');
  if (conectado) {
    btn.innerHTML = '<i class="bi bi-plug-fill"></i> Desconectar';
    btn.classList.add('on');
    badge.classList.add('on');
    btxt.textContent = 'Conectado · ' + puerto;
    toast('Conectado a ' + puerto);
  } else {
    btn.innerHTML = '<i class="bi bi-plug-fill"></i> Conectar';
    btn.classList.remove('on');
    badge.classList.remove('on');
    btxt.textContent = 'Desconectado';
    toast('Desconectado');
  }
}

// ── Reset forzado DAC → 0 ─────────────────────────────────────
async function resetMotor() {
  const r = await fetch('/api/reset', { method: 'POST' });
  const j = await r.json();
  if (j.ok) toast('⚡ DAC forzado a 0', 'warn');
}

function actualizarBtnReset() {
  document.getElementById('btnReset').disabled = (refActual > 0);
}

// ── Enviar referencia + modo ──────────────────────────────────
async function enviar() {
  refActual     = Math.max(0, Math.min(110, parseFloat(document.getElementById('inpRef').value) || 0));
  modoPendiente = parseInt(document.getElementById('selModo').value);
  document.getElementById('inpRef').value = refActual;
  actualizarBtnReset();

  // Si el usuario quiere cambiar modo y el motor no está listo, advertir pero igualmente enviar;
  // el ESP descartará el cambio de modo si no es seguro y lo confirmará en la próxima trama.
  const r = await fetch('/api/send', {
    method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({ ref: refActual, modo: modoPendiente })
  });
  const j = await r.json();
  if (j.ok) {
    toast('Enviado: ref=' + refActual + '  modo=' + MODOS[modoPendiente]);
  }
}

// ── Toast ─────────────────────────────────────────────────────
let toastTimer;
function toast(msg, tipo='ok') {
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.className   = 'toast-box show' + (tipo === 'error' ? ' error' : tipo === 'warn' ? ' warn' : '');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => t.classList.remove('show'), 3200);
}
</script>
</body>
</html>"""

@app.route('/')
def index():
    return render_template_string(HTML)

if __name__ == '__main__':
    webbrowser.open('http://127.0.0.1:5000')
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
