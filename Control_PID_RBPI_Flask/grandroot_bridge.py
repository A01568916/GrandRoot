#!/usr/bin/env python3
"""
grandroot_bridge.py
───────────────────
Puente Flask entre la laptop (vía hotspot de la Raspberry Pi 5) y el ESP32
(vía Serial). Sirve el panel de control HTML y traduce las peticiones web en
comandos seriales hacia el ESP32, además de exponer la telemetría que el
ESP32 emite continuamente.

Arquitectura:
    Laptop ──(hotspot WiFi)──> Raspberry Pi 5 [Flask] ──(Serial)──> ESP32

¿Por qué un hilo lector?
    Solo un proceso puede tener abierto el puerto serial. El ESP32 envía
    "TEL,..." cada 200 ms de forma continua. Un hilo en segundo plano lee
    esas líneas sin parar y guarda la última en memoria. Las rutas HTTP solo
    consultan ese valor (GET /telemetria) o escriben comandos (POST /...).
    Así el navegador hace polling ligero sin bloquear el serial.

Endpoints:
    GET  /            → sirve grandroot.html
    GET  /telemetria  → último TEL como JSON {pi,pd,ri,rd,ei,ed,daci,dacd}
    POST /move        → {vx, vy}            → escribe "MOVE,vx,vy"
    POST /enable      → {on: true|false}    → escribe "ENABLE,true|false"
    POST /param       → {nombre, valor}     → escribe "PARAM,nombre,valor"
    GET  /status      → estado de la conexión serial
"""

from flask import Flask, request, jsonify, send_file
import serial
import threading
import time
import os

# ── Configuración Serial ──────────────────────────────────────────
# Tu regla udev expone el ESP32 como /dev/ttyESP32 (symlink estable).
SERIAL_PORT = '/dev/ttyESP32'
BAUD_RATE   = 115200

# Ruta del HTML (mismo directorio que este script)
BASE_DIR  = os.path.dirname(os.path.abspath(__file__))
HTML_FILE = os.path.join(BASE_DIR, 'grandroot.html')

app = Flask(__name__)

# ── Estado compartido (protegido por lock) ────────────────────────
ser = None
serial_lock = threading.Lock()        # Serializa las escrituras al puerto

# Última telemetría recibida. Claves alineadas con el protocolo TEL,
# pi,pd,ri,rd,ei,ed,daci,dacd
_telemetria = {
    'pi': 0, 'pd': 0,       # pulsos (medida) izq/der
    'ri': 0, 'rd': 0,       # referencia izq/der
    'ei': 0.0, 'ed': 0.0,   # error izq/der
    'daci': 0, 'dacd': 0,   # esfuerzo (DAC) izq/der
    'ts': 0.0,              # timestamp de la última lectura
}
_tel_lock = threading.Lock()


# ── Apertura del puerto serial ────────────────────────────────────
def abrir_serial():
    """Intenta abrir el puerto. Devuelve el objeto Serial o None."""
    global ser
    try:
        s = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # El ESP32 se reinicia al abrir el puerto; esperarlo
        s.reset_input_buffer()
        print(f'[OK] ESP32 conectado en {SERIAL_PORT} @ {BAUD_RATE}')
        return s
    except Exception as e:
        print(f'[WARN] No se pudo abrir serial: {e}')
        return None


# ── Hilo lector: consume el serial sin parar ──────────────────────
def hilo_lector():
    """
    Lee líneas del ESP32 de forma continua. Si la línea es telemetría
    (TEL,...) actualiza el estado compartido. Reabre el puerto si se cae.
    Corre como daemon: muere automáticamente al terminar el proceso.
    """
    global ser
    while True:
        if ser is None:
            ser = abrir_serial()
            if ser is None:
                time.sleep(2)   # Reintentar conexión
                continue

        try:
            raw = ser.readline()
            if not raw:
                continue
            linea = raw.decode('utf-8', errors='ignore').strip()
            if linea.startswith('TEL,'):
                _parse_telemetria(linea)
            # Otras líneas (ENABLE,.. / PARAM_OK,..) se ignoran aquí;
            # son ecos de confirmación, no se necesitan para el panel.
        except (serial.SerialException, OSError) as e:
            print(f'[ERR] Serial caído: {e} — reintentando...')
            try:
                ser.close()
            except Exception:
                pass
            ser = None
            time.sleep(2)
        except Exception as e:
            print(f'[ERR] Lectura: {e}')


def _parse_telemetria(linea):
    """Convierte 'TEL,pi,pd,ri,rd,ei,ed,daci,dacd' en el dict compartido."""
    partes = linea.split(',')
    if len(partes) < 9:
        return
    try:
        nuevo = {
            'pi':   int(float(partes[1])),
            'pd':   int(float(partes[2])),
            'ri':   int(float(partes[3])),
            'rd':   int(float(partes[4])),
            'ei':   float(partes[5]),
            'ed':   float(partes[6]),
            'daci': int(float(partes[7])),
            'dacd': int(float(partes[8])),
            'ts':   time.time(),
        }
    except (ValueError, IndexError):
        return
    with _tel_lock:
        _telemetria.update(nuevo)


# ── Escritura segura al serial ────────────────────────────────────
def enviar_serial(comando):
    """Escribe 'comando\\n' al ESP32. Devuelve True si se envió."""
    if ser is None:
        return False
    try:
        with serial_lock:
            ser.write((comando + '\n').encode())
        return True
    except (serial.SerialException, OSError) as e:
        print(f'[ERR] Escritura serial: {e}')
        return False


# ── Rutas ─────────────────────────────────────────────────────────
@app.route('/')
def index():
    return send_file(HTML_FILE)


@app.route('/telemetria')
def telemetria():
    with _tel_lock:
        datos = dict(_telemetria)
    # ¿Hay datos frescos? (recibidos en el último segundo)
    datos['online'] = (ser is not None) and (time.time() - datos['ts'] < 1.0)
    return jsonify(datos)


@app.route('/move', methods=['POST'])
def move():
    data = request.get_json(silent=True) or {}
    try:
        vx = max(-1.0, min(1.0, float(data.get('vx', 0))))
        vy = max(-1.0, min(1.0, float(data.get('vy', 0))))
    except (ValueError, TypeError):
        return jsonify({'error': 'vx/vy inválidos'}), 400

    if not enviar_serial(f'MOVE,{vx:.4f},{vy:.4f}'):
        return jsonify({'error': 'ESP32 no conectado'}), 503
    return jsonify({'ok': True})


@app.route('/enable', methods=['POST'])
def enable():
    data = request.get_json(silent=True) or {}
    on = bool(data.get('on', False))
    if not enviar_serial('ENABLE,' + ('true' if on else 'false')):
        return jsonify({'error': 'ESP32 no conectado'}), 503
    return jsonify({'ok': True, 'on': on})


@app.route('/param', methods=['POST'])
def param():
    data   = request.get_json(silent=True) or {}
    nombre = str(data.get('nombre', '')).strip()
    if not nombre:
        return jsonify({'error': 'falta nombre'}), 400
    try:
        valor = float(data.get('valor'))
    except (ValueError, TypeError):
        return jsonify({'error': 'valor inválido'}), 400

    if not enviar_serial(f'PARAM,{nombre},{valor}'):
        return jsonify({'error': 'ESP32 no conectado'}), 503
    return jsonify({'ok': True, 'nombre': nombre, 'valor': valor})


@app.route('/status')
def status():
    return jsonify({
        'serial_port': SERIAL_PORT,
        'baud': BAUD_RATE,
        'conectado': ser is not None,
    })


# ── Arranque ──────────────────────────────────────────────────────
# El hilo lector se lanza al importar el módulo para que funcione tanto
# con `python grandroot_bridge.py` como bajo gunicorn/systemd.
_lector = threading.Thread(target=hilo_lector, daemon=True)
_lector.start()

if __name__ == '__main__':
    # threaded=True para que el polling de varios clientes no se bloquee.
    # use_reloader=False evita que se lance un segundo hilo lector que
    # pelearía por el puerto serial.
    app.run(host='0.0.0.0', port=8080, threaded=True, use_reloader=False)
