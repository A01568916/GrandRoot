"""
serial_bridge.py — GrandRoot
Lee el JSON del ESP32 por Serial USB y lo sirve al dashboard via Socket.IO.

USO:
    pip install flask flask-socketio pyserial

    python serial_bridge.py                        # autodetecta puerto
    python serial_bridge.py --port COM3            # Windows
    python serial_bridge.py --port /dev/ttyUSB0   # Linux
    python serial_bridge.py --port /dev/cu.usbserial-0001  # Mac

Luego abre en tu navegador: http://localhost:5000
"""

import sys
import os
import json
import threading
import time
import argparse
import glob

import serial
from flask import Flask, send_file, send_from_directory
from flask_socketio import SocketIO

# ═══════════════════════════════════════════════════════════════
# CONFIGURACION
# ═══════════════════════════════════════════════════════════════

BAUDRATE   = 115200
HOST       = "0.0.0.0"
PORT_HTTP  = 5000

# Nombre del dashboard HTML — debe estar en la misma carpeta que este script
DASHBOARD_FILE = "grandroot_dashboard.html"

# ═══════════════════════════════════════════════════════════════
# AUTODETECTAR PUERTO SERIAL
# ═══════════════════════════════════════════════════════════════

def autodetectar_puerto():
    candidatos = (
        glob.glob("/dev/ttyUSB*") +
        glob.glob("/dev/ttyACM*") +
        glob.glob("/dev/cu.usbserial*") +
        glob.glob("/dev/cu.SLAB*") +
        glob.glob("/dev/cu.wchusbserial*") +
        (["COM" + str(i) for i in range(3, 20)] if sys.platform == "win32" else [])
    )
    for p in candidatos:
        try:
            s = serial.Serial(p, BAUDRATE, timeout=1)
            s.close()
            print(f"[SERIAL] Puerto autodetectado: {p}")
            return p
        except (serial.SerialException, OSError):
            continue
    return None

# ═══════════════════════════════════════════════════════════════
# APP
# ═══════════════════════════════════════════════════════════════

app = Flask(__name__)
app.config["SECRET_KEY"] = "grandroot"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

ultimo_dato = {}        # ultimo paquete recibido del ESP32
clientes_conectados = 0

# ═══════════════════════════════════════════════════════════════
# RUTAS HTTP
# ═══════════════════════════════════════════════════════════════

@app.route("/")
def index():
    ruta = os.path.join(os.path.dirname(os.path.abspath(__file__)), DASHBOARD_FILE)
    if os.path.exists(ruta):
        return send_file(ruta)
    return f"<h2>No se encontró {DASHBOARD_FILE} en {os.path.dirname(ruta)}</h2>", 404

# ═══════════════════════════════════════════════════════════════
# EVENTOS SOCKET.IO
# ═══════════════════════════════════════════════════════════════

@socketio.on("connect")
def on_connect():
    global clientes_conectados
    clientes_conectados += 1
    print(f"[WS] Cliente conectado ({clientes_conectados} total)")
    # Mandar el ultimo dato inmediatamente para que el dashboard no espere
    if ultimo_dato:
        socketio.emit("telemetry", ultimo_dato)

@socketio.on("disconnect")
def on_disconnect():
    global clientes_conectados
    clientes_conectados = max(0, clientes_conectados - 1)
    print(f"[WS] Cliente desconectado ({clientes_conectados} total)")

@socketio.on("set_mode")
def on_set_mode(data):
    print(f"[WS] Modo recibido: {data}")

@socketio.on("manual_cmd")
def on_manual_cmd(data):
    print(f"[WS] Comando manual: {data}")

@socketio.on("set_param")
def on_set_param(data):
    print(f"[WS] Parámetro: {data}")

# ═══════════════════════════════════════════════════════════════
# HILO LECTOR DE SERIAL
# ═══════════════════════════════════════════════════════════════

def leer_serial(puerto):
    global ultimo_dato
    ser = None
    reconectar = True

    while reconectar:
        try:
            print(f"[SERIAL] Abriendo {puerto} a {BAUDRATE} baud...")
            ser = serial.Serial(puerto, BAUDRATE, timeout=2)
            print(f"[SERIAL] Conectado. Esperando datos del ESP32...\n")

            while True:
                try:
                    linea = ser.readline().decode("utf-8", errors="replace").strip()
                except UnicodeDecodeError:
                    continue

                if not linea:
                    continue

                # Imprimir en consola para debug
                #print(f"[ESP32] {linea}")

                # Ignorar las líneas de arranque que no son JSON
                if not linea.startswith("{"):
                    continue

                try:
                    datos = json.loads(linea)
                except json.JSONDecodeError as e:
                    print(f"[SERIAL][WARN] JSON inválido: {e}")
                    continue

                # Guardar y emitir a todos los clientes conectados
                ultimo_dato = datos
                if clientes_conectados > 0:
                    socketio.emit("telemetry", datos)

        except serial.SerialException as e:
            print(f"[SERIAL][ERROR] {e}")
            if ser:
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
            print("[SERIAL] Reintentando en 3 segundos...")
            time.sleep(3)

        except KeyboardInterrupt:
            print("\n[SERIAL] Interrumpido por usuario.")
            reconectar = False
            break

    if ser:
        ser.close()

# ═══════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="GrandRoot Serial Bridge")
    parser.add_argument("--port", type=str, default=None,
                        help="Puerto serial del ESP32 (ej: COM3, /dev/ttyUSB0)")
    args = parser.parse_args()

    puerto = args.port or autodetectar_puerto()

    if not puerto:
        print("[ERROR] No se encontró ningún puerto serial.")
        print("        Especifica uno con:  python serial_bridge.py --port /dev/ttyUSB0")
        sys.exit(1)

    # Iniciar hilo lector de serial
    hilo = threading.Thread(target=leer_serial, args=(puerto,), daemon=True)
    hilo.start()

    print(f"\n{'═'*50}")
    print(f"  GrandRoot Serial Bridge")
    print(f"  Puerto:    {puerto}  @  {BAUDRATE} baud")
    print(f"  Dashboard: http://localhost:{PORT_HTTP}")
    print(f"{'═'*50}\n")

    socketio.run(app, host=HOST, port=PORT_HTTP, debug=False, use_reloader=False)
