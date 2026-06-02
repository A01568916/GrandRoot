"""
GrandRoot — Raspberry Pi Flask Server
======================================
Arquitectura:
  PC (browser) ←→ Flask/SocketIO ←→ Serial USB ←→ ESP32 Motores (/dev/ttyESP32)
                                  ←→ Serial USB ←→ ESP32 Sensores (/dev/ttySensores)

Instalar dependencias:
  pip install flask flask-socketio pyserial eventlet

Ejecutar:
  python app.py
"""

import json
import threading
import time
import serial
import serial.tools.list_ports
from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO, emit

# ─── Configuración ────────────────────────────────────────────────────────────

SERIAL_MOTORES   = "/dev/ttyESP32"       # nombre del puerto del ESP32 de motores
SERIAL_SENSORES  = "/dev/ttySensores"    # nombre del puerto del ESP32 de sensores
SERIAL_BAUD      = 115200
SERIAL_TIMEOUT   = 0.1                  # segundos

# ─── App Flask + SocketIO ─────────────────────────────────────────────────────

app = Flask(__name__)
app.config["SECRET_KEY"] = "grandroot_secret"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="eventlet")

# ─── Estado global compartido ─────────────────────────────────────────────────

state = {
    "tel": {
        "medida_i": 0, "medida_d": 0,
        "ref_i": 0,    "ref_d": 0,
        "error_i": 0,  "error_d": 0,
        "dac_i": 0,    "dac_d": 0,
    },
    "sens": {
        "lat": 0, "lon": 0, "x": 0, "y": 0,
        "sats": 0,
        "ax": 0, "ay": 0, "az": 0,
        "gx": 0, "gy": 0, "gz": 0,
        "gps_ok": 0, "imu_ok": 0, "ts": 0,
    },
    "status": {
        "motores_conectado": False,
        "sensores_conectado": False,
    }
}
state_lock = threading.Lock()

# ─── Serial ports ─────────────────────────────────────────────────────────────

ser_motores  = None
ser_sensores = None
serial_lock_motores  = threading.Lock()
serial_lock_sensores = threading.Lock()


def abrir_puerto(path, baud=SERIAL_BAUD):
    """Intenta abrir un puerto serial. Retorna el objeto o None."""
    try:
        s = serial.Serial(path, baud, timeout=SERIAL_TIMEOUT)
        print(f"[Serial] Puerto abierto: {path}")
        return s
    except serial.SerialException as e:
        print(f"[Serial] No se pudo abrir {path}: {e}")
        return None


def listar_puertos():
    """Devuelve lista de puertos disponibles (útil para debug)."""
    return [p.device for p in serial.tools.list_ports.comports()]


# ─── Parsers de líneas seriales ────────────────────────────────────────────────

def parsear_telemetria(linea: str):
    """
    Formato esperado del ESP32 Motores:
      TEL,<medida_i>,<medida_d>,<ref_i>,<ref_d>,<error_i>,<error_d>,<dac_i>,<dac_d>
    """
    try:
        partes = linea.strip().split(",")
        if partes[0] != "TEL" or len(partes) < 9:
            return None
        return {
            "medida_i": int(partes[1]),
            "medida_d": int(partes[2]),
            "ref_i":    int(partes[3]),
            "ref_d":    int(partes[4]),
            "error_i":  float(partes[5]),
            "error_d":  float(partes[6]),
            "dac_i":    int(partes[7]),
            "dac_d":    int(partes[8]),
        }
    except (ValueError, IndexError):
        return None


def parsear_sensores(linea: str):
    """
    Formato esperado del ESP32 Sensores:
      SENS,{json}
    donde el JSON contiene lat, lon, x, y, sats, ax, ay, az, gx, gy, gz, gps_ok, imu_ok, ts
    """
    try:
        idx = linea.find(",")
        if idx == -1 or not linea.startswith("SENS"):
            return None
        return json.loads(linea[idx + 1:].strip())
    except (json.JSONDecodeError, ValueError):
        return None


# ─── Hilos lectores de serial ──────────────────────────────────────────────────

def leer_motores():
    """Hilo dedicado a leer el serial del ESP32 de motores."""
    global ser_motores
    while True:
        # Intentar reconectar si no hay puerto
        if ser_motores is None or not ser_motores.is_open:
            with serial_lock_motores:
                ser_motores = abrir_puerto(SERIAL_MOTORES)
            with state_lock:
                state["status"]["motores_conectado"] = ser_motores is not None
            socketio.emit("status", state["status"])
            if ser_motores is None:
                time.sleep(3)
                continue

        try:
            with serial_lock_motores:
                linea = ser_motores.readline().decode("utf-8", errors="replace")

            if not linea:
                continue

            datos = parsear_telemetria(linea)
            if datos:
                with state_lock:
                    state["tel"].update(datos)
                socketio.emit("tel", datos)

        except (serial.SerialException, OSError) as e:
            print(f"[Motores] Error serial: {e}")
            with serial_lock_motores:
                if ser_motores:
                    ser_motores.close()
                ser_motores = None
            with state_lock:
                state["status"]["motores_conectado"] = False
            socketio.emit("status", state["status"])
            time.sleep(2)


def leer_sensores():
    """Hilo dedicado a leer el serial del ESP32 de sensores."""
    global ser_sensores
    while True:
        if ser_sensores is None or not ser_sensores.is_open:
            with serial_lock_sensores:
                ser_sensores = abrir_puerto(SERIAL_SENSORES)
            with state_lock:
                state["status"]["sensores_conectado"] = ser_sensores is not None
            socketio.emit("status", state["status"])
            if ser_sensores is None:
                time.sleep(3)
                continue

        try:
            with serial_lock_sensores:
                linea = ser_sensores.readline().decode("utf-8", errors="replace")

            if not linea:
                continue

            datos = parsear_sensores(linea)
            if datos:
                with state_lock:
                    state["sens"].update(datos)
                socketio.emit("sens", datos)

        except (serial.SerialException, OSError) as e:
            print(f"[Sensores] Error serial: {e}")
            with serial_lock_sensores:
                if ser_sensores:
                    ser_sensores.close()
                ser_sensores = None
            with state_lock:
                state["status"]["sensores_conectado"] = False
            socketio.emit("status", state["status"])
            time.sleep(2)


# ─── Enviar comando al ESP32 de motores ───────────────────────────────────────

def enviar_comando(cmd: str):
    """Escribe un comando de texto al ESP32 de motores por serial."""
    global ser_motores
    with serial_lock_motores:
        if ser_motores and ser_motores.is_open:
            try:
                ser_motores.write((cmd + "\n").encode("utf-8"))
                return True
            except serial.SerialException as e:
                print(f"[Motores] Error al enviar '{cmd}': {e}")
    return False


# ─── Rutas HTTP ───────────────────────────────────────────────────────────────

@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/state")
def api_state():
    with state_lock:
        return jsonify(state)


@app.route("/api/puertos")
def api_puertos():
    return jsonify({"puertos": listar_puertos()})


# ─── Eventos SocketIO ─────────────────────────────────────────────────────────

@socketio.on("connect")
def on_connect():
    print(f"[WS] Cliente conectado")
    with state_lock:
        emit("tel",    state["tel"])
        emit("sens",   state["sens"])
        emit("status", state["status"])


@socketio.on("disconnect")
def on_disconnect():
    print(f"[WS] Cliente desconectado")


@socketio.on("cmd")
def on_cmd(data):
    """
    El browser envía: { "cmd": "MOVE,0.5,0.0" }
    Se reenvía directamente al ESP32 de motores por serial.

    Comandos compatibles con el firmware existente:
      MOVE,<vx>,<vy>     — joystick  (-1.0 a 1.0)
      STOP               — detener
      EN,1 / EN,0        — habilitar/deshabilitar motores
      KP,<valor>         — ajustar Kp
      KI,<valor>         — ajustar Ki
      PMAX,<valor>       — ajustar pulsos máximos
    """
    cmd = data.get("cmd", "").strip()
    if cmd:
        ok = enviar_comando(cmd)
        emit("cmd_ack", {"cmd": cmd, "ok": ok})


# ─── Arranque ─────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 50)
    print("  GrandRoot — Raspberry Pi Server")
    print("=" * 50)
    print(f"  Puertos disponibles: {listar_puertos()}")
    print(f"  ESP32 Motores  → {SERIAL_MOTORES}")
    print(f"  ESP32 Sensores → {SERIAL_SENSORES}")
    print("=" * 50)

    # Iniciar hilos seriales como daemon (se matan al cerrar el proceso)
    threading.Thread(target=leer_motores,  daemon=True, name="hilo_motores").start()
    threading.Thread(target=leer_sensores, daemon=True, name="hilo_sensores").start()

    # Arrancar Flask con eventlet
    socketio.run(app, host="0.0.0.0", port=5000, debug=False)
