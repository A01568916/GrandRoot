#!/usr/bin/env python3
"""
grandroot_server.py
GrandRoot — Servidor Flask + Serial USB
Raspberry Pi 5  ←USB→  ESP32

Dependencias:
    pip install flask pyserial

Uso:
    python3 grandroot_server.py [--port /dev/ttyESP32] [--baud 115200] [--http-port 8080]
"""

import argparse
import logging
import signal
import sys
import threading
import time
from pathlib import Path
from collections import deque

import serial
import serial.tools.list_ports
from flask import Flask, jsonify, request, send_file

# ─────────────────────────────────────────────
# Config
# ─────────────────────────────────────────────
DEFAULT_SERIAL_PORT = "/dev/ttyESP32"
DEFAULT_BAUD        = 115200
DEFAULT_HTTP_PORT   = 8080
HTML_FILE           = Path(__file__).parent / "index.html"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-7s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("grandroot")

app = Flask(__name__)

# ─────────────────────────────────────────────
# Estado global
# ─────────────────────────────────────────────
class State:
    def __init__(self):
        self.serial_conn: serial.Serial | None = None
        self.serial_lock = threading.Lock()
        # Último frame de telemetría recibido
        self.telemetria = {
            "pi": 0, "pd": 0,
            "ri": 0, "rd": 0,
            "ei": 0.0, "ed": 0.0,
            "daci": 0, "dacd": 0,
        }
        self.tele_lock = threading.Lock()
        self.motores_activos = False

state = State()

# ─────────────────────────────────────────────
# Serial helpers
# ─────────────────────────────────────────────
def serial_write(line: str):
    if state.serial_conn and state.serial_conn.is_open:
        try:
            with state.serial_lock:
                state.serial_conn.write((line.strip() + "\n").encode())
            log.info(f"[Serial →] {line.strip()}")
        except serial.SerialException as e:
            log.warning(f"[Serial] Error al escribir: {e}")

def serial_reader_thread():
    log.info("[Serial] Hilo lector iniciado")
    buf = b""
    while True:
        if not state.serial_conn or not state.serial_conn.is_open:
            time.sleep(1.0)
            continue
        try:
            chunk = state.serial_conn.read(256)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode(errors="replace").strip()
                if not text:
                    continue
                log.debug(f"[Serial ←] {text}")
                parse_serial(text)
        except serial.SerialException as e:
            log.error(f"[Serial] Error: {e}")
            time.sleep(2.0)

def parse_serial(text: str):
    if text.startswith("TEL,"):
        parts = text.split(",")
        if len(parts) >= 9:
            with state.tele_lock:
                # TEL: medida_izq, medida_der, ref_izq, ref_der, error_izq, error_der, dac_izq, dac_der
                state.telemetria = {
                    "ri":   _n(parts[1]),  # medida izq (pulsos reales)
                    "rd":   _n(parts[2]),  # medida der (pulsos reales)
                    "pi":   _n(parts[3]),  # referencia izq
                    "pd":   _n(parts[4]),  # referencia der
                    "ei":   _n(parts[5]),  # error izq
                    "ed":   _n(parts[6]),  # error der
                    "daci": _n(parts[7]),  # DAC izq
                    "dacd": _n(parts[8]),  # DAC der
                }

def _n(v):
    try:
        x = float(v)
        return x if x == x else 0  # nan check
    except Exception:
        return 0

# ─────────────────────────────────────────────
# Rutas Flask
# ─────────────────────────────────────────────

@app.route("/")
def index():
    return send_file(HTML_FILE)

@app.route("/telemetria")
def telemetria():
    with state.tele_lock:
        data = dict(state.telemetria)
    data["motores"] = state.motores_activos
    return jsonify(data)

@app.route("/enable", methods=["POST"])
def enable():
    body = request.get_json(silent=True) or {}
    valor = bool(body.get("value", False))
    state.motores_activos = valor
    serial_write(f"ENABLE,{'true' if valor else 'false'}")
    return jsonify({"ok": True, "motores": valor})

@app.route("/move", methods=["POST"])
def move():
    body = request.get_json(silent=True) or {}
    vx = float(body.get("vx", 0))
    vy = float(body.get("vy", 0))
    serial_write(f"MOVE,{vx:.4f},{vy:.4f}")
    return jsonify({"ok": True})

@app.route("/param", methods=["POST"])
def param():
    body = request.get_json(silent=True) or {}
    nombre = body.get("nombre", "")
    valor  = body.get("valor", 0)
    if not nombre:
        return jsonify({"error": "nombre requerido"}), 400
    serial_write(f"PARAM,{nombre},{valor}")
    return jsonify({"ok": True, "nombre": nombre, "valor": valor})

# ─────────────────────────────────────────────
# Serial open
# ─────────────────────────────────────────────
def open_serial(port: str, baud: int) -> serial.Serial:
    candidates = [port]
    if not Path(port).exists():
        log.warning(f"[Serial] {port} no existe, buscando ESP32...")
        for p in serial.tools.list_ports.comports():
            desc = p.description or ""
            if any(x in desc for x in ("CP210", "CH340", "UART", "ESP")):
                candidates.insert(0, p.device)
                log.info(f"[Serial] Candidato: {p.device} ({desc})")

    for candidate in candidates:
        try:
            conn = serial.Serial(
                port=candidate, baudrate=baud,
                timeout=0.1, write_timeout=1.0,
                dsrdtr=False,
                rtscts=False,
            )
            conn.dtr = False
            log.info(f"[Serial] Conectado a {candidate} @ {baud} baud")
            return conn
        except serial.SerialException as e:
            log.warning(f"[Serial] No se pudo abrir {candidate}: {e}")

    raise RuntimeError(
        f"No se pudo conectar al ESP32. "
        f"Puertos disponibles: {[p.device for p in serial.tools.list_ports.comports()]}"
    )

# ─────────────────────────────────────────────
# Shutdown
# ─────────────────────────────────────────────
def shutdown(sig, frame):
    log.info("\n[Main] Apagando... enviando ENABLE,false")
    serial_write("ENABLE,false")
    if state.serial_conn and state.serial_conn.is_open:
        state.serial_conn.close()
    sys.exit(0)

# ─────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="GrandRoot Flask Server")
    parser.add_argument("--port",      default=DEFAULT_SERIAL_PORT)
    parser.add_argument("--baud",      type=int, default=DEFAULT_BAUD)
    parser.add_argument("--http-port", type=int, default=DEFAULT_HTTP_PORT)
    args = parser.parse_args()

    signal.signal(signal.SIGINT,  shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        state.serial_conn = open_serial(args.port, args.baud)
    except RuntimeError as e:
        log.error(str(e))
        sys.exit(1)

    reader = threading.Thread(target=serial_reader_thread, daemon=True)
    reader.start()

    log.info("─" * 50)
    log.info("  GrandRoot Flask Server listo")
    log.info(f"  Interfaz web → http://<IP-de-RPi>:{args.http_port}")
    log.info(f"  Serial ESP32 → {args.port} @ {args.baud}")
    log.info("─" * 50)

    app.run(host="0.0.0.0", port=args.http_port, debug=False, threaded=True)
