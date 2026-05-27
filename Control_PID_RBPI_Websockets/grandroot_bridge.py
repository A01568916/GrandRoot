#!/usr/bin/env python3
"""
grandroot_bridge.py
GrandRoot — Puente Serial-USB ↔ WebSocket + Servidor HTTP
Raspberry Pi 5  ←USB→  ESP32

Dependencias:
    pip install pyserial websockets

Uso:
    python3 grandroot_bridge.py [--port /dev/ttyUSB0] [--baud 115200] [--http-port 8080] [--ws-port 8765]

El ESP32 debe tener su firmware modificado:
  - Sin WiFi / WebSocket propio (o puede coexistir si lo deseas)
  - Enviar/recibir por Serial a 115200 baud
  - Mismos comandos: ENABLE,true|false  /  MOVE,vx,vy
  - Misma telemetría: TEL,pi,pd,ri,rd,ei,ed,daci,dacd

Arquitectura:
    Browser  ←WS:8765→  RPi bridge  ←Serial USB→  ESP32
    Browser  ←HTTP:8080→ RPi (sirve index.html)
"""

import asyncio
import argparse
import logging
import signal
import sys
import threading
from pathlib import Path

import serial
import serial.tools.list_ports
import websockets
from http.server import HTTPServer, SimpleHTTPRequestHandler
from functools import partial

# ─────────────────────────────────────────────
# Configuración por defecto
# ─────────────────────────────────────────────

DEFAULT_SERIAL_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD        = 115200
DEFAULT_HTTP_PORT   = 8080
DEFAULT_WS_PORT     = 8765
HTML_FILE           = Path(__file__).parent / "index.html"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-7s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("grandroot")


# ─────────────────────────────────────────────
# Estado global compartido
# ─────────────────────────────────────────────

class State:
    def __init__(self):
        self.serial_conn: serial.Serial | None = None
        self.ws_clients: set = set()
        self.serial_lock = threading.Lock()
        self.loop: asyncio.AbstractEventLoop | None = None

state = State()


# ─────────────────────────────────────────────
# Escritura serial (thread-safe)
# ─────────────────────────────────────────────

def serial_write(line: str):
    """Envía una línea al ESP32 por Serial."""
    if state.serial_conn and state.serial_conn.is_open:
        try:
            with state.serial_lock:
                state.serial_conn.write((line.strip() + "\n").encode())
        except serial.SerialException as e:
            log.warning(f"[Serial] Error al escribir: {e}")


# ─────────────────────────────────────────────
# Hilo de lectura Serial → broadcast WebSocket
# ─────────────────────────────────────────────

def serial_reader_thread():
    """
    Lee líneas del ESP32 y las reenvía a todos los clientes WS conectados.
    Corre en un hilo separado (Serial es blocking I/O).
    """
    log.info("[Serial] Hilo lector iniciado")
    buf = b""

    while True:
        if not state.serial_conn or not state.serial_conn.is_open:
            threading.Event().wait(1.0)
            continue
        try:
            chunk = state.serial_conn.read(256)   # no-blocking con timeout=0.1
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode(errors="replace").strip()
                if not text:
                    continue
                log.debug(f"[Serial ←] {text}")
                # Enviar a todos los clientes WS (desde hilo → asyncio)
                if state.loop and state.ws_clients:
                    asyncio.run_coroutine_threadsafe(
                        broadcast_ws(text), state.loop
                    )
        except serial.SerialException as e:
            log.error(f"[Serial] Error de lectura: {e}")
            threading.Event().wait(2.0)
        except Exception as e:
            log.error(f"[Serial] Excepción inesperada: {e}")
            threading.Event().wait(1.0)


async def broadcast_ws(message: str):
    """Envía un mensaje a todos los clientes WebSocket conectados."""
    if not state.ws_clients:
        return
    dead = set()
    for ws in state.ws_clients.copy():
        try:
            await ws.send(message)
        except Exception:
            dead.add(ws)
    state.ws_clients -= dead


# ─────────────────────────────────────────────
# Handler WebSocket  Browser → ESP32
# ─────────────────────────────────────────────

async def ws_handler(websocket):
    client_addr = websocket.remote_address
    log.info(f"[WS] Cliente conectado: {client_addr}")
    state.ws_clients.add(websocket)

    try:
        async for message in websocket:
            text = message.strip()
            if not text:
                continue
            log.debug(f"[WS →] {text}")
            serial_write(text)
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        state.ws_clients.discard(websocket)
        log.info(f"[WS] Cliente desconectado: {client_addr}")
        # Si no quedan clientes, apagar motores por seguridad
        if not state.ws_clients:
            serial_write("ENABLE,false")
            log.warning("[WS] Sin clientes — motores desactivados por seguridad")


# ─────────────────────────────────────────────
# Servidor HTTP (sirve index.html)
# ─────────────────────────────────────────────

class HTMLHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, html_path: Path, **kwargs):
        self.html_path = html_path
        super().__init__(*args, **kwargs)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            try:
                content = self.html_path.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)
            except FileNotFoundError:
                self.send_error(404, f"No se encontró {self.html_path}")
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        log.info(f"[HTTP] {format % args}")


def start_http_server(port: int):
    handler = partial(HTMLHandler, html_path=HTML_FILE)
    server = HTTPServer(("0.0.0.0", port), handler)
    log.info(f"[HTTP] Servidor en http://0.0.0.0:{port}")
    server.serve_forever()


# ─────────────────────────────────────────────
# Conexión Serial con auto-detect
# ─────────────────────────────────────────────

def open_serial(port: str, baud: int) -> serial.Serial:
    # Intentar el puerto especificado, o auto-detectar ESP32
    candidates = [port]
    if not Path(port).exists():
        log.warning(f"[Serial] {port} no existe, buscando ESP32...")
        for p in serial.tools.list_ports.comports():
            if "CP210" in (p.description or "") or \
               "CH340" in (p.description or "") or \
               "UART"  in (p.description or ""):
                candidates.insert(0, p.device)
                log.info(f"[Serial] Candidato detectado: {p.device} ({p.description})")

    for candidate in candidates:
        try:
            conn = serial.Serial(
                port=candidate,
                baudrate=baud,
                timeout=0.1,
                write_timeout=1.0,
            )
            log.info(f"[Serial] Conectado a {candidate} @ {baud} baud")
            return conn
        except serial.SerialException as e:
            log.warning(f"[Serial] No se pudo abrir {candidate}: {e}")

    raise RuntimeError(
        f"No se pudo conectar al ESP32. "
        f"Verifica que el cable USB esté conectado y que el puerto sea correcto.\n"
        f"Puertos disponibles: {[p.device for p in serial.tools.list_ports.comports()]}"
    )


# ─────────────────────────────────────────────
# Señal de salida limpia
# ─────────────────────────────────────────────

def shutdown(sig, frame):
    log.info("\n[Main] Apagando... enviando ENABLE,false al ESP32")
    serial_write("ENABLE,false")
    if state.serial_conn and state.serial_conn.is_open:
        state.serial_conn.close()
    sys.exit(0)


# ─────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────

async def main(args):
    # Conectar Serial
    state.serial_conn = open_serial(args.port, args.baud)

    # Guardar referencia al event loop para el hilo lector
    state.loop = asyncio.get_running_loop()

    # Iniciar hilo lector serial (daemon → se cierra con el proceso)
    reader = threading.Thread(target=serial_reader_thread, daemon=True)
    reader.start()

    # Iniciar servidor HTTP en hilo separado
    http_thread = threading.Thread(
        target=start_http_server,
        args=(args.http_port,),
        daemon=True,
    )
    http_thread.start()

    # Iniciar servidor WebSocket
    log.info(f"[WS] Servidor en ws://0.0.0.0:{args.ws_port}")
    async with websockets.serve(ws_handler, "0.0.0.0", args.ws_port):
        log.info("─" * 50)
        log.info("  GrandRoot Bridge listo")
        log.info(f"  Interfaz web → http://<IP-de-RPi>:{args.http_port}")
        log.info(f"  WebSocket    → ws://<IP-de-RPi>:{args.ws_port}")
        log.info(f"  Serial ESP32 → {args.port} @ {args.baud}")
        log.info("─" * 50)
        await asyncio.Future()   # corre para siempre


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="GrandRoot Serial↔WS Bridge")
    parser.add_argument("--port",      default=DEFAULT_SERIAL_PORT, help="Puerto serial del ESP32")
    parser.add_argument("--baud",      type=int, default=DEFAULT_BAUD,      help="Baud rate")
    parser.add_argument("--http-port", type=int, default=DEFAULT_HTTP_PORT, help="Puerto HTTP")
    parser.add_argument("--ws-port",   type=int, default=DEFAULT_WS_PORT,   help="Puerto WebSocket")
    args = parser.parse_args()

    signal.signal(signal.SIGINT,  shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        asyncio.run(main(args))
    except RuntimeError as e:
        log.error(str(e))
        sys.exit(1)
