"""
╔══════════════════════════════════════════════════════════════════════════════╗
║  GrandRoot — Raspberry Pi 5 — Script Principal                              ║
║                                                                              ║
║  Corre tres hilos en paralelo:                                              ║
║    1. Hilo VISION   — abre camara USB, carga modelo U-Net, detecta lineas   ║
║                       y calcula el angulo de error cada frame               ║
║    2. Hilo CONTROL  — lee el angulo calculado y envia comandos JSON         ║
║                       al ESP32 de motores por Serial USB                    ║
║    3. Hilo SENSORES — lee paquetes JSON del ESP32 de sensores (GPS+IMU)     ║
║                       y los almacena en estado compartido                   ║
║  Un servidor Flask+SocketIO sirve el dashboard HTML en tiempo real.         ║
║                                                                              ║
║  Uso:                                                                        ║
║    python3 grandroot_main.py                                                 ║
║                                                                              ║
║  Prerequisitos (instalar con pip):                                           ║
║    pip3 install opencv-python tensorflow flask flask-socketio pyserial      ║
║                                                                              ║
║  Conectar los ESP32 por USB antes de ejecutar.                              ║
║  Identificar los puertos con: ls /dev/ttyUSB*                               ║
║  Si los puertos se invierten, intercambiar PORT_SENSORES y PORT_MOTORES.    ║
╚══════════════════════════════════════════════════════════════════════════════╝
"""

import cv2
import numpy as np
import serial
import serial.tools.list_ports
import threading
import time
import json
import logging
import sys
import os
import tensorflow as tf

from flask import Flask, render_template_string
from flask_socketio import SocketIO

# ═══════════════════════════════════════════════════════════════════════════
# LOGGING — configuracion centralizada
# Todos los modulos usan el mismo logger "grandroot"
# Nivel INFO en consola, DEBUG en archivo grandroot.log
# ═══════════════════════════════════════════════════════════════════════════

LOG_FILE = "grandroot.log"

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[
        logging.FileHandler(LOG_FILE, encoding="utf-8"),
        logging.StreamHandler(sys.stdout),
    ]
)
# Reducir verbosidad de librerias externas
logging.getLogger("werkzeug").setLevel(logging.WARNING)
logging.getLogger("engineio").setLevel(logging.WARNING)
logging.getLogger("socketio").setLevel(logging.WARNING)
logging.getLogger("tensorflow").setLevel(logging.ERROR)

log = logging.getLogger("grandroot")

# ═══════════════════════════════════════════════════════════════════════════
# CONFIGURACION — modifica aqui segun tu hardware
# ═══════════════════════════════════════════════════════════════════════════

# Puertos USB de los ESP32
# Para identificarlos: ls /dev/ttyUSB*  (conecta uno a la vez si no sabes cual es cual)
PORT_SENSORES = "/dev/ttyUSB0"   # ESP32 que tiene GPS + IMU
PORT_MOTORES  = "/dev/ttyUSB1"   # ESP32 que tiene los drivers

BAUD_RATE     = 115200           # mismo baud en ambos ESP32

# Ruta al modelo de vision U-Net
MODEL_PATH = os.path.join(os.path.dirname(__file__), "modelo_final.h5")

# Indice de la camara USB (0 = primera camara USB detectada por OpenCV)
CAMERA_INDEX = 0

# Umbral de angulo en grados: si |angulo| < ANGLE_UMBRAL → ir recto
ANGLE_UMBRAL = 3.0

# Confianza minima para considerar que la linea es valida
CONF_MIN = 0.30

# Velocidad de avance cuando va recto (0.0 a 1.0)
# Ajusta este valor para cambiar la velocidad del robot
VX_BASE = 0.5

# Factor de giro: cuanto gira en funcion del angulo detectado
# angulo_max_esperado ~30 grados → vy_max = 1.0
# vy = angulo / ANGULO_ESCALA  (se limita a ±1)
ANGULO_ESCALA = 30.0

# Puerto del servidor web
WEB_PORT = 5000

# Tamaño de imagen que espera el modelo
IMG_SIZE = (128, 128)

# Intervalo de timeout: si no llegan datos del ESP32 de sensores
# en este tiempo, se registra una advertencia
SENSOR_TIMEOUT_S = 2.0

# ═══════════════════════════════════════════════════════════════════════════
# ESTADO COMPARTIDO ENTRE HILOS
# Todas las variables que se comparten usan un Lock para evitar
# condiciones de carrera (leer a mitad de escritura)
# ═══════════════════════════════════════════════════════════════════════════

class EstadoGlobal:
    """
    Clase que centraliza todo el estado del sistema.
    Todos los hilos leen y escriben aqui, siempre bajo el lock.

    Por que usar Lock:
    Python tiene el GIL pero operaciones compuestas (leer+escribir un dict)
    no son atomicas. El Lock garantiza que ningun hilo lee un estado parcialmente
    escrito por otro.
    """

    def __init__(self):
        self._lock = threading.Lock()

        # Datos de vision (escritos por hilo_vision, leidos por hilo_control)
        self.angulo     = 0.0
        self.confianza  = 0.0
        self.direccion  = "RECTO"   # "RECTO", "IZQUIERDA", "DERECHA"
        self.vision_ok  = False

        # Datos de sensores (escritos por hilo_sensores, leidos por servidor web)
        self.gps_lat    = 0.0
        self.gps_lon    = 0.0
        self.gps_x      = 0.0
        self.gps_y      = 0.0
        self.gps_sats   = 0
        self.gps_ok     = False
        self.imu_ax     = 0.0
        self.imu_ay     = 0.0
        self.imu_az     = 0.0
        self.imu_gx     = 0.0
        self.imu_gy     = 0.0
        self.imu_gz     = 0.0
        self.imu_ok     = False
        self.sensor_ts  = 0         # timestamp del ultimo paquete del ESP32

        # Telemetria de motores (escrita por hilo_control al recibir TEL del ESP32)
        self.motor_pi   = 0
        self.motor_pd   = 0
        self.motor_ri   = 0
        self.motor_rd   = 0
        self.motor_daci = 0
        self.motor_dacd = 0
        self.motor_ok   = False

        # Control del sistema
        self.sistema_activo = True    # False = apagar todos los hilos
        self.motores_on     = False   # True = motores habilitados
        self.param_pendiente = None   # dict {"param":..., "value":...} o None

    def leer(self):
        """Devuelve una copia del estado completo como dict (thread-safe)."""
        with self._lock:
            return {
                "angulo":    self.angulo,
                "confianza": self.confianza,
                "direccion": self.direccion,
                "vision_ok": self.vision_ok,
                "lat":       self.gps_lat,
                "lon":       self.gps_lon,
                "x":         self.gps_x,
                "y":         self.gps_y,
                "sats":      self.gps_sats,
                "gps_ok":    self.gps_ok,
                "ax": self.imu_ax, "ay": self.imu_ay, "az": self.imu_az,
                "gx": self.imu_gx, "gy": self.imu_gy, "gz": self.imu_gz,
                "imu_ok":    self.imu_ok,
                "motor_pi":  self.motor_pi,
                "motor_pd":  self.motor_pd,
                "motor_ri":  self.motor_ri,
                "motor_rd":  self.motor_rd,
                "daci":      self.motor_daci,
                "dacd":      self.motor_dacd,
                "motor_ok":  self.motor_ok,
                "motores_on": self.motores_on,
            }

    def escribir_vision(self, angulo, confianza, direccion):
        with self._lock:
            self.angulo    = angulo
            self.confianza = confianza
            self.direccion = direccion
            self.vision_ok = True

    def escribir_sensores(self, datos: dict):
        with self._lock:
            self.gps_lat  = datos.get("lat",  0.0)
            self.gps_lon  = datos.get("lon",  0.0)
            self.gps_x    = datos.get("x",    0.0)
            self.gps_y    = datos.get("y",    0.0)
            self.gps_sats = datos.get("sats", 0)
            self.gps_ok   = datos.get("gps_ok", False)
            self.imu_ax   = datos.get("ax", 0.0)
            self.imu_ay   = datos.get("ay", 0.0)
            self.imu_az   = datos.get("az", 0.0)
            self.imu_gx   = datos.get("gx", 0.0)
            self.imu_gy   = datos.get("gy", 0.0)
            self.imu_gz   = datos.get("gz", 0.0)
            self.imu_ok   = datos.get("imu_ok", False)
            self.sensor_ts = datos.get("ts", 0)

    def escribir_telemetria_motor(self, datos: dict):
        with self._lock:
            self.motor_pi   = datos.get("pi",   0)
            self.motor_pd   = datos.get("pd",   0)
            self.motor_ri   = datos.get("ri",   0)
            self.motor_rd   = datos.get("rd",   0)
            self.motor_daci = datos.get("daci", 0)
            self.motor_dacd = datos.get("dacd", 0)
            self.motor_ok   = True


estado = EstadoGlobal()

# ═══════════════════════════════════════════════════════════════════════════
# FUNCIONES DEL MODELO DE VISION
# (copiadas de camara_tiempo_real.py — sin modificar la logica)
# ═══════════════════════════════════════════════════════════════════════════

def dice_coef(y_true, y_pred, smooth=1.0):
    y_true_f = tf.keras.backend.flatten(y_true)
    y_pred_f = tf.keras.backend.flatten(y_pred)
    inter    = tf.keras.backend.sum(y_true_f * y_pred_f)
    return (2. * inter + smooth) / (
        tf.keras.backend.sum(y_true_f) + tf.keras.backend.sum(y_pred_f) + smooth)

def dice_loss(y_true, y_pred):
    return 1.0 - dice_coef(y_true, y_pred)

def bce_dice_loss(y_true, y_pred):
    return tf.keras.losses.binary_crossentropy(y_true, y_pred) + dice_loss(y_true, y_pred)

def iou_metric(y_true, y_pred):
    y_pred_b = tf.cast(y_pred > 0.5, tf.float32)
    inter    = tf.reduce_sum(y_true * y_pred_b)
    union    = tf.reduce_sum(y_true) + tf.reduce_sum(y_pred_b) - inter
    return (inter + 1.0) / (union + 1.0)

CUSTOM_OBJECTS = {
    "bce_dice_loss": bce_dice_loss,
    "dice_coef":     dice_coef,
    "iou_metric":    iou_metric,
}


def ransac_line(pts, n_iter=200, threshold=4.0):
    """Ajusta la recta x = a*y + b a los puntos usando RANSAC."""
    best_coeffs  = None
    best_inliers = 0
    xs = pts[:, 0].astype(float)
    ys = pts[:, 1].astype(float)
    for _ in range(n_iter):
        idx = np.random.choice(len(pts), 2, replace=False)
        y1, y2 = ys[idx[0]], ys[idx[1]]
        x1, x2 = xs[idx[0]], xs[idx[1]]
        if abs(y2 - y1) < 1e-6:
            continue
        a = (x2 - x1) / (y2 - y1)
        b = x1 - a * y1
        residuals = np.abs(xs - (a * ys + b))
        n_in = np.sum(residuals < threshold)
        if n_in > best_inliers:
            best_inliers = n_in
            best_coeffs  = (a, b)
    if best_coeffs is None:
        return np.polyfit(ys, xs, 1)
    return best_coeffs


def extraer_linea_navegacion(mask_binary):
    """
    Extrae angulo y confianza de la mascara binaria (128x128).
    Devuelve: (angulo_deg, confianza)
    """
    h, w = mask_binary.shape
    kernel  = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    cleaned = cv2.morphologyEx(mask_binary, cv2.MORPH_CLOSE, kernel)
    cleaned = cv2.morphologyEx(cleaned,     cv2.MORPH_OPEN,  kernel)

    band_half = int(w * 0.45)
    band_mask = np.zeros_like(cleaned)
    band_mask[:, w//2 - band_half : w//2 + band_half] = \
        cleaned[:, w//2 - band_half : w//2 + band_half]

    centers = []
    for row_y in range(h - 1, h // 3, -1):
        cols = np.where(band_mask[row_y] > 0)[0]
        if len(cols) >= 3:
            centers.append((int(np.mean(cols)), row_y))

    if len(centers) < 8:
        centers = []
        for row_y in range(h - 1, h // 3, -1):
            cols = np.where(cleaned[row_y] > 0)[0]
            if len(cols) >= 3:
                centers.append((int(np.mean(cols)), row_y))

    if len(centers) < 5:
        return 0.0, 0.0

    pts  = np.array(centers)
    a, b = ransac_line(pts)
    x_mid    = int(a * (h // 2) + b)
    deviation = x_mid - (w // 2)
    angle_deg = float(np.degrees(np.arctan2(deviation, h // 2)))
    confidence = min(1.0, len(centers) / len(range(h - 1, h // 3, -1)))
    return angle_deg, confidence


# ═══════════════════════════════════════════════════════════════════════════
# HILO 1 — VISION
# Abre la camara, carga el modelo una vez, procesa frames continuamente
# y escribe angulo+confianza en el estado global
# ═══════════════════════════════════════════════════════════════════════════

def hilo_vision(model):
    """
    Hilo de vision. No retorna hasta que sistema_activo sea False.

    Por que es un hilo separado:
    La inferencia del modelo tarda ~50-200 ms por frame en la RPi5.
    Si estuviera en el hilo principal bloquaria la lectura de sensores
    y la comunicacion con los ESP32 durante ese tiempo.
    """
    log.info("[VISION] Iniciando hilo de vision. Abriendo camara %d...", CAMERA_INDEX)

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        log.error("[VISION][ERROR] No se pudo abrir la camara %d. "
                  "Verifica que esta conectada y no esta en uso por otro proceso.", CAMERA_INDEX)
        # El sistema sigue funcionando sin vision (modo solo GPS)
        estado.vision_ok = False
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    log.info("[VISION] Camara abierta. Resolución: %dx%d",
             int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
             int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

    frames_procesados = 0
    t_inicio = time.time()

    while estado.sistema_activo:
        ret, frame = cap.read()
        if not ret:
            log.warning("[VISION][WARN] Frame perdido (ret=False). "
                        "La camara puede haberse desconectado.")
            time.sleep(0.1)
            continue

        # Preprocesar para el modelo
        img_rgb   = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_model = cv2.resize(img_rgb, IMG_SIZE).astype(np.float32) / 255.0

        # Inferencia
        pred     = model.predict(img_model[np.newaxis, ...], verbose=0)[0, :, :, 0]
        pred_bin = (pred > 0.5).astype(np.uint8)

        # Extraer angulo y confianza
        angulo, confianza = extraer_linea_navegacion(pred_bin * 255)

        # Determinar direccion
        if confianza < CONF_MIN:
            # Confianza baja: ir recto por defecto (mas seguro que girar sin certeza)
            direccion = "RECTO"
            log.debug("[VISION] Confianza baja (%.2f < %.2f) — comando RECTO por defecto",
                      confianza, CONF_MIN)
        elif abs(angulo) <= ANGLE_UMBRAL:
            direccion = "RECTO"
        elif angulo < -ANGLE_UMBRAL:
            direccion = "IZQUIERDA"
        else:
            direccion = "DERECHA"

        estado.escribir_vision(angulo, confianza, direccion)

        frames_procesados += 1
        if frames_procesados % 30 == 0:
            fps = frames_procesados / (time.time() - t_inicio)
            log.info("[VISION] %d frames procesados | FPS=%.1f | angulo=%.1f dir=%s conf=%.2f",
                     frames_procesados, fps, angulo, direccion, confianza)

    cap.release()
    log.info("[VISION] Hilo de vision terminado.")


# ═══════════════════════════════════════════════════════════════════════════
# HILO 2 — CONTROL DE MOTORES
# Lee el angulo del estado global y envia comandos al ESP32 de motores
# Tambien lee la telemetria que devuelve el ESP32
# ═══════════════════════════════════════════════════════════════════════════

def hilo_control(ser_motores: serial.Serial):
    """
    Hilo de control. Se comunica con el ESP32 de motores por Serial USB.
    Envia un comando MOVE cada ciclo segun el angulo calculado por vision.
    Lee la telemetria que devuelve el ESP32.
    """
    log.info("[CONTROL] Hilo de control iniciado.")

    # Habilitar motores al arrancar
    _enviar_comando(ser_motores, {"cmd": "ENABLE", "val": True})
    with estado._lock:
        estado.motores_on = True

    # Buffer para leer telemetria linea por linea
    tel_buffer = ""

    while estado.sistema_activo:

        # ── Leer telemetria del ESP32 si hay datos disponibles ────────────
        try:
            while ser_motores.in_waiting > 0:
                c = ser_motores.read(1).decode("utf-8", errors="ignore")
                if c == "\n":
                    if tel_buffer:
                        try:
                            datos = json.loads(tel_buffer)
                            if datos.get("tel"):
                                estado.escribir_telemetria_motor(datos)
                        except json.JSONDecodeError:
                            log.debug("[CONTROL] Telemetria malformada: %s", tel_buffer)
                    tel_buffer = ""
                else:
                    tel_buffer += c
                    if len(tel_buffer) > 300:
                        log.warning("[CONTROL][WARN] Buffer telemetria overflow — descartando.")
                        tel_buffer = ""
        except serial.SerialException as e:
            log.error("[CONTROL][ERROR] Error leyendo telemetria del ESP32 de motores: %s", e)
            log.error("[CONTROL][ERROR] El ESP32 de motores puede haberse desconectado.")
            break

        # ── Enviar parámetro pendiente si el admin lo solicitó ────────────
        with estado._lock:
            pendiente = estado.param_pendiente
            estado.param_pendiente = None

        if pendiente:
            cmd = {"cmd": "SET_PARAM", "param": pendiente["param"], "value": pendiente["value"]}
            _enviar_comando(ser_motores, cmd)
            log.info("[CONTROL] Parámetro enviado al ESP32: %s = %s",
                     pendiente["param"], pendiente["value"])

        # ── Calcular y enviar comando de movimiento ────────────────────────
        angulo    = estado.angulo
        confianza = estado.confianza
        vision_ok = estado.vision_ok

        if not vision_ok:
            # Sin vision, detener por seguridad
            _enviar_comando(ser_motores, {"cmd": "MOVE", "vx": 0.0, "vy": 0.0})
        else:
            # Calcular vx y vy
            vx = VX_BASE if confianza >= CONF_MIN else 0.0

            # vy proporcional al angulo: angulo positivo = girar derecha = vy positivo
            # Se escala para que ANGULO_ESCALA grados = vy maxima
            vy = float(np.clip(angulo / ANGULO_ESCALA, -1.0, 1.0))

            # Si la confianza es baja, no girar
            if confianza < CONF_MIN:
                vy = 0.0

            _enviar_comando(ser_motores, {"cmd": "MOVE", "vx": vx, "vy": vy})

        # Cadencia de control: el ESP32 tiene SAMPLE_MS=200 ms
        # No tiene sentido enviar mas rapido que eso
        time.sleep(0.18)

    # Apagar motores al salir
    log.warning("[CONTROL] Saliendo del hilo — deshabilitando motores.")
    _enviar_comando(ser_motores, {"cmd": "STOP"})


def _enviar_comando(ser: serial.Serial, datos: dict):
    """
    Serializa el dict como JSON y lo envia al ESP32 por Serial.
    Agrega \\n al final (el ESP32 usa readline-style parsing).
    """
    try:
        linea = json.dumps(datos, separators=(",", ":")) + "\n"
        ser.write(linea.encode("utf-8"))
    except serial.SerialException as e:
        log.error("[CONTROL][ERROR] Fallo al enviar comando %s: %s", datos, e)


# ═══════════════════════════════════════════════════════════════════════════
# HILO 3 — LECTURA DE SENSORES
# Lee paquetes JSON del ESP32 de sensores (GPS + IMU) por Serial USB
# ═══════════════════════════════════════════════════════════════════════════

def hilo_sensores(ser_sensores: serial.Serial):
    """
    Hilo de sensores. Lee paquetes JSON del ESP32 de sensores.
    El ESP32 envia un JSON por linea cada INTERVALO_MS (200 ms).
    """
    log.info("[SENSORES] Hilo de sensores iniciado.")

    buffer     = ""
    t_ultimo   = time.time()
    paquetes   = 0

    while estado.sistema_activo:

        # ── Detectar timeout: si no llegan datos, el ESP32 puede estar desconectado
        if time.time() - t_ultimo > SENSOR_TIMEOUT_S:
            log.warning("[SENSORES][WARN] Sin datos del ESP32 de sensores por %.1f s. "
                        "Verifica conexion USB en %s.", SENSOR_TIMEOUT_S, PORT_SENSORES)
            t_ultimo = time.time()  # reset para no repetir el warning cada ciclo

        # ── Leer datos disponibles ─────────────────────────────────────────
        try:
            while ser_sensores.in_waiting > 0:
                c = ser_sensores.read(1).decode("utf-8", errors="ignore")
                if c == "\n":
                    if buffer:
                        try:
                            datos = json.loads(buffer)
                            estado.escribir_sensores(datos)
                            t_ultimo = time.time()
                            paquetes += 1
                            if paquetes % 50 == 0:
                                log.info("[SENSORES] %d paquetes recibidos | "
                                         "GPS: lat=%.6f lon=%.6f sats=%d ok=%s | "
                                         "IMU: ax=%.2f ay=%.2f az=%.2f",
                                         paquetes,
                                         datos.get("lat", 0), datos.get("lon", 0),
                                         datos.get("sats", 0), datos.get("gps_ok", False),
                                         datos.get("ax", 0), datos.get("ay", 0), datos.get("az", 0))
                        except json.JSONDecodeError as e:
                            log.debug("[SENSORES] JSON malformado: %s | Error: %s", buffer, e)
                    buffer = ""
                else:
                    buffer += c
                    if len(buffer) > 500:
                        log.warning("[SENSORES][WARN] Buffer overflow — descartando linea.")
                        buffer = ""

        except serial.SerialException as e:
            log.error("[SENSORES][ERROR] Error leyendo del ESP32 de sensores: %s", e)
            log.error("[SENSORES][ERROR] El ESP32 de sensores puede haberse desconectado.")
            break

        time.sleep(0.01)  # ceder CPU brevemente

    log.info("[SENSORES] Hilo de sensores terminado.")


# ═══════════════════════════════════════════════════════════════════════════
# SERVIDOR WEB — Flask + SocketIO
# Sirve el dashboard HTML y emite telemetria por WebSocket cada 200 ms
# ═══════════════════════════════════════════════════════════════════════════

# Leer el archivo HTML del dashboard
_DASHBOARD_PATH = os.path.join(os.path.dirname(__file__), "grandroot_dashboard.html")
try:
    with open(_DASHBOARD_PATH, "r", encoding="utf-8") as f:
        DASHBOARD_HTML = f.read()
    log.info("[WEB] Dashboard HTML cargado desde %s", _DASHBOARD_PATH)
except FileNotFoundError:
    DASHBOARD_HTML = "<h1>GrandRoot — Dashboard no encontrado</h1>"
    log.warning("[WEB][WARN] grandroot_dashboard.html no encontrado en %s", _DASHBOARD_PATH)

app = Flask(__name__)
app.config["SECRET_KEY"] = "grandroot2025"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")


@app.route("/")
def index():
    return DASHBOARD_HTML


@socketio.on("connect")
def on_connect():
    log.info("[WEB] Cliente conectado al dashboard.")


@socketio.on("disconnect")
def on_disconnect():
    log.info("[WEB] Cliente desconectado del dashboard.")


@socketio.on("emergency_stop")
def on_emergency(data=None):
    """El dashboard puede enviar una parada de emergencia."""
    log.warning("[WEB][EMERGENCIA] Parada de emergencia solicitada desde el dashboard.")
    estado.sistema_activo = False


@socketio.on("set_param")
def on_set_param(data):
    """
    El panel admin del dashboard envía un parámetro nuevo para el ESP32 de motores.
    Ejemplo: {"param": "kp", "value": 8.0}
    Parámetros soportados: kp, ki, sample_ms, pulsos_max
    El comando llega al hilo_control que lo reenvía al ESP32 por Serial.
    """
    param = data.get("param")
    value = data.get("value")
    params_validos = {"kp", "ki", "sample_ms", "pulsos_max"}

    if param not in params_validos:
        log.warning("[WEB][PARAM] Parámetro desconocido recibido: %s", param)
        return
    if value is None or not isinstance(value, (int, float)) or value <= 0:
        log.warning("[WEB][PARAM] Valor inválido para %s: %s", param, value)
        return

    log.info("[WEB][PARAM] Recibido %s = %s desde el dashboard admin.", param, value)
    with estado._lock:
        estado.param_pendiente = {"param": param, "value": value}



def hilo_broadcast():
    """
    Emite el estado completo al dashboard cada 200 ms via SocketIO.
    Corre dentro del contexto de la aplicacion Flask.
    """
    log.info("[WEB] Hilo de broadcast iniciado.")
    with app.app_context():
        while estado.sistema_activo:
            datos = estado.leer()
            socketio.emit("telemetry", datos)
            time.sleep(0.2)
    log.info("[WEB] Hilo de broadcast terminado.")


# ═══════════════════════════════════════════════════════════════════════════
# ABRIR PUERTOS SERIAL — con diagnostico detallado si falla
# ═══════════════════════════════════════════════════════════════════════════

def abrir_serial(port: str, baud: int, nombre: str) -> serial.Serial:
    """
    Intenta abrir un puerto Serial.
    Si falla, lista los puertos disponibles para facilitar el diagnostico.
    """
    try:
        ser = serial.Serial(port, baud, timeout=1.0)
        log.info("[SERIAL] %s abierto en %s a %d baud.", nombre, port, baud)
        return ser
    except serial.SerialException as e:
        log.error("[SERIAL][ERROR] No se pudo abrir %s en %s: %s", nombre, port, e)
        log.error("[SERIAL][ERROR] Puertos USB disponibles:")
        for p in serial.tools.list_ports.comports():
            log.error("   %s — %s", p.device, p.description)
        log.error("[SERIAL][ERROR] Edita PORT_SENSORES y PORT_MOTORES en la seccion "
                  "CONFIGURACION de este script para usar el puerto correcto.")
        sys.exit(1)


# ═══════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════

def main():
    log.info("=" * 60)
    log.info("  GrandRoot — Sistema de navegacion autonoma")
    log.info("=" * 60)

    # ── Cargar modelo de vision ───────────────────────────────────────────
    if not os.path.exists(MODEL_PATH):
        log.error("[VISION][ERROR] Modelo no encontrado en: %s", MODEL_PATH)
        log.error("[VISION][ERROR] Coloca modelo_final.h5 en la misma carpeta que este script.")
        sys.exit(1)

    log.info("[VISION] Cargando modelo U-Net desde %s ...", MODEL_PATH)
    model = tf.keras.models.load_model(MODEL_PATH, custom_objects=CUSTOM_OBJECTS)
    log.info("[VISION] Modelo cargado (%d parametros).", model.count_params())

    # ── Abrir puertos Serial ──────────────────────────────────────────────
    ser_sensores = abrir_serial(PORT_SENSORES, BAUD_RATE, "ESP32-Sensores")
    ser_motores  = abrir_serial(PORT_MOTORES,  BAUD_RATE, "ESP32-Motores")

    # Dar tiempo al ESP32 para reiniciarse despues de abrir el puerto
    # (la RPi resetea el ESP32 al abrir el puerto Serial, normal en Arduino)
    log.info("[SERIAL] Esperando 2 s para que los ESP32 terminen de inicializar...")
    time.sleep(2.0)

    # ── Lanzar hilos ──────────────────────────────────────────────────────
    hilos = [
        threading.Thread(target=hilo_vision,    args=(model,),        name="vision",   daemon=True),
        threading.Thread(target=hilo_control,   args=(ser_motores,),  name="control",  daemon=True),
        threading.Thread(target=hilo_sensores,  args=(ser_sensores,), name="sensores", daemon=True),
        threading.Thread(target=hilo_broadcast,                        name="web_bcast",daemon=True),
    ]

    for h in hilos:
        h.start()
        log.info("[MAIN] Hilo '%s' iniciado.", h.name)

    # ── Servidor web (bloqueante — corre en el hilo principal) ────────────
    log.info("[WEB] Dashboard disponible en http://0.0.0.0:%d", WEB_PORT)
    log.info("[WEB] Abre esa URL en cualquier dispositivo en la misma red.")
    log.info("[MAIN] Sistema activo. Ctrl+C para detener.")

    try:
        socketio.run(app, host="0.0.0.0", port=WEB_PORT, debug=False, use_reloader=False)
    except KeyboardInterrupt:
        log.info("[MAIN] Ctrl+C recibido — apagando sistema...")

    # ── Apagado ordenado ─────────────────────────────────────────────────
    estado.sistema_activo = False
    log.info("[MAIN] Esperando que los hilos terminen...")
    for h in hilos:
        h.join(timeout=3.0)

    # Parada final de motores por seguridad
    try:
        _enviar_comando(ser_motores, {"cmd": "STOP"})
    except Exception:
        pass

    ser_sensores.close()
    ser_motores.close()
    log.info("[MAIN] Sistema apagado correctamente.")


if __name__ == "__main__":
    main()
