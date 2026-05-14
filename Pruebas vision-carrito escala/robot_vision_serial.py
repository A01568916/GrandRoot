"""
robot_vision_serial.py  —  Vision + Control de Motores via Serial

Captura frames de la webcam, los procesa con el modelo U-Net y envia
comandos MOVE al ESP32 por puerto Serial segun la direccion detectada.

Protocolo Serial (identico al Control_PID_Serial original):
  PC  →  ESP32 :  ENABLE,true / ENABLE,false
  PC  →  ESP32 :  MOVE,vx,vy         (vx y vy en rango [-1, 1])
  ESP32 →  PC  :  TEL,pi,pd,ri,rd,ei,ed,daci,dacd

Controles de ventana:
  Q / ESC   = salir (apaga motores antes)
  SPACE     = pausar / reanudar (pausa vision, motors siguen con ultimo cmd)
  E         = habilitar / deshabilitar motores (toggle)
  S         = guardar screenshot
  + / -     = subir / bajar velocidad (REF_PULSOS)
"""

import cv2
import numpy as np
import sys
import os
import time
import threading
import serial
import serial.tools.list_ports
import tensorflow as tf

# =============================================================================
# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                     PARÁMETROS — EDITA AQUI                             ║
# ╚══════════════════════════════════════════════════════════════════════════╝
# =============================================================================

# ── Velocidad de referencia ───────────────────────────────────────────────────
# Pulsos/muestra que se le pide a cada motor al ir recto.
# Rango util: 4 (lento) — 22 (maximo).  Cambia con + / - en tiempo real.
REF_PULSOS = 10            # <─── CAMBIA ESTE VALOR PARA AJUSTAR VELOCIDAD

# ── Factor de giro ────────────────────────────────────────────────────────────
# Fraccion de vx que se aplica como vy al girar (0.0 = sin giro, 1.0 = giro maximo).
# Con 0.5 el robot gira suavemente mientras avanza.
TURN_FACTOR = 0.5

# ── Umbral de angulo para considerar "recto" ─────────────────────────────────
ANGLE_UMBRAL = 3.0         # grados

# ── Confianza minima para actuar ─────────────────────────────────────────────
# Si el modelo detecta con menos confianza que esto, se manda MOVE,0,0 (parar).
CONF_MIN_ACCION = 0.30

# ── Serial ───────────────────────────────────────────────────────────────────
SERIAL_PORT = None         # None = autodetectar ESP32; o pon 'COM3', '/dev/ttyUSB0', etc.
SERIAL_BAUD = 115200

# ── Camara ────────────────────────────────────────────────────────────────────
CAMERA_INDEX = 1           # 0 = integrada, 1 = USB externa
DISPLAY_W    = 960
DISPLAY_H    = 540

# ── Modelo ────────────────────────────────────────────────────────────────────
_HERE      = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(_HERE, '..', 'vision_computacional',
                          'modelo_final_vision', 'modelo_final.h5')
IMG_SIZE   = (128, 128)

# =============================================================================
# FUNCIONES CUSTOM DEL MODELO (copiadas de camara_tiempo_real.py)
# =============================================================================

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
    'bce_dice_loss': bce_dice_loss,
    'dice_coef':     dice_coef,
    'iou_metric':    iou_metric
}

# =============================================================================
# RANSAC + LÍNEA DE NAVEGACIÓN  (identicas a camara_tiempo_real.py)
# =============================================================================

def ransac_line(pts, n_iter=200, threshold=4.0):
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


def extract_navigation_line(mask_binary):
    h, w    = mask_binary.shape
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
        return (w // 2, h - 1), (w // 2, 10), 0.0, 0.0

    pts  = np.array(centers)
    a, b = ransac_line(pts)
    x_bottom = int(np.clip(a * (h - 1) + b, 0, w - 1))
    x_top    = int(np.clip(a * 10      + b, 0, w - 1))
    x_mid    = int(a * (h // 2) + b)
    deviation = x_mid - (w // 2)
    angle_deg = float(np.degrees(np.arctan2(deviation, h // 2)))
    confidence = min(1.0, len(centers) / len(range(h - 1, h // 3, -1)))
    return (x_bottom, h - 1), (x_top, 10), angle_deg, confidence


def procesar_frame(frame_bgr):
    h_orig, w_orig = frame_bgr.shape[:2]
    img_rgb   = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    img_model = cv2.resize(img_rgb, IMG_SIZE).astype(np.float32) / 255.0
    pred      = model.predict(img_model[np.newaxis, ...], verbose=0)[0, :, :, 0]
    pred_bin  = (pred > 0.5).astype(np.uint8)

    pt_bot_s, pt_top_s, angulo, confianza = extract_navigation_line(pred_bin * 255)

    sx = w_orig / IMG_SIZE[0]
    sy = h_orig / IMG_SIZE[1]
    pt_bot = (int(pt_bot_s[0] * sx), int(pt_bot_s[1] * sy))
    pt_top = (int(pt_top_s[0] * sx), int(pt_top_s[1] * sy))

    mask_grande = cv2.resize(pred_bin, (w_orig, h_orig),
                             interpolation=cv2.INTER_NEAREST)
    overlay = frame_bgr.astype(np.float32) / 255.0
    m3 = mask_grande[:, :, np.newaxis].astype(bool)
    cian_layer = np.zeros_like(overlay)
    cian_layer[mask_grande > 0] = [0.9, 0.85, 0.0]
    overlay = np.where(m3, overlay * 0.55 + cian_layer * 0.45, overlay)

    band_half = int(w_orig * 0.45)
    cx = w_orig // 2
    cv2.line(overlay, (cx - band_half, 0), (cx - band_half, h_orig - 1), (0.0, 1.0, 1.0), 2)
    cv2.line(overlay, (cx + band_half, 0), (cx + band_half, h_orig - 1), (0.0, 1.0, 1.0), 2)
    cv2.line(overlay, (cx, 0), (cx, h_orig - 1), (0.6, 0.6, 0.6), 1)

    nav_color = (0.0, 0.0, 1.0) if confianza >= CONF_MIN_ACCION else (0.0, 0.45, 1.0)
    cv2.line(overlay, pt_bot, pt_top, nav_color, 3)
    cv2.circle(overlay, pt_top, 7, nav_color, -1)

    overlay_bgr = (np.clip(overlay, 0.0, 1.0) * 255).astype(np.uint8)
    return overlay_bgr, angulo, confianza

# =============================================================================
# SERIAL — conexión con el ESP32
# =============================================================================

ser        = None          # objeto serial activo
_rx_buffer = ''            # buffer de lineas entrantes
last_tele  = {}            # ultima telemetria recibida

def autodetectar_puerto():
    """Devuelve el primer puerto que tenga 'CP210' o 'CH340' o 'FTDI' en descripcion."""
    for p in serial.tools.list_ports.comports():
        desc = (p.description or '').upper()
        if any(k in desc for k in ('CP210', 'CH340', 'FTDI', 'USB SERIAL', 'ESP')):
            return p.device
    return None


def conectar_serial():
    global ser
    puerto = SERIAL_PORT or autodetectar_puerto()
    if puerto is None:
        print("[Serial] No se encontro ESP32. Conecta el cable USB y reintenta.")
        return False
    try:
        ser = serial.Serial(puerto, SERIAL_BAUD, timeout=0.1)
        time.sleep(2)          # esperar reset del ESP32
        ser.reset_input_buffer()
        print(f"[Serial] Conectado en {puerto} @ {SERIAL_BAUD} baud")
        return True
    except serial.SerialException as e:
        print(f"[Serial] Error al abrir {puerto}: {e}")
        return False


def enviar(cmd: str):
    """Envia un comando al ESP32. Hilo-seguro."""
    if ser and ser.is_open:
        try:
            ser.write((cmd.strip() + '\n').encode())
        except serial.SerialException:
            pass


def _hilo_rx():
    """Hilo que lee telemetria entrante del ESP32 y la guarda en last_tele."""
    global _rx_buffer, last_tele
    while True:
        if ser is None or not ser.is_open:
            time.sleep(0.1)
            continue
        try:
            datos = ser.read(ser.in_waiting or 1).decode(errors='replace')
        except serial.SerialException:
            time.sleep(0.1)
            continue

        _rx_buffer += datos
        while '\n' in _rx_buffer:
            linea, _rx_buffer = _rx_buffer.split('\n', 1)
            linea = linea.strip()
            if linea.startswith('TEL,'):
                p = linea.split(',')
                try:
                    if len(p) >= 9:
                        # Formato robot grande: TEL,pi,pd,ri,rd,ei,ed,daci,dacd
                        last_tele = {
                            'pi':   int(p[1]),   'pd':   int(p[2]),
                            'ri':   int(p[3]),   'rd':   int(p[4]),
                            'ei': float(p[5]),   'ed': float(p[6]),
                            'di':   int(p[7]),   'dd':   int(p[8]),
                        }
                    elif len(p) >= 4:
                        # Formato carrito simple: TEL,vx,vy,DIR
                        last_tele = {
                            'vx': float(p[1]),
                            'vy': float(p[2]),
                            'dir': p[3].strip(),
                        }
                except ValueError:
                    pass

# =============================================================================
# LÓGICA DE DIRECCIÓN → vx, vy
# =============================================================================

def angulo_a_velocidades(angulo: float, confianza: float, ref_pulsos: int):
    """
    Convierte el angulo de navegacion en (vx, vy) normalizados [-1, 1]
    usando REF_PULSOS como referencia de velocidad base.

    vx = velocidad lineal  (adelante/atras)
    vy = velocidad angular (izquierda/derecha)

    Si la confianza es baja se detiene.
    Si el angulo es < ANGLE_UMBRAL va recto.
    Si no, avanza y gira proporcional al angulo.
    """
    vx_base = ref_pulsos / 22.0   # normalizado respecto al maximo (PULSOS_MAX=22)
    vx_base = max(0.0, min(1.0, vx_base))

    if confianza < CONF_MIN_ACCION:
        return 0.0, 0.0           # sin confianza: parar

    if abs(angulo) <= ANGLE_UMBRAL:
        return vx_base, 0.0       # recto

    # Giro proporcional al angulo (clamp a ±1)
    # angulo positivo = girar derecha → vy positivo
    vy = (angulo / 45.0) * TURN_FACTOR
    vy = max(-1.0, min(1.0, vy))
    return vx_base, vy

# =============================================================================
# HUD  (adaptado de camara_tiempo_real.py, agrega info de motores/serial)
# =============================================================================

def dibujar_hud(im, angulo, confianza, fps, paused, motors_on,
                serial_ok, ref_pulsos, last_tele):
    h, w  = im.shape[:2]
    font  = cv2.FONT_HERSHEY_DUPLEX
    fontP = cv2.FONT_HERSHEY_PLAIN

    # Panel superior oscuro
    panel_h = 130
    roi = im[:panel_h].astype(np.float32)
    im[:panel_h] = (roi * 0.30).astype(np.uint8)

    # Direccion
    if abs(angulo) <= ANGLE_UMBRAL:
        txt_dir, color_dir = "^^  RECTO",           (80, 220, 80)
    elif angulo < 0:
        txt_dir, color_dir = "<<  GIRAR IZQUIERDA", (40, 160, 255)
    else:
        txt_dir, color_dir = "GIRAR DERECHA  >>",   (40, 160, 255)

    (tw, _), _ = cv2.getTextSize(txt_dir, font, 1.05, 2)
    tx = max(0, (w - tw) // 2)
    cv2.putText(im, txt_dir, (tx + 2, 48), font, 1.05, (0, 0, 0),   4)
    cv2.putText(im, txt_dir, (tx,     48), font, 1.05, color_dir,   2)

    # Angulo y confianza
    cv2.putText(im, f"Angulo: {angulo:+.1f} deg",       (12, 80), font, 0.65, (200, 230, 255), 1)
    cv2.putText(im, f"Confianza: {confianza*100:.0f}%",  (12,100), font, 0.60, (200, 230, 255), 1)

    # Velocidad referencia
    cv2.putText(im, f"Ref: {ref_pulsos} pul/s  [+/-]",  (12,120), font, 0.60, (180, 255, 180), 1)

    # Estado Serial y Motores (esquina derecha)
    col_serial  = (80, 220, 80)  if serial_ok    else (50, 50, 220)
    col_motor   = (80, 220, 80)  if motors_on    else (50, 50, 220)
    txt_serial  = "Serial: OK"  if serial_ok    else "Serial: OFF"
    txt_motor   = "Motor: ON"   if motors_on    else "Motor: OFF"
    cv2.putText(im, txt_serial, (w - 170, 28), font, 0.60, col_serial, 1)
    cv2.putText(im, txt_motor,  (w - 170, 50), font, 0.60, col_motor,  1)

    # FPS
    (fw, _), _ = cv2.getTextSize(f"FPS: {fps:.1f}", font, 0.60, 1)
    cv2.putText(im, f"FPS: {fps:.1f}", (w - fw - 12, 72), font, 0.60, (200, 230, 255), 1)

    # Telemetria inferior (si hay datos)
    if last_tele:
        tele_txt = (f"I: ref={last_tele.get('ri','-'):>3}  pul={last_tele.get('pi','-'):>3}"
                    f"  err={last_tele.get('ei','-'):>5.1f}  dac={last_tele.get('di','-'):>3}   |   "
                    f"D: ref={last_tele.get('rd','-'):>3}  pul={last_tele.get('pd','-'):>3}"
                    f"  err={last_tele.get('ed','-'):>5.1f}  dac={last_tele.get('dd','-'):>3}")
        cv2.putText(im, tele_txt, (8, h - 22), fontP, 0.85, (0, 0, 0),         2)
        cv2.putText(im, tele_txt, (8, h - 22), fontP, 0.85, (170, 200, 255),   1)

    # PAUSADO
    if paused:
        (pw, ph), _ = cv2.getTextSize("PAUSADO", font, 1.8, 3)
        px, py = (w - pw) // 2, h // 2 + ph // 2
        cv2.rectangle(im, (px - 14, py - ph - 10), (px + pw + 14, py + 10), (0, 0, 0), -1)
        cv2.putText(im, "PAUSADO", (px, py), font, 1.8, (0, 0, 200), 3)

    # Leyenda
    ley = "E=motor  SPACE=pausar  +/-=velocidad  S=captura  Q/ESC=salir"
    cv2.putText(im, ley, (8, h - 8), fontP, 0.85, (0, 0, 0),       2)
    cv2.putText(im, ley, (8, h - 8), fontP, 0.85, (160, 160, 160), 1)

# =============================================================================
# MAIN
# =============================================================================

def escanear_camaras(max_idx=5):
    encontradas = []
    for idx in range(max_idx):
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None and frame.size > 0:
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                encontradas.append((idx, w, h))
        cap.release()
    return encontradas


def main():
    global ser, REF_PULSOS

    # ── Modelo ────────────────────────────────────────────────────────────────
    print("=" * 55)
    print("  Robot Vision — Control de Motores via Serial")
    print("=" * 55)

    if not os.path.exists(MODEL_PATH):
        print(f"\nERROR: No se encontro el modelo en:\n  {MODEL_PATH}")
        sys.exit(1)

    print("\nCargando modelo U-Net...")
    global model
    model = tf.keras.models.load_model(MODEL_PATH, custom_objects=CUSTOM_OBJECTS)
    print(f"Modelo listo  ({model.count_params():,} parametros)\n")

    # ── Serial ────────────────────────────────────────────────────────────────
    serial_ok = conectar_serial()
    if serial_ok:
        hilo = threading.Thread(target=_hilo_rx, daemon=True)
        hilo.start()

    # ── Camara ────────────────────────────────────────────────────────────────
    print("Buscando camaras disponibles...")
    disponibles = escanear_camaras()
    if not disponibles:
        print("\nERROR: No se encontro ninguna camara.")
        if ser:
            ser.close()
        sys.exit(1)

    indices_ok  = [i for i, _, _ in disponibles]
    indice_usar = CAMERA_INDEX if CAMERA_INDEX in indices_ok else indices_ok[0]
    cap = cv2.VideoCapture(indice_usar, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    print(f"Camara abierta [indice={indice_usar}]\n")

    cv2.namedWindow("Robot Vision — Motor Control", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Robot Vision — Motor Control", DISPLAY_W, DISPLAY_H)

    print("Controles:")
    print("  E         = habilitar / deshabilitar motores")
    print("  SPACE     = pausar / reanudar vision")
    print("  + / -     = subir / bajar velocidad")
    print("  S         = guardar screenshot")
    print("  Q / ESC   = salir\n")

    # ── Estado ────────────────────────────────────────────────────────────────
    motors_on    = False
    paused       = False
    fps          = 0.0
    t_prev       = time.time()
    angulo       = 0.0
    confianza    = 0.0
    last_frame   = None
    n_screenshot = 0

    # Habilitar motores automaticamente si hay serial
    if serial_ok:
        enviar("ENABLE,true")
        motors_on = True
        print("[Motor] ENABLE,true enviado")

    # ── Bucle principal ───────────────────────────────────────────────────────
    while True:

        # Vision (solo si no esta pausado)
        if not paused:
            ret, frame = cap.read()
            if not ret:
                print("Error: no se pudo leer el frame.")
                break

            last_frame, angulo, confianza = procesar_frame(frame)

            t_now  = time.time()
            dt     = max(t_now - t_prev, 1e-6)
            fps    = 0.85 * fps + 0.15 * (1.0 / dt)
            t_prev = t_now

            # Calcular y enviar MOVE solo si motores habilitados
            if motors_on and serial_ok:
                vx, vy = angulo_a_velocidades(angulo, confianza, REF_PULSOS)
                enviar(f"MOVE,{vx:.4f},{vy:.4f}")

        if last_frame is None:
            continue

        # Display
        display = cv2.resize(last_frame, (DISPLAY_W, DISPLAY_H))
        dibujar_hud(display, angulo, confianza, fps, paused,
                    motors_on, serial_ok and ser.is_open,
                    REF_PULSOS, last_tele)
        cv2.imshow("Robot Vision — Motor Control", display)

        # Teclas
        key = cv2.waitKey(1) & 0xFF

        if key in (ord('q'), 27):                   # Salir
            break

        elif key == ord('e'):                        # Toggle motores
            motors_on = not motors_on
            cmd = "ENABLE,true" if motors_on else "ENABLE,false"
            enviar(cmd)
            print(f"[Motor] {cmd}")
            if not motors_on:
                enviar("MOVE,0.0000,0.0000")

        elif key == ord(' '):                        # Pausar vision
            paused = not paused
            if paused:
                enviar("MOVE,0.0000,0.0000")        # parar al pausar
            print("PAUSADO" if paused else "REANUDADO")

        elif key == ord('+') or key == ord('='):     # Subir velocidad
            REF_PULSOS = min(22, REF_PULSOS + 1)
            print(f"[Velocidad] REF_PULSOS = {REF_PULSOS}")

        elif key == ord('-'):                        # Bajar velocidad
            REF_PULSOS = max(1, REF_PULSOS - 1)
            print(f"[Velocidad] REF_PULSOS = {REF_PULSOS}")

        elif key == ord('s'):                        # Screenshot
            n_screenshot += 1
            ruta = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                f"screenshot_{n_screenshot:03d}.png")
            cv2.imwrite(ruta, display)
            print(f"Screenshot guardado: {ruta}")

    # ── Cierre limpio ─────────────────────────────────────────────────────────
    print("\nCerrando...")
    if serial_ok and ser.is_open:
        enviar("MOVE,0.0000,0.0000")
        enviar("ENABLE,false")
        time.sleep(0.3)
        ser.close()
        print("[Serial] Cerrado")

    cap.release()
    cv2.destroyAllWindows()
    print("Cerrado correctamente.")


if __name__ == '__main__':
    main()
