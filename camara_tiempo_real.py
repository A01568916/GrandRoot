"""
camara_tiempo_real.py  —  Visualizacion en tiempo real de deteccion de surcos.

Captura frames de la webcam, los procesa con el modelo U-Net entrenado y
muestra una ventana con:
  - Overlay cian sobre los surcos detectados
  - Linea de navegacion RANSAC (rojo = alta confianza, naranja = baja)
  - Banda central +-45% (amarillo)
  - HUD: direccion, angulo, confianza, FPS

Sin motores ni ESP32 — solo para testear el modelo en campo.

Controles:
  Q / ESC   = salir
  S         = guardar screenshot en la misma carpeta
  SPACE     = pausar / reanudar
"""

import cv2
import numpy as np
import sys
import os
import time
import tensorflow as tf

# ─────────────────────────────────────────────────────────────────────────────
# FUNCIONES CUSTOM DEL MODELO (copiadas de test_model.py para evitar importar
# ese modulo, el cual carga el modelo base y corre evaluaciones al importarse)
# ─────────────────────────────────────────────────────────────────────────────

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

IMG_SIZE = (128, 128)


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


def extract_navigation_line(mask_binary):
    """
    Extrae la linea de navegacion de la mascara binaria (128x128).
    Devuelve: pt_bottom, pt_top, angulo_deg, confianza (0-1).
    """
    h, w = mask_binary.shape
    kernel  = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    cleaned = cv2.morphologyEx(mask_binary, cv2.MORPH_CLOSE, kernel)
    cleaned = cv2.morphologyEx(cleaned,     cv2.MORPH_OPEN,  kernel)

    band_half  = int(w * 0.45)
    band_mask  = np.zeros_like(cleaned)
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
    x_bottom = int(np.clip(a * (h - 1)  + b, 0, w - 1))
    x_top    = int(np.clip(a * 10       + b, 0, w - 1))
    x_mid    = int(a * (h // 2) + b)
    deviation = x_mid - (w // 2)
    angle_deg = float(np.degrees(np.arctan2(deviation, h // 2)))
    confidence = min(1.0, len(centers) / len(range(h - 1, h // 3, -1)))
    return (x_bottom, h - 1), (x_top, 10), angle_deg, confidence

# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURACION  — modifica aqui si es necesario
# ─────────────────────────────────────────────────────────────────────────────

_HERE        = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(_HERE, '..', 'vision_computacional',
                        'modelo_final_vision', 'modelo_final.h5')
CAMERA_INDEX = 1        # 0 = camara integrada del laptop, 1 = webcam USB externa
CONF_MIN     = 0.30     # confianza minima para linea roja (por debajo = naranja)
ANGLE_UMBRAL = 3.0      # grados: |angulo| < ANGLE_UMBRAL -> RECTO

DISPLAY_W    = 960      # ancho de la ventana de visualizacion
DISPLAY_H    = 540      # alto  de la ventana de visualizacion

# ─────────────────────────────────────────────────────────────────────────────
# CARGA DEL MODELO
# ─────────────────────────────────────────────────────────────────────────────

print("=" * 55)
print("  Deteccion de Surcos — Tiempo Real")
print("=" * 55)

if not os.path.exists(MODEL_PATH):
    print(f"\nERROR: No se encontro el modelo en:\n  {MODEL_PATH}")
    sys.exit(1)

print("\nCargando modelo U-Net...")
model = tf.keras.models.load_model(MODEL_PATH, custom_objects=CUSTOM_OBJECTS)
print(f"Modelo listo  ({model.count_params():,} parametros)\n")


# ─────────────────────────────────────────────────────────────────────────────
# PROCESAMIENTO DE UN FRAME
# ─────────────────────────────────────────────────────────────────────────────

def procesar_frame(frame_bgr):
    """
    Recibe un frame BGR de OpenCV.
    Devuelve (overlay_bgr, angulo_deg, confianza).

    overlay_bgr tiene el mismo tamaño que el frame original con los
    overlays dibujados (mascara, linea de navegacion, banda central).
    """
    h_orig, w_orig = frame_bgr.shape[:2]

    # ── Preprocesar para el modelo (128×128, float [0,1]) ────────────────────
    img_rgb   = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    img_model = cv2.resize(img_rgb, IMG_SIZE).astype(np.float32) / 255.0

    # ── Inferencia ───────────────────────────────────────────────────────────
    pred      = model.predict(img_model[np.newaxis, ...], verbose=0)[0, :, :, 0]
    pred_bin  = (pred > 0.5).astype(np.uint8)   # 0 o 1, shape (128,128)

    # ── Extraer linea de navegacion en espacio 128×128 ───────────────────────
    pt_bot_s, pt_top_s, angulo, confianza = extract_navigation_line(pred_bin * 255)

    # ── Escalar puntos al tamaño original ────────────────────────────────────
    sx = w_orig / IMG_SIZE[0]
    sy = h_orig / IMG_SIZE[1]
    pt_bot = (int(pt_bot_s[0] * sx), int(pt_bot_s[1] * sy))
    pt_top = (int(pt_top_s[0] * sx), int(pt_top_s[1] * sy))

    # ── Escalar mascara al tamaño original ───────────────────────────────────
    mask_grande = cv2.resize(pred_bin, (w_orig, h_orig),
                             interpolation=cv2.INTER_NEAREST)

    # ── Construir overlay sobre el frame original ─────────────────────────────
    overlay = frame_bgr.astype(np.float32) / 255.0

    # Capa cian (BGR: [B, G, R] = [0.9, 0.85, 0.0] ≈ cian)
    m3 = mask_grande[:, :, np.newaxis].astype(bool)
    cian_layer = np.zeros_like(overlay)
    cian_layer[mask_grande > 0] = [0.9, 0.85, 0.0]   # cian en BGR float
    overlay = np.where(m3, overlay * 0.55 + cian_layer * 0.45, overlay)

    # Banda central ±45% (amarillo: BGR=[0, 1, 1])
    band_half = int(w_orig * 0.45)
    cx = w_orig // 2
    cv2.line(overlay, (cx - band_half, 0), (cx - band_half, h_orig - 1),
             (0.0, 1.0, 1.0), 2)
    cv2.line(overlay, (cx + band_half, 0), (cx + band_half, h_orig - 1),
             (0.0, 1.0, 1.0), 2)

    # Linea central de referencia (gris)
    cv2.line(overlay, (cx, 0), (cx, h_orig - 1), (0.6, 0.6, 0.6), 1)

    # Linea de navegacion: rojo si conf>=CONF_MIN, naranja si baja confianza
    nav_color = (0.0, 0.0, 1.0) if confianza >= CONF_MIN else (0.0, 0.45, 1.0)
    cv2.line(overlay, pt_bot, pt_top, nav_color, 3)
    cv2.circle(overlay, pt_top, 7, nav_color, -1)

    overlay_bgr = (np.clip(overlay, 0.0, 1.0) * 255).astype(np.uint8)
    return overlay_bgr, angulo, confianza


# ─────────────────────────────────────────────────────────────────────────────
# DIBUJAR HUD (Heads-Up Display)
# ─────────────────────────────────────────────────────────────────────────────

def dibujar_hud(im, angulo, confianza, fps, paused):
    """Dibuja el HUD directamente sobre la imagen im (BGR uint8)."""
    h, w = im.shape[:2]
    font  = cv2.FONT_HERSHEY_DUPLEX
    fontP = cv2.FONT_HERSHEY_PLAIN

    # ── Panel superior oscuro (semitransparente) ──────────────────────────────
    panel_h = 115
    roi = im[:panel_h].astype(np.float32)
    im[:panel_h] = (roi * 0.30).astype(np.uint8)

    # ── Texto de direccion (centrado, grande) ─────────────────────────────────
    if abs(angulo) <= ANGLE_UMBRAL:
        txt_dir   = "^^  RECTO"
        color_dir = (80, 220, 80)      # verde BGR
    elif angulo < -ANGLE_UMBRAL:
        txt_dir   = "<<  GIRAR IZQUIERDA"
        color_dir = (40, 160, 255)     # naranja BGR
    else:
        txt_dir   = "GIRAR DERECHA  >>"
        color_dir = (40, 160, 255)

    scale_dir = 1.05
    thick_dir = 2
    (tw, _), _ = cv2.getTextSize(txt_dir, font, scale_dir, thick_dir)
    tx = max(0, (w - tw) // 2)
    # sombra negra + texto de color
    cv2.putText(im, txt_dir, (tx + 2, 48), font, scale_dir, (0, 0, 0),   thick_dir + 2)
    cv2.putText(im, txt_dir, (tx,     48), font, scale_dir, color_dir,   thick_dir)

    # ── Angulo ────────────────────────────────────────────────────────────────
    ang_color = (200, 230, 255)
    txt_ang   = f"Angulo: {angulo:+.1f} deg"
    cv2.putText(im, txt_ang, (12 + 1, 80 + 1), font, 0.65, (0, 0, 0),   2)
    cv2.putText(im, txt_ang, (12,     80),      font, 0.65, ang_color,   1)

    # ── Barra de confianza ────────────────────────────────────────────────────
    bx, by, bw, bh = 12, 89, 190, 14
    relleno = int(confianza * bw)
    bar_col = (80, 210, 80) if confianza >= CONF_MIN else (0, 90, 210)
    cv2.rectangle(im, (bx, by),          (bx + bw, by + bh), (55, 55, 55), -1)
    if relleno > 0:
        cv2.rectangle(im, (bx, by),      (bx + relleno, by + bh), bar_col, -1)
    cv2.rectangle(im, (bx, by),          (bx + bw, by + bh), (140, 140, 140), 1)
    txt_conf = f"Confianza: {confianza * 100:.0f}%"
    cv2.putText(im, txt_conf, (bx + bw + 8, by + 11), font, 0.55, ang_color, 1)

    # ── FPS (esquina superior derecha) ────────────────────────────────────────
    txt_fps = f"FPS: {fps:.1f}"
    (fw, _), _ = cv2.getTextSize(txt_fps, font, 0.60, 1)
    cv2.putText(im, txt_fps, (w - fw - 12 + 1, 31), font, 0.60, (0, 0, 0),   2)
    cv2.putText(im, txt_fps, (w - fw - 12,     31), font, 0.60, ang_color,   1)

    # ── Flecha de direccion (debajo del panel) ────────────────────────────────
    arr_cx   = w // 2
    arr_by   = panel_h + 44
    arr_len  = 32
    arr_col  = color_dir
    if abs(angulo) <= ANGLE_UMBRAL:
        # flecha hacia arriba
        cv2.arrowedLine(im,
                        (arr_cx, arr_by + arr_len),
                        (arr_cx, arr_by),
                        arr_col, 3, tipLength=0.40)
    elif angulo < 0:
        # flecha a la izquierda
        cv2.arrowedLine(im,
                        (arr_cx + arr_len, arr_by),
                        (arr_cx - arr_len, arr_by),
                        arr_col, 3, tipLength=0.35)
    else:
        # flecha a la derecha
        cv2.arrowedLine(im,
                        (arr_cx - arr_len, arr_by),
                        (arr_cx + arr_len, arr_by),
                        arr_col, 3, tipLength=0.35)

    # ── Texto PAUSADO ─────────────────────────────────────────────────────────
    if paused:
        (pw, ph), _ = cv2.getTextSize("PAUSADO", font, 1.8, 3)
        px = (w - pw) // 2
        py = h // 2 + ph // 2
        cv2.rectangle(im, (px - 14, py - ph - 10), (px + pw + 14, py + 10),
                      (0, 0, 0), -1)
        cv2.putText(im, "PAUSADO", (px, py), font, 1.8, (0, 0, 200), 3)

    # ── Leyenda en la parte inferior ─────────────────────────────────────────
    leyenda = ("Cian=surcos  |  Amarillo=banda+-45%  |  Rojo=nav  |"
               "  [Q/ESC]=salir   [S]=captura   [SPACE]=pausar")
    cv2.putText(im, leyenda, (8 + 1, h - 9 + 1), fontP, 0.88, (0, 0, 0), 2)
    cv2.putText(im, leyenda, (8,     h - 9),      fontP, 0.88, (170, 170, 170), 1)


# ─────────────────────────────────────────────────────────────────────────────
# BUCLE PRINCIPAL
# ─────────────────────────────────────────────────────────────────────────────

def escanear_camaras(max_idx=5):
    """
    Prueba indices 0..max_idx-1 y devuelve los que realmente dan imagen.
    En Windows, isOpened() puede devolver True incluso para indices invalidos,
    por eso se verifica leyendo un frame real.
    """
    encontradas = []
    for idx in range(max_idx):
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)   # CAP_DSHOW = backend Windows nativo
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None and frame.size > 0:
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                encontradas.append((idx, w, h))
        cap.release()
    return encontradas


def main():
    # ── Escanear camaras disponibles ─────────────────────────────────────────
    print("Buscando camaras disponibles...")
    disponibles = escanear_camaras()
    if not disponibles:
        print("\nERROR: No se encontro ninguna camara conectada.")
        print("  - Conecta la webcam y vuelve a ejecutar el script.")
        sys.exit(1)

    print(f"Camaras encontradas: {len(disponibles)}")
    for idx, w, h in disponibles:
        marcador = " <-- (CAMERA_INDEX actual)" if idx == CAMERA_INDEX else ""
        print(f"  Indice {idx} : {w}x{h}{marcador}")

    # Elegir la camara: usar CAMERA_INDEX si esta disponible, si no la primera
    indice_usar = CAMERA_INDEX
    indices_ok  = [i for i, _, _ in disponibles]
    if indice_usar not in indices_ok:
        indice_usar = indices_ok[0]
        print(f"\nAVISO: CAMERA_INDEX={CAMERA_INDEX} no encontrado."
              f" Usando indice {indice_usar} automaticamente.")
        print(f"  Para fijar la camara, cambia CAMERA_INDEX={indice_usar} en el script.\n")

    cap = cv2.VideoCapture(indice_usar, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    real_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    real_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camara abierta  [indice={indice_usar}  {real_w}x{real_h}]")

    cv2.namedWindow("Deteccion de Surcos — Tiempo Real", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Deteccion de Surcos — Tiempo Real", DISPLAY_W, DISPLAY_H)

    print("\nControles de la ventana:")
    print("  Q / ESC  = salir")
    print("  S        = guardar screenshot")
    print("  SPACE    = pausar / reanudar\n")

    fps          = 0.0
    t_prev       = time.time()
    paused       = False
    n_screenshot = 0
    angulo       = 0.0
    confianza    = 0.0
    last_frame   = None   # ultimo overlay calculado (para modo pausa)

    while True:
        # ── Captura y procesamiento (solo si no esta pausado) ─────────────────
        if not paused:
            ret, frame = cap.read()
            if not ret:
                print("Error: no se pudo leer el frame de la camara.")
                break

            last_frame, angulo, confianza = procesar_frame(frame)

            # Calcular FPS con suavizado exponencial
            t_now  = time.time()
            dt     = max(t_now - t_prev, 1e-6)
            fps    = 0.85 * fps + 0.15 * (1.0 / dt)
            t_prev = t_now

        if last_frame is None:
            continue

        # ── Redimensionar al tamaño de display ───────────────────────────────
        display = cv2.resize(last_frame, (DISPLAY_W, DISPLAY_H))

        # ── HUD ───────────────────────────────────────────────────────────────
        dibujar_hud(display, angulo, confianza, fps, paused)

        # ── Mostrar ───────────────────────────────────────────────────────────
        cv2.imshow("Deteccion de Surcos — Tiempo Real", display)

        # ── Teclas ────────────────────────────────────────────────────────────
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):           # Q o ESC
            break
        elif key == ord('s'):
            n_screenshot += 1
            ruta = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                f"screenshot_{n_screenshot:03d}.png")
            cv2.imwrite(ruta, display)
            print(f"Screenshot guardado: {ruta}")
        elif key == ord(' '):
            paused = not paused
            estado = "PAUSADO" if paused else "REANUDADO"
            print(estado)

    cap.release()
    cv2.destroyAllWindows()
    print("\nCerrado correctamente.")


if __name__ == '__main__':
    main()
