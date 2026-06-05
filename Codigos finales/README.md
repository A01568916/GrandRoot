# GrandRoot — Sistema Unificado

Sistema de control para robot diferencial con telemetría inalámbrica.
**100% offline** — no requiere Raspberry Pi, ni cable USB en operación, ni internet.

## Arquitectura

```
┌─────────────────────────┐    TX/RX    ┌──────────────────────────────┐
│  ESP32 SENSORES         │   Serial1   │  ESP32 PID (motores)         │
│  · GPS NEO-6M           │ ──────────► │  · WiFi AP "ESP32-Robot"     │
│  · IMU MPU6050          │  GPIO 4     │  · WebSocket ws://192.168.4.1/ws │
│                         │  → GPIO 16  │  · Sirve dashboard HTML      │
└─────────────────────────┘   115200    └────────────┬─────────────────┘
                                                     │ WiFi
                                                     │
                                              ┌──────▼──────┐
                                              │  Navegador  │
                                              │  (celular,  │
                                              │   laptop)   │
                                              └─────────────┘
```

## Lo importante

- **Leaflet y Chart.js van EMBEBIDOS en el HTML** — no se descargan de internet
- **Tiles del mapa**: ArcGIS World Imagery (satélite, sin API key), CartoDB (calles), OpenTopoMap (topográfico) — todos sin API key, todos con CORS habilitado
- **Caché offline**: cuando tengas internet, ve a tu parcela y pulsa "⬇ Guardar zona" — los tiles se guardan en IndexedDB del navegador y funcionan después sin internet

## Conexiones físicas

### ESP32 SENSORES → ESP32 PID
| ESP32 SENSORES | ESP32 PID | Función |
|---|---|---|
| GPIO 4 (TX1) | GPIO 16 (RX2) | Datos JSON |
| GND | GND | **Referencia común — ¡obligatorio!** |

### ESP32 SENSORES — periféricos
| Periférico | Pin ESP32 |
|---|---|
| GPS TXD → | GPIO 16 (RX2) |
| GPS RXD ← | GPIO 17 (TX2) |
| MPU6050 SDA | GPIO 21 |
| MPU6050 SCL | GPIO 22 |
| MPU6050 AD0 | GND (dirección I2C 0x68) |

### ESP32 PID — motores
Sin cambios respecto al firmware original (pines 14, 18, 21, 25, 26, 27, 33, 34).

## Archivos

| Archivo | Para qué |
|---|---|
| `esp32_sensores.ino` | Firmware del ESP32 de sensores |
| `Control_PID_WIFI.ino` | Firmware del ESP32 de motores + WiFi |
| `index_html.h` | Dashboard embebido con Leaflet+Chart.js (~424 KB en PROGMEM) |
| `grandroot_dashboard_offline.html` | El dashboard completo standalone — ábrelo en cualquier navegador para desarrollo |
| `grandroot_dashboard.html` | Versión "fuente" sin las librerías embebidas (más fácil de editar) |
| `build_dashboard.py` | Script que toma `grandroot_dashboard.html` + Leaflet + Chart.js y produce `grandroot_dashboard_offline.html` e `index_html.h` |
| `leaflet.css`, `leaflet.js`, `chart.js` | Librerías que el script embebe |

## Librerías Arduino necesarias

**ESP32 SENSORES:**
- TinyGPS++

**ESP32 PID:**
- ESPAsyncWebServer (lacamera / me-no-dev)
- AsyncTCP (dvarrel / me-no-dev)

## Uso

1. Sube `esp32_sensores.ino` al ESP32 de sensores.
2. Sube `Control_PID_WIFI.ino` (junto con `index_html.h` en la misma carpeta) al ESP32 de motores.
3. Conéctate a la red WiFi **`ESP32-Robot`** (contraseña: `robot1234`).
4. Abre el navegador en **http://192.168.4.1**.

La página inicial puede tardar 5–10 segundos en cargar la primera vez (424 KB por WiFi del ESP32 no es instantáneo), pero después el navegador la cachea y abre rápido.

## Mapa

El mapa tiene 3 capas seleccionables (botones arriba a la izquierda del mapa):
- **Satélite** — ArcGIS World Imagery (recomendado para parcelas en el campo)
- **Calles** — CartoDB Voyager (mapa estilo claro para zonas urbanas)
- **Topo** — OpenTopoMap (curvas de nivel para terrenos accidentados)

### Para usar el mapa sin internet

1. Conecta tu celular/laptop a internet (red doméstica)
2. Abre `http://192.168.4.1` (necesitas estar conectado al ESP32 también — usa hotspot si tu OS soporta múltiples redes, o pre-cachea con el HTML standalone primero)
3. Selecciona la capa que quieras usar
4. Navega el mapa hasta tu parcela
5. Pulsa **⬇ Guardar zona** abajo a la izquierda
6. Espera a que termine (verás el progreso en un toast)
7. Repite con otras capas o zoom si lo necesitas

Los tiles se guardan en IndexedDB del navegador y quedan disponibles offline para siempre (hasta que limpies datos del sitio).

**Alternativa más práctica para desarrollo**: abre `grandroot_dashboard_offline.html` directamente en tu navegador con internet, navega y guarda la zona, ya queda cacheada en ese navegador. Cuando luego abras `http://192.168.4.1` desde el mismo navegador y misma máquina, el caché se reusa (mismo origen no, pero IndexedDB sí persiste por dominio... ojo: este truco requiere abrir el dashboard del ESP32 al menos una vez para que IndexedDB se asocie al dominio `192.168.4.1`).

## Modo de operación

- **Modo Automático**: muestra un modal "🚧 en desarrollo" — placeholder para U-Net futura
- **Modo Manual**: habilita el D-pad del panel izquierdo y los controles de teclado

Teclado en modo Manual:
- `W / ↑` — adelante
- `S / ↓` — atrás
- `A / ←` — girar izquierda
- `D / →` — girar derecha
- `Espacio` — STOP

## Botón "Control Manual" (header)

Despliega un panel con:
- Slider de velocidad de referencia (0–22 pul/s)
- Telemetría detallada de motor izquierdo y derecho (Ref, Pulsos, Error, Esfuerzo DAC)
- 4 gráficas en tiempo real (Ref vs Medida × 2, Error, Esfuerzo)

Las gráficas siguen acumulando datos aunque el panel esté cerrado.

## Panel Admin

Botón ⚙ Admin del header. Contraseña: `grandroot2025`

Permite ajustar en tiempo real: Kp, Ki, Pulsos Max, VMAX, WMAX, Ref Min Giro, DAC Min Arranque.

## Protocolo WebSocket (ws://192.168.4.1/ws)

**Cliente → ESP32:**
- `ENABLE,true` / `ENABLE,false` — activar/desactivar motores
- `MOVE,vx,vy` — vx,vy ∈ [-1, 1]
- `PARAM,nombre,valor` — ajustar parámetro PI

**ESP32 → Cliente:**
- `TEL,pi,pd,ri,rd,ei,ed,daci,dacd` — telemetría motores (cada 200 ms)
- `SENS,{...json...}` — telemetría sensores (cada 200 ms si llega del ESP32 SENSORES)
- `ENABLE,true/false` — eco de cambio de estado
- `PARAM_OK,nombre,valor` — confirmación de parámetro

## Modificar el dashboard

Si quieres cambiar algo del HTML/JS/CSS:

1. Edita `grandroot_dashboard.html`
2. Ejecuta: `python3 build_dashboard.py`
3. Esto regenera `grandroot_dashboard_offline.html` (para probar en PC) e `index_html.h` (para subir al ESP32)
4. Vuelve a subir `Control_PID_WIFI.ino` al ESP32

## Diagnóstico

- **No carga el dashboard**: 424 KB son ~5–10 s por WiFi del ESP32. Espera. Si después de 30 s no carga, mira la consola del navegador.
- **Sin datos de sensores**: revisa el cable TX→RX entre los dos ESP32 y **GND común**. El log USB del ESP32 PID muestra `SENS_age=...ms` — si crece mucho, no llega nada.
- **Sin GPS fix**: GPS necesita cielo abierto. Espera 1-2 minutos al primer arranque.
- **No conecta el WS**: la red `ESP32-Robot` está conectada pero el ESP32 puede estar reiniciándose. Verifica el log USB del ESP32 PID.
- **Mapa con cuadros grises**: no tienes internet y no has cacheado tiles. Conéctate a una red con internet, ve a tu zona, pulsa "Guardar zona".
- **Tiles 403**: el provider de OSM ya no se usa porque les molesta el scraping; ahora se usa ArcGIS por defecto, que tolera bien las peticiones.
