# GrandRoot — Sistema distribuido: PC → Raspberry Pi → ESP32s

## Arquitectura

```
PC (browser)
    │  HTTP + Socket.IO
    ▼
Raspberry Pi  ──Flask + flask-socketio──
    │                                  │
    │ Serial USB /dev/ttyESP32         │ Serial USB /dev/ttySensores
    ▼                                  ▼
ESP32 Motores                    ESP32 Sensores
(PID + encoders)                 (GPS + IMU)
```

## 1. Raspberry Pi — instalación

```bash
pip install flask flask-socketio pyserial eventlet
```

Estructura de archivos:
```
grandroot/
├── app.py
└── templates/
    └── index.html
```

Ejecutar:
```bash
cd grandroot
python app.py
```

Abrir en el navegador: `http://<ip-de-la-raspberry>:5000`

---

## 2. Alias de puertos USB (udev)

Para que los ESP32 siempre tengan el mismo nombre de puerto, identifica su número
de serie USB y crea reglas udev.

### Encontrar el serial de cada ESP32

Conecta un ESP32 a la vez y ejecuta:
```bash
udevadm info -a -n /dev/ttyUSB0 | grep "ATTRS{serial}"
```
Anota el valor (ejemplo: `0001` o `A50285BI0`).

### Crear las reglas

Crea el archivo `/etc/udev/rules.d/99-grandroot.rules`:

```
# ESP32 Motores
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  ATTRS{serial}=="<SERIAL_MOTORES>", SYMLINK+="ttyESP32"

# ESP32 Sensores
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  ATTRS{serial}=="<SERIAL_SENSORES>", SYMLINK+="ttySensores"
```

> Nota: el idVendor `10c4` / idProduct `ea60` es del chip CP2102 (el más común
> en módulos ESP32). Si tu módulo usa CH340, usa `1a86`/`7523` en su lugar.
> Compruébalo con `lsusb`.

Recargar udev:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Verificar:
```bash
ls -la /dev/ttyESP32 /dev/ttySensores
```

---

## 3. Firmware ESP32 Motores

Archivo: `firmware/ESP32_Motores/Control_PID_Serial.ino`

- **Sin WiFi** — no crea AP, no sirve HTML.
- Baud rate: 115200 por USB.
- Salida (→ Raspberry): `TEL,medida_i,medida_d,ref_i,ref_d,error_i,error_d,dac_i,dac_d`
- Entrada (← Raspberry): `MOVE,vx,vy` / `STOP` / `EN,1` / `EN,0` / `KP,val` / `KI,val` / `PMAX,val`

---

## 4. Firmware ESP32 Sensores

Archivo: `firmware/ESP32_Sensores/ESP32_Sensores_Serial.ino`

- **Sin WiFi ni ESP-NOW** — datos por USB.
- Baud rate: 115200.
- Salida (→ Raspberry): `SENS,{"lat":...,"lon":...,"x":...,"y":...,"sats":...,"ax":...,...}`
- Frecuencia: cada 200 ms.

Librerías requeridas (Arduino Library Manager):
- `TinyGPS++`

---

## 5. Protocolo de comunicación

### ESP32 Motores → Raspberry Pi
```
TEL,-3,5,10,10,13.0,5.0,140,100
ENABLE,true
PARAM_OK,kp,8.0000
```

### Raspberry Pi → ESP32 Motores
```
MOVE,0.5000,-0.2000
STOP
EN,1
KP,8.0
PMAX,22
```

### ESP32 Sensores → Raspberry Pi
```
SENS,{"lat":28.738577,"lon":-106.121736,"x":0.000,"y":0.000,"sats":8,"ax":0.012,"ay":0.003,"az":0.998,"gx":0.01,"gy":0.02,"gz":0.00,"gps_ok":1,"imu_ok":1,"ts":12540}
```

---

## 6. Auto-arranque en la Raspberry Pi (systemd)

Crea `/etc/systemd/system/grandroot.service`:

```ini
[Unit]
Description=GrandRoot Dashboard Server
After=network.target

[Service]
ExecStart=/usr/bin/python3 /home/pi/grandroot/app.py
WorkingDirectory=/home/pi/grandroot
Restart=always
User=pi

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable grandroot
sudo systemctl start grandroot
```

---

## 7. Notas de diseño

- El HTML del dashboard es **idéntico** al original. Solo cambia el transporte:
  antes era `WebSocket` directo al ESP32; ahora es `Socket.IO` a la Raspberry.
- La Raspberry Pi reconecta automáticamente si un ESP32 se desconecta o reinicia.
- El estado de conexión de cada puerto serial se emite como evento `status`
  al dashboard.
