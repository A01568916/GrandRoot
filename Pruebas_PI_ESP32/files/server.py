from flask import Flask, request, jsonify, send_file
import serial
import serial.tools.list_ports
import time

app = Flask(__name__)

# ── Configuración Serial ───────────────────────────────────────────
# Cambia este puerto si es necesario. Para encontrarlo corre:
#   ls /dev/ttyUSB* /dev/ttyACM*
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE   = 9600

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Espera a que el ESP32 arranque
    print(f'[OK] ESP32 conectado en {SERIAL_PORT}')
except Exception as e:
    ser = None
    print(f'[WARN] No se pudo abrir serial: {e}')

# ── Rutas ──────────────────────────────────────────────────────────
@app.route('/')
def index():
    return send_file('leds.html')

@app.route('/led', methods=['POST'])
def led():
    data  = request.get_json()
    color = data.get('color', '').lower()

    if color not in ('rojo', 'azul'):
        return jsonify({'error': 'color inválido'}), 400

    if ser is None:
        return jsonify({'error': 'ESP32 no conectado'}), 500

    comando = 'R\n' if color == 'rojo' else 'B\n'
    ser.write(comando.encode())
    print(f'[SERIAL] → {comando.strip()}')

    return jsonify({'ok': True, 'color': color})

# ── Main ───────────────────────────────────────────────────────────
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080)
