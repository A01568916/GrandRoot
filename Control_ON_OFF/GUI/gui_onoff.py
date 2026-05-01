# gui_onoff.py — Interfaz gráfica: Control On/Off
# Requiere: pip install pyserial matplotlib

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation

# ── Configuración ────────────────────────────────────────────
BUFFER = 150    # Puntos visibles en las gráficas
BAUD   = 115200

# ── Buffers circulares de datos ──────────────────────────────
tiempos     = deque([0]*BUFFER, maxlen=BUFFER)
ref_buf     = deque([0]*BUFFER, maxlen=BUFFER)

pul_izq_buf = deque([0]*BUFFER, maxlen=BUFFER)
err_izq_buf = deque([0]*BUFFER, maxlen=BUFFER)
esf_izq_buf = deque([0]*BUFFER, maxlen=BUFFER)

pul_der_buf = deque([0]*BUFFER, maxlen=BUFFER)
err_der_buf = deque([0]*BUFFER, maxlen=BUFFER)
esf_der_buf = deque([0]*BUFFER, maxlen=BUFFER)

conexion  = None    # Objeto serial
t_actual  = 0.0    # Tiempo acumulado
ref_valor = 0.0    # Referencia compartida entre hilos

# ── Funciones de conexión ────────────────────────────────────
def listar_puertos():
    return [p.device for p in serial.tools.list_ports.comports()]

def toggle_conexion():
    global conexion
    if conexion and conexion.is_open:
        conexion.close()
        conexion = None
        btn_conn.config(text="Conectar")
    else:
        try:
            conexion = serial.Serial(cb_puerto.get(), BAUD, timeout=1)
            btn_conn.config(text="Desconectar")
            threading.Thread(target=leer_serial, daemon=True).start()
        except Exception as e:
            messagebox.showerror("Error de conexión", str(e))

def enviar(event=None):
    global ref_valor
    try:
        ref_valor = float(ent_ref.get())
        if conexion and conexion.is_open:
            conexion.write(f"{ref_valor}\n".encode())  # un solo valor
    except ValueError:
        pass

# ── Hilo de lectura serial (no bloquea la GUI) ───────────────
def leer_serial():
    global t_actual
    while conexion and conexion.is_open:
        try:
            linea  = conexion.readline().decode(errors='ignore').strip()
            partes = linea.split(",")
            if len(partes) == 6:
                pul_izq = float(partes[0])
                err_izq = float(partes[1])
                esf_izq = float(partes[2])
                pul_der = float(partes[3])
                err_der = float(partes[4])
                esf_der = float(partes[5])
                t_actual += 0.1
                tiempos.append(t_actual)
                ref_buf.append(ref_valor)
                pul_izq_buf.append(pul_izq)
                err_izq_buf.append(err_izq)
                esf_izq_buf.append(esf_izq)
                pul_der_buf.append(pul_der)
                err_der_buf.append(err_der)
                esf_der_buf.append(esf_der)
        except Exception:
            pass

# ── Actualización de gráficas (FuncAnimation) ────────────────
def actualizar(frame):
    T   = list(tiempos)
    REF = list(ref_buf)

    ax1.clear(); ax2.clear(); ax3.clear()

    # Gráfica 1: Referencia — pulsos reales vs referencia
    ax1.plot(T, list(pul_izq_buf), 'b',   label='Izquierdo')
    ax1.plot(T, list(pul_der_buf), 'c',   label='Derecho')
    ax1.plot(T, REF,               'r--', label='Referencia')
    ax1.set_title('Referencia')
    ax1.set_ylabel('Pulsos / 100 ms')
    ax1.legend(loc='upper left', fontsize=8)
    ax1.grid(True)

    # Gráfica 2: Error
    ax2.plot(T, list(err_izq_buf), 'b', label='Izquierdo')
    ax2.plot(T, list(err_der_buf), 'c', label='Derecho')
    ax2.set_title('Error')
    ax2.set_ylabel('Pulsos')
    ax2.legend(loc='upper left', fontsize=8)
    ax2.grid(True)

    # Gráfica 3: Esfuerzo
    ax3.plot(T, list(esf_izq_buf), 'b', label='Izquierdo')
    ax3.plot(T, list(esf_der_buf), 'c', label='Derecho')
    ax3.set_title('Esfuerzo')
    ax3.set_ylabel('DAC (0–255)')
    ax3.set_xlabel('Tiempo (s)')
    ax3.legend(loc='upper left', fontsize=8)
    ax3.grid(True)

    fig.tight_layout(pad=1.5)

# ── Ventana principal ────────────────────────────────────────
root = tk.Tk()
root.title("Control On/Off — Motores Izquierdo y Derecho")
root.geometry("1100x800")

FUENTE        = ("Segoe UI", 11)
FUENTE_GRANDE = ("Segoe UI", 16, "bold")

# Panel de parámetros (fondo gris claro para distinguirlo)
panel = tk.Frame(root, pady=12, padx=14, bg="#e8e8e8", relief=tk.RIDGE, bd=1)
panel.pack(side=tk.TOP, fill=tk.X)

# ── Conexión ─────────────────────────────────────────────────
frame_conn = tk.Frame(panel, bg="#e8e8e8")
frame_conn.pack(side=tk.LEFT, padx=(0, 20))

tk.Label(frame_conn, text="Puerto:", font=FUENTE, bg="#e8e8e8").pack(side=tk.LEFT)
cb_puerto = ttk.Combobox(frame_conn, values=listar_puertos(), width=10, font=FUENTE)
cb_puerto.pack(side=tk.LEFT, padx=6)

btn_conn = tk.Button(frame_conn, text="Conectar", width=12, font=FUENTE, command=toggle_conexion)
btn_conn.pack(side=tk.LEFT, padx=6)

# ── Separador visual ─────────────────────────────────────────
tk.Frame(panel, width=2, bg="#aaaaaa").pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=4)

# ── Referencia (prominente) ──────────────────────────────────
frame_ref = tk.Frame(panel, bg="#e8e8e8")
frame_ref.pack(side=tk.LEFT, padx=10)

tk.Label(frame_ref, text="Referencia (pulsos)", font=FUENTE, bg="#e8e8e8").pack(anchor="w")
sub_ref = tk.Frame(frame_ref, bg="#e8e8e8")
sub_ref.pack()

ent_ref = tk.Entry(sub_ref, width=7, font=FUENTE_GRANDE, justify="center",
                   relief=tk.SOLID, bd=2)
ent_ref.insert(0, "0")
ent_ref.pack(side=tk.LEFT, padx=(0, 8), ipady=4)
ent_ref.bind("<Return>", enviar)

tk.Button(sub_ref, text="Enviar", font=FUENTE_GRANDE, command=enviar,
          bg="#4a90d9", fg="white", relief=tk.FLAT, padx=14, pady=4).pack(side=tk.LEFT)

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 7), gridspec_kw={"height_ratios": [2, 1, 1]})
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10, pady=6)

ani = FuncAnimation(fig, actualizar, interval=200, cache_frame_data=False)

root.mainloop()
