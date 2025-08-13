# ===== N20 + HC-020K: velocidad a CSV (30 líneas) =====
from machine import Pin, Timer
from time import ticks_ms, ticks_us, ticks_diff
import sys

# --- Configuración ---
ENC_PIN = 27           # DO del HC-020K
TEST_PIN = 25          # ver tasa de actualización en el osciloscopio
WINDOW_MS = 100        # ventana de cálculo (100 ms -> 10 Hz de actualización)
PULSOS_POR_VUELTA = 20 # <-- AJUSTA según tu rueda/disco
N_MAX = 30             # número de filas a enviar
BUF_N = 10             # tamaño de lote para escribir por bloques
MIN_US = 150           # anti-rebote/ruido (ignora pulsos <150 us)

# --- Pines ---
enc = Pin(ENC_PIN, Pin.IN, Pin.PULL_UP)   # <- constante correcta
test = Pin(TEST_PIN, Pin.OUT)

# --- Estado ---
pulsos = 0
last_us = 0
do_compute = False
t0 = ticks_ms()
t_win0 = t0

def isr_edge(pin):
    global pulsos, last_us
    now = ticks_us()
    if ticks_diff(now, last_us) >= MIN_US:
        pulsos += 1
        last_us = now

def isr_timer(t):
    global do_compute
    do_compute = True
    test.value(1 - test.value())

enc.irq(trigger=Pin.IRQ_RISING, handler=isr_edge)

tim = Timer(0)
tim.init(period=WINDOW_MS, mode=Timer.PERIODIC, callback=isr_timer)

print("t_ms,pulses,rpm,rad_s")   # encabezado CSV

buf = []
lineas = 0

while True:
    if not do_compute:
        continue
    do_compute = False

    c = pulsos
    pulsos = 0

    t_now = ticks_ms()
    dt_ms = ticks_diff(t_now, t_win0)
    t_win0 = t_now
    if dt_ms <= 0:
        continue

    rev_per_s = (c / PULSOS_POR_VUELTA) / (dt_ms / 1000.0)
    rpm = rev_per_s * 60.0
    rad_s = rev_per_s * 6.283185307
    t_elapsed = ticks_diff(t_now, t0)

    buf.append(f"{t_elapsed},{c},{rpm:.2f},{rad_s:.3f}\n")
    lineas += 1

    if len(buf) >= BUF_N:
        sys.stdout.write(''.join(buf))
        buf.clear()

    if lineas >= N_MAX:
        if buf:
            sys.stdout.write(''.join(buf))
            buf.clear()
        tim.deinit()
        print("#FIN")
        break
