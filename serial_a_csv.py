# serial_a_csv.py — Lee del puerto serial y guarda CSV. Se detiene al recibir "#FIN".
import serial, csv

PORT = 'COM3'          # cambia si tu puerto es otro
BAUD = 115200
OUT  = 'datos_capturados.csv'

HEADER = None          # deja None si la ESP32 ya imprime el encabezado (p.ej. "t_ms,volt_mV")

def parece_encabezado(s: str) -> bool:
    return any(c.isalpha() for c in s)

with serial.Serial(PORT, BAUD, timeout=1) as ser, open(OUT, 'w', newline='') as f:
    w = csv.writer(f)
    lineas = 0
    print(f"Grabando desde {PORT} @ {BAUD} → {OUT}  (Ctrl+C para detener)")

    try:
        if HEADER:
            w.writerow([c.strip() for c in HEADER.split(',')])

        while True:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode(errors='ignore').strip()
            if not line:
                continue

            # --- NUEVO: detenerse si llega #FIN ---
            if line.startswith("#FIN"):
                print("Fin recibido desde la ESP32.")
                break

            # Detectar encabezado si el micro lo envía (primera línea con letras)
            if lineas == 0 and HEADER is None and parece_encabezado(line):
                w.writerow([c.strip() for c in line.split(',')])
                lineas += 1
                continue

            # Guardar línea de datos
            w.writerow([c.strip() for c in line.split(',')])
            lineas += 1

            if lineas % 100 == 0:
                f.flush()
                print(f"{lineas} líneas guardadas…")

    except KeyboardInterrupt:
        print("Interrumpido por el usuario.")

print("Listo.")
