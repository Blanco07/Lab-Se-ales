# ===== ADC + MPU6050 @500 Hz, 30 muestras, envío por lotes CSV =====
from machine import I2C, Pin, Timer, ADC
from time import ticks_ms, ticks_diff
import struct, math, sys, gc

# --- Configuración ---
FS = 500
T_MS = 1000 // FS
ADC_PIN = 34
TEST_PIN = 25
N_MAX = 30
BUF_N = 10                # tamaño de lote (envía cada ~20 ms)
GC_MS = 1000

# --- I2C (autodetección de bus y dirección) ---
def mk_i2c(bus, freq=400000):
    return I2C(bus, scl=Pin(22), sda=Pin(21), freq=freq)

i2c = mk_i2c(0, 100000)
found = i2c.scan()
if not found:
    i2c = mk_i2c(1, 100000)
    found = i2c.scan()
if not found:
    raise OSError("I2C vacío: revisa VCC/GND/SCL/SDA")

MPU_ADDR = found[0]

# --- Registros MPU6050 ---
REG_PWR_MGMT_1 = 0x6B
REG_SMPLRT_DIV = 0x19
REG_CONFIG     = 0x1A
REG_GYRO_CFG   = 0x1B
REG_ACC_CFG    = 0x1C
REG_ACC_OUT    = 0x3B

# --- Escalas ---
ACC_SENS = 16384.0       # LSB/g (±2g)
G_TO_MS2 = 9.80665
GYR_SENS = 131.0         # LSB/(°/s) (±250 dps)
DPS_TO_RAD = math.pi/180.0

def w8(reg, val):
    i2c.writeto_mem(MPU_ADDR, reg, bytes([val & 0xFF]))

def rmem(reg, n):
    return i2c.readfrom_mem(MPU_ADDR, reg, n)

def mpu_init():
    w8(REG_PWR_MGMT_1, 0x00)
    w8(REG_CONFIG, 0x01)         # DLPF ~184 Hz
    w8(REG_GYRO_CFG, 0x00)       # ±250 dps
    w8(REG_ACC_CFG, 0x00)        # ±2 g
    w8(REG_SMPLRT_DIV, 0x01)     # 1000/(1+1)=500 Hz

def mpu_read_all():
    b = rmem(REG_ACC_OUT, 14)
    ax, ay, az, tmp, gx, gy, gz = struct.unpack('>hhhhhhh', b)
    ax = (ax/ACC_SENS)*G_TO_MS2
    ay = (ay/ACC_SENS)*G_TO_MS2
    az = (az/ACC_SENS)*G_TO_MS2
    gx = (gx/GYR_SENS)*DPS_TO_RAD
    gy = (gy/GYR_SENS)*DPS_TO_RAD
    gz = (gz/GYR_SENS)*DPS_TO_RAD
    return ax, ay, az, gx, gy, gz

# --- ADC ---
adc = ADC(Pin(ADC_PIN))
adc.atten(ADC.ATTN_11DB)
adc.width(ADC.WIDTH_12BIT)

# --- Timer / flag ---
test = Pin(TEST_PIN, Pin.OUT)
do_sample = False
def isr(_):
    global do_sample
    do_sample = True
    test.value(1 - test.value())

mpu_init()
tim = Timer(0)
tim.init(period=T_MS, mode=Timer.PERIODIC, callback=isr)

# --- CSV + loop ---
print("t_ms,volt_mV,ax_ms2,ay_ms2,az_ms2,gx_rad_s,gy_rad_s,gz_rad_s")

t0 = ticks_ms()
t_gc = t0
buf = []
n = 0

while True:
    if not do_sample:
        continue
    do_sample = False

    raw = adc.read()
    mv  = (raw*3300 + 2047)//4095
    tms = ticks_diff(ticks_ms(), t0)
    ax, ay, az, gx, gy, gz = mpu_read_all()

    # línea CSV (floats con 3 decimales)
    buf.append("{},".format(tms))
    buf.append("{},".format(mv))
    buf.append("{:.3f},{:.3f},{:.3f},".format(ax, ay, az))
    buf.append("{:.3f},{:.3f},{:.3f}\n".format(gx, gy, gz))
    n += 1

    if len(buf) >= 8*BUF_N:
        sys.stdout.write(''.join(buf))
        buf.clear()

    if n >= N_MAX:
        if buf:
            sys.stdout.write(''.join(buf)); buf.clear()
        tim.deinit()
        print("#FIN")
        break

    if ticks_diff(tms, t_gc) >= GC_MS:
        gc.collect()
        t_gc = tms