# mpu6050.py
from machine import I2C
import math

class accel:
    def __init__(self, i2c, addr=0x68):
        self.i2c = i2c
        self.addr = addr
        # Quitar modo sleep
        self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')

    def get_values(self):
        raw_vals = self.i2c.readfrom_mem(self.addr, 0x3B, 14)
        vals = {}
        vals['AcX'] = self._short(raw_vals[0:2]) / 16384.0
        vals['AcY'] = self._short(raw_vals[2:4]) / 16384.0
        vals['AcZ'] = self._short(raw_vals[4:6]) / 16384.0
        vals['Tmp'] = self._short(raw_vals[6:8]) / 340.0 + 36.53
        vals['GyX'] = self._short(raw_vals[8:10]) / 131.0
        vals['GyY'] = self._short(raw_vals[10:12]) / 131.0
        vals['GyZ'] = self._short(raw_vals[12:14]) / 131.0
        return vals

    def _short(self, data):
        value = data[0] << 8 | data[1]
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value
