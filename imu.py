import smbus2
import time
import math


class MPU6050:
    def __init__(self, bus_id=1, address=0x68):
        self.bus = smbus2.SMBus(bus_id)
        self.address = address

        # Wake up device
        self.bus.write_byte_data(self.address, 0x6B, 0)

        self.accel_scale = 16384.0
        self.gyro_scale = 131.0

        self.pitch = 0.0
        self.roll = 0.0
        self.last_time = time.time()

    def _read_word(self, reg):
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        val = (high << 8) | low
        if val & 0x8000:
            val = -((65535 - val) + 1)
        return val

    def read(self):
        ax = self._read_word(0x3B) / self.accel_scale
        ay = self._read_word(0x3D) / self.accel_scale
        az = self._read_word(0x3F) / self.accel_scale

        gx = self._read_word(0x43) / self.gyro_scale
        gy = self._read_word(0x45) / self.gyro_scale

        t = time.time()
        dt = t - self.last_time
        self.last_time = t

        pitch_acc = math.degrees(math.atan2(ax, math.sqrt(ay * ay + az * az)))
        roll_acc = math.degrees(math.atan2(ay, math.sqrt(ax * ax + az * az)))

        self.pitch += gy * dt
        self.roll += gx * dt

        alpha = 0.98
        self.pitch = alpha * self.pitch + (1.0 - alpha) * pitch_acc
        self.roll = alpha * self.roll + (1.0 - alpha) * roll_acc

        return self.pitch, self.roll
