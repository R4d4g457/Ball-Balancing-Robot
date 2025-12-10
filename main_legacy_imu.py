import math
import os
import time

from controller_legacy import LegacyRobotController
from imu import MPU6050
from robotKinematics_legacy import RobotKinematics


class AxisPID:
    def __init__(self, kp, ki, kd, max_out):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral = 0.0
        self.prev_err = 0.0
        self.prev_time = None

    def update(self, error):
        now = time.time()
        if self.prev_time is None:
            dt = 0.01
        else:
            dt = max(1e-3, now - self.prev_time)
        self.prev_time = now

        self.integral += error * dt
        derivative = (error - self.prev_err) / dt
        self.prev_err = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(min(output, self.max_out), -self.max_out)


def env_float(name, default):
    try:
        return float(os.environ.get(name, default))
    except ValueError:
        return default


def run():
    loop_hz = env_float("LEGACY_LOOP_HZ", 100.0)
    dt = 1.0 / loop_hz
    max_tilt = env_float("LEGACY_MAX_TILT_DEG", 25.0)
    kp = env_float("LEGACY_KP", 0.8)
    ki = env_float("LEGACY_KI", 0.0)
    kd = env_float("LEGACY_KD", 0.03)

    model = RobotKinematics()
    robot = LegacyRobotController(model, model.lp, model.l1, model.l2, model.lb)
    imu = MPU6050()

    pid_x = AxisPID(kp, ki, kd, max_tilt)
    pid_y = AxisPID(kp, ki, kd, max_tilt)

    time.sleep(1.0)
    while True:
        pitch, roll = imu.read()
        corr_x = pid_x.update(-pitch)
        corr_y = pid_y.update(-roll)

        magnitude = math.sqrt(corr_x**2 + corr_y**2)
        theta = min(max_tilt, magnitude)
        phi = (math.degrees(math.atan2(corr_y, corr_x)) + 360.0) % 360.0

        robot.Goto_N_time_spherical(theta, phi, model.h)
        time.sleep(dt)


if __name__ == "__main__":
    run()
