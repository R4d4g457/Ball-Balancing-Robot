import robotKinematics as rk
from imu import MPU6050


class PID:
    def __init__(self, kp, ki, kd, max_out=15.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral = 0.0
        self.prev_err = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_err) / dt if dt > 0 else 0.0
        self.prev_err = error

        out = self.kp * error + self.ki * self.integral + self.kd * derivative

        return max(min(out, self.max_out), -self.max_out)


class TiltController:
    """
    Controller for 3-RRS plate using MPU6050.
    Servo objects come from original RobotController.
    """

    def __init__(self, robot_controller):
        self.robot = robot_controller
        self.s1 = self.robot.s1
        self.s2 = self.robot.s2
        self.s3 = self.robot.s3

        self.pid_x = PID(0.9, 0.0, 0.03)
        self.pid_y = PID(0.9, 0.0, 0.03)

        # Instantiate MPU6050 here
        self.imu = MPU6050()

        # Track last update time
        import time

        self.last_time = time.time()

    def update(self):
        """
        Reads MPU6050 internally, computes tilt correction, applies to servos
        """
        import time

        t = time.time()
        dt = t - self.last_time
        self.last_time = t

        # Read pitch/roll directly from IMU
        pitch, roll = self.imu.read()

        # Compute PID corrections
        corr_x = self.pid_x.update(-pitch, dt)
        corr_y = self.pid_y.update(-roll, dt)

        # Map to servo angles
        theta1, theta2, theta3 = rk.tilt_to_servos(corr_x, corr_y)

        # Apply angles
        self.s1.angle = theta1
        self.s2.angle = theta2
        self.s3.angle = theta3
