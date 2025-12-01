import os
import time
from controller import RobotController  # Original class that creates s1, s2, s3
from controller import TiltController  # Updated tilt controller with MPU6050


def main():
    # Instantiate the original robot controller (sets up servos)
    robot = RobotController()

    # Create the tilt controller, passing the robot instance
    debug = os.environ.get("TILT_DEBUG", "").lower() not in ("", "0", "false", "no")
    controller = TiltController(robot, debug=debug)
    if debug:
        print("TiltController debug logging enabled (TILT_DEBUG set).", flush=True)

    dt = 0.01  # Loop period (~100 Hz)
    time.sleep(1)  # Allow IMU to stabilise

    while True:
        # Controller reads MPU6050 internally, computes PID, updates servos
        controller.update()

        # Maintain loop rate
        time.sleep(dt)


if __name__ == "__main__":
    main()
