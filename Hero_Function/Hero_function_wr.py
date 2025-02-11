import time
import numpy as np
from dynamixel_python import DynamixelManager
import serial

class MyRobot:
    """
    Example of controlling multiple dynamixels with pynamixel
    """
    DYNAMIXEL_MODEL_A = 'mx-106-2'
    DYNAMIXEL_MODEL_B = 'mx-28-2'
    USB_PORT = '/dev/ttyUSB0'
    BAUD_RATE = 1000000
    
    # Define joint limits for each motor
    JOINT_LIMITS = {
        'base': (1751, 3588),
        'elbow': (3000, 4095),
        'shoulder': (1950, 3100),
    }

    # Define home positions for each motor
    HOME_POSITIONS = {
        'base': 2534,
        'elbow': 3764,
        'shoulder': 2658,
    }

    # Define offset positions for each motor
    OFFSET_POSITIONS = {
        'base': 2534,
        'elbow': 3970,
        'shoulder': 3212,
    }

    # Define link lengths
    L1 = 65
    L2 = 155
    L3 = 54.4 + 160
    L4 = 90.52
    L5 = 148.4
    L6 = 54.4
    L7 = 160
    shoulder_horiz_Offset = 18.71
    shoulder_vert_Offset = 45

    y_origin = 0  # current location of the base
    y_threshhold = 100

    def __init__(self):
        """
        Set up and initialize motors according to MOTOR_LIST
        """
        self.motor_list = [('base', 1, self.DYNAMIXEL_MODEL_B), ('elbow', 2, self.DYNAMIXEL_MODEL_A), ('shoulder', 3, self.DYNAMIXEL_MODEL_A)]

        self.goal_positions = {
            'base': 600,
            'elbow': 4000,
            'shoulder': 2500,
        }

        self.motors = DynamixelManager(self.USB_PORT, baud_rate=self.BAUD_RATE)
        for dxl_name, dxl_id, dxl_model in self.motor_list:
            self.motors.add_dynamixel(dxl_name, dxl_id, dxl_model)

        self.motors.init()
        if not self.motors.ping_all():
            raise BaseException("Motors aren't configured correctly")

    def test(self):
        """
        Test all motors by turning on their LED and sweeping their position
        """
        self.motors.for_all(lambda motor: motor.set_operating_mode(3))  # Set operating mode to position control
        for motor in self.motors:
            motor.set_led(True)  # Turn on motor LED
            time.sleep(0.2)

        self.motors.enable_all()  # Enable motor control
        self.motors.for_all(lambda motor: motor.set_profile_velocity(2500))  # Set velocity profile

    def go_home(self):
        """
        Move all motors to their home positions
        """
        for dxl_name, home_position in self.HOME_POSITIONS.items():
            print(f"Moving {dxl_name} to home position: {home_position}")
            self.motors[dxl_name].set_goal_position(home_position)
        time.sleep(2)  # Adjust the sleep time for motors to reach positions

    def set_joint_angles(self, positions):
        """
        Move all motors to specified positions
        :param positions: Dictionary with motor names as keys and target positions as values
        """
        for dxl_name, position in positions.items():
            if dxl_name == 'base':
                steps = self.deg2stepsbase(position)
            elif dxl_name == 'elbow':
                steps = self.deg2stepelbow(position)
            elif dxl_name == 'shoulder':
                steps = self.deg2stepshoulder(position)
            else:
                continue
            print(f"Setting {dxl_name} to {steps} steps")
            self.motors[dxl_name].set_goal_position(steps)
        time.sleep(2)  # Allow motors to move to positions

    def deg2steps(self, angle_degrees, resolution=4096):
        """
        Converts angle_degrees to steps in resolution.
        """
        steps = int((angle_degrees / 360.0) * resolution)
        return steps

    def deg2stepsbase(self, angle_degrees_base, resolution=4096):
        steps_base = self.deg2steps(angle_degrees_base)
        result_base = (steps_base + self.OFFSET_POSITIONS["base"]) % resolution
        return self.trim_angle(result_base, "base")

    def deg2stepelbow(self, angle_degrees_elbow, resolution=4096):
        steps_elbow = self.deg2steps(angle_degrees_elbow)
        result_elbow = (self.OFFSET_POSITIONS["elbow"] - steps_elbow) % resolution
        return self.trim_angle(result_elbow, "elbow")

    def deg2stepshoulder(self, angle_degrees_shoulder, resolution=4096):
        steps_shoulder = self.deg2steps(angle_degrees_shoulder)
        result_shoulder = (self.OFFSET_POSITIONS["shoulder"] - steps_shoulder) % resolution
        return self.trim_angle(result_shoulder, "shoulder")

    def trim_angle(self, angle, joint):
        """
        Return the angle thresholded to the joint limits.
        """
        min_limit, max_limit = self.JOINT_LIMITS[joint]
        return max(min_limit, min(max_limit, angle))

    def planner_ik(self, x, y, z):
        """
        Inverse kinematics to find the angles α0 and α1 based on end-effector position (x, y, z)
        """
        print(f"Calculating IK for position: x={x}, y={y}, z={z}")
        a = np.sqrt(x**2 + y**2) - self.shoulder_horiz_Offset - self.L4
        b = z - self.shoulder_vert_Offset
        r = np.sqrt(a**2 + b**2)
        phi = np.arctan2(b, a)

        cos_angle_L5_r = (self.L5**2 + r**2 - self.L7**2) / (2 * self.L5 * r)
        angle_L5_r = np.arccos(cos_angle_L5_r) + phi
        alpha_0 = np.degrees(np.pi/2 - angle_L5_r)

        cos_angle_L5_L7 = (self.L5**2 + self.L7**2 - r**2) / (2 * self.L5 * self.L7)
        angle_L5_L7 = np.arccos(cos_angle_L5_L7)
        angle_L6_L5 = np.pi - angle_L5_L7

        Db = np.sqrt(self.L5**2 + self.L6**2 - 2 * self.L5 * self.L6 * np.cos(angle_L6_L5))
        cos_q = (Db**2 + self.L5**2 - self.L6**2) / (2 * self.L5 * Db)
        q = np.degrees(np.arccos(cos_q))

        cos_w = (Db**2 + self.L1**2 - self.L2**2) / (2 * self.L1 * Db)
        w = np.degrees(np.arccos(cos_w))

        v = alpha_0 - q
        alpha_1 = w - v

        return alpha_0, alpha_1

    def base_ik(self, x, y):
        """
        Inverse kinematics to find the base angle based on end-effector position (x, y)
        """
        base_angle = np.degrees(np.arctan2(y, x))
        return base_angle

if __name__ == '__main__':
    robot = MyRobot()
    robot.test()
    time.sleep(2)
    
    robot.go_home()
    time.sleep(2)

    x, y, z = 259, 90, 100  # Example end-effector position
    alpha_0, alpha_1 = robot.planner_ik(x, y - robot.y_origin, z)
    print(f"Calculated angles: α0 = {alpha_0}, α1 = {alpha_1}")
    
    base_angle = robot.base_ik(x, y)
    print(f"Base angle: {base_angle} degrees")

    new_positions = {
        'base': base_angle,
        'shoulder': alpha_1,
        'elbow': alpha_0
    }

    print(f"Setting new joint angles: {new_positions}")
    robot.set_joint_angles(new_positions)
