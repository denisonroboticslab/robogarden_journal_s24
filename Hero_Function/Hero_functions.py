import time
import numpy as np
from dynamixel_python import DynamixelManager
import pdb
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

    # Define constants for rail movement direction
    RIGHT = 0
    LEFT = 1

    def __init__(self):
        """
        Set up and initialize motors according to MOTOR_LIST
        """
        # Set your list of motors formatted as (<human readable name>, <configured id>, <model>)
        self.motor_list = [('base', 1, self.DYNAMIXEL_MODEL_B), ('elbow', 2, self.DYNAMIXEL_MODEL_A), ('shoulder', 3, self.DYNAMIXEL_MODEL_A)]

        # Define goal positions for each motor
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
    # Initialize the Arduino serial connection
        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Update with the correct port
            time.sleep(2)  # Wait for the serial connection to initialize
        except serial.SerialException as e:
            print(f"Error initializing Arduino connection: {e}")
            self.arduino = None

    def test(self):
        """
        Test all motors by turning on their LED and sweeping their position
        """
        
        # Set operating mode to position control
        self.motors.for_all(lambda motor: motor.set_operating_mode(3))

        for motor in self.motors:
            # Sequentially turn on LEDs
            motor.set_led(True)
            time.sleep(0.2)
        # Enable motor control
        self.motors.enable_all()
        #self.disable_motor('elbow')
        self.motors.for_all(lambda motor: motor.set_profile_velocity(2500))

    def go_home(self):
        """
        Move all motors to their home positions
        """
        # Move each motor to its home position
        for dxl_name, home_position in self.HOME_POSITIONS.items():
            self.motors[dxl_name].set_goal_position(home_position)
        time.sleep(1.5)  # Adjust the sleep time as needed for motors to reach the target positions

    def set_joint_angles(self, positions):
        """
        Move all motors to specified positions
        :param positions: Dictionary with motor names as keys and target positions as values
        """
        for dxl_name, position in positions.items():  # positions are in degrees
            if dxl_name == 'base':
                steps = self.deg2stepsbase(position)
            elif dxl_name == 'elbow':
                steps = self.deg2stepelbow(position)
            elif dxl_name == 'shoulder':
                steps = self.deg2stepshoulder(position)
            else:
                continue

            self.motors[dxl_name].set_goal_position(steps)
        time.sleep(1.5)  # Adjust the sleep time as needed for motors to reach the target positions
    
    def deg2steps(self, angle_degrees, resolution=4096):
        """
        Converts angle_degrees to steps in resolution.
        
        :param angle_degrees: The angle in degrees to convert.
        :param resolution: The resolution of the motor, default is 4096.
        :return: The equivalent steps for the given angle.
        """
        # Convert angle degrees to steps
        steps = int((angle_degrees / 360.0) * resolution)
        return steps

    def deg2stepsbase(self, angle_degrees_base, resolution=4096):
        """
        Converts base angle to steps.
        
        :param angle_degrees_base: The base angle in degrees to convert.
        :param resolution: The resolution of the motor, default is 4096.
        :return: The equivalent steps for the given angle.
        """
        steps_base = self.deg2steps(angle_degrees_base)
        result_base = (steps_base + self.OFFSET_POSITIONS["base"]) % resolution
        thresholded_pos_base = self.trim_angle(result_base, "base")
        return thresholded_pos_base

    def deg2stepelbow(self, angle_degrees_elbow, resolution=4096):
        """
        Converts elbow angle to steps.
        
        :param angle_degrees_elbow: The elbow angle in degrees to convert.
        :param resolution: The resolution of the motor, default is 4096.
        :return: The equivalent steps for the given angle.
        """
        steps_elbow = self.deg2steps(angle_degrees_elbow)
        result_elbow = (self.OFFSET_POSITIONS["elbow"] - steps_elbow) % resolution
        thresholded_pos_elbow = self.trim_angle(result_elbow, "elbow")
        return thresholded_pos_elbow

    def deg2stepshoulder(self, angle_degrees_shoulder, resolution=4096):
        """
        Converts shoulder angle to steps.
        
        :param angle_degrees_shoulder: The shoulder angle in degrees to convert.
        :param resolution: The resolution of the motor, default is 4096.
        :return: The equivalent steps for the given angle.
        """
        steps_shoulder = self.deg2steps(angle_degrees_shoulder)
        result_shoulder = (self.OFFSET_POSITIONS["shoulder"] - steps_shoulder) % resolution
        thresholded_pos_shoulder = self.trim_angle(result_shoulder, "shoulder")
        return thresholded_pos_shoulder

    def disable_motor(self, motor_name):
        if motor_name in self.motors.dxl_dict:
            self.motors[motor_name].set_torque_enable(False)
        else:
            # terminate here to prevent damage if motor failed to disable
            raise ValueError(f"Failed to disable motor {motor_name}")

    def enable_motor(self, motor_name):
        if motor_name in self.motors.dxl_dict:
            self.motors[motor_name].set_torque_enable(True)
        else:
            # terminate here to prevent damage if motor failed to enable
            raise ValueError(f"Failed to enable motor {motor_name}")

    def trim_angle(self, angle, joint):
        """
        Return the angle thresholded to the joint limits.
        
        :param angle: The angle in steps to threshold.
        :param joint: The joint name to apply limits.
        :return: The thresholded angle within joint limits.
        """
        min_limit, max_limit = self.JOINT_LIMITS[joint]
        if angle < min_limit:
            return min_limit
        elif angle > max_limit:
            return max_limit
        else:
            return angle
    
    def planner_ik(self, x, z):
        """
        Inverse kinematics to find the angles α0 and α1 based on end-effector position (x, y)
        :param x: X position of the end-effector
        :param y: Y position of the end-effector
        :return: Tuple of angles (α0, α1) in degrees
        """
        print(f"Running planner_ik with end-effector position: ({x}, {z})")

        # Getting the values of a and b

        a = x - self.shoulder_horiz_Offset - self.L4 # a is the distannce from the servo center to the wrist joint

        b = z - self.shoulder_vert_Offset # a is the vertical comp from the servo center to the wrist joint

        # Calculate hypotenuse r
        r = np.sqrt(a**2 + b**2)
        print(f"Hypotenuse (r): {r}")

        # Calculate angle phi using the alternate angle
        phi = np.arctan2(b, a)
        print(f"Phi: {np.degrees(phi)} degrees")

        # Calculate the angle between L5 and r using the law of cosines
        cos_angle_L5_r = (self.L5**2 + r**2 - self.L7**2) / (2 * self.L5 * r)
        angle_L5_r = np.arccos(cos_angle_L5_r) + phi
        print(f"Angle between L5 and r: {np.degrees(angle_L5_r)} degrees")

        # Calculate α0
        alpha_0 = np.degrees(np.pi/2 - angle_L5_r)
        print(f"α0: {alpha_0} degrees")

        # Calculate the anglebetween L5 and L7 using the law of cosines
        cos_angle_L5_L7 = (self.L5**2 + self.L7**2 - r**2) / (2 * self.L5 * self.L7)
        angle_L5_L7 = np.arccos(cos_angle_L5_L7)
        print(f"Angle between L5 and L7: {np.degrees(angle_L5_L7)} degrees")

        # Calculate the angle between L6 and L5
        angle_L6_L5 = np.pi - angle_L5_L7
        print(f"Angle between L6 and L5: {np.degrees(angle_L6_L5)} degrees")

        # Calculate Db using the law of cosines
        Db = np.sqrt(self.L5**2 + self.L6**2 - 2 * self.L5 * self.L6 * np.cos(angle_L6_L5))
        print(f"Db: {Db}")

        # Calculate q using the law of cosines
        cos_q = (Db**2 + self.L5**2 - self.L6**2) / (2 * self.L5 * Db)
        q = np.degrees(np.arccos(cos_q))
        print(f"q: {np.degrees(q)} degrees")

        # Calculate w using the law of cosines
        cos_w = (Db**2 + self.L1**2 - self.L2**2) / (2 * self.L1 * Db)
        w = np.degrees(np.arccos(cos_w))
        print(f"w: {np.degrees(w)} degrees")

        # Calculate v that is alpha0 - q  
        v = alpha_0 - q
        print(f"v: {v} degrees")

        # Calculate α1
        alpha_1 = w - v 
        print(f"α1: {alpha_1} degrees")

        return alpha_0, alpha_1

    def base_ik(self, x, y):
        """
        Inverse kinematics to find the base angle based on end-effector position (x, y)
        :param x: X position of the end-effector
        :param y: Y position of the end-effector
        :return: Base angle in degrees
        """
        print(f"Running base_ik with end-effector position: ({x}, {y})")

        # Calculate base angle using arctan of x and y
        base_angle = np.degrees(np.arctan2(y, x))
        print(f"Base angle: {base_angle} degrees")

        return base_angle

    def move_rail(self, y):
        """
        Move the rail based on the y coordinate.
        :param y: Y position of the end-effector
        """
        if y > 100:
            distance = y - 100  # Calculate the distance to move
            direction = self.RIGHT

        elif y < 100:
            distance = 100 - y  # Calculate the distance to move
            direction = self.LEFT
        else:
            distance = 100 - y
            direction = RIGHT
        
        command = f"MOVE{direction}{distance}\n"
        self.arduino.write(command.encode())
        print(f"Moving rail: {command}")


if __name__ == '__main__':
    robot = MyRobot()
    robot.test()
    time.sleep(2)  # Adjust the sleep time if needed
    robot.go_home()
    time.sleep(2)  # Adjust the sleep time if needed
    
    # Define end-effector position
    x, y, z = 200, 80, 100
    
    # Calculate angles using inverse kinematics
    alpha_0, alpha_1 = robot.planner_ik(x, z)
    print(f"Calculated angles: α0 = {alpha_0} degrees, α1 = {alpha_1} degrees")

    base_angle = robot.base_ik(x, y)

    #robot.move_rail(y)
    
    # Set joint angles based on calculated angles
    new_positions = {
        'base': base_angle,
        'shoulder': alpha_1,
        'elbow': alpha_0
    }
    robot.set_joint_angles(new_positions)