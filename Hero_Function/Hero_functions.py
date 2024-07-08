import time
from dynamixel_python import DynamixelManager

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

    def __init__(self):
        """
        Set up and initialize motors according to MOTOR_LIST
        """
        # Set your list of motors formatted as (<human readable name>, <configured id>, <model>)
        self.motor_list = [('base', 1, self.DYNAMIXEL_MODEL_B), ('elbow' , 2, self.DYNAMIXEL_MODEL_A), ('shoulder', 3, self.DYNAMIXEL_MODEL_A)]

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
            offset = self.HOME_POSITIONS[dxl_name]
            steps = self.deg2steps(position, offset=offset)
            thresholded_pos = self.trim_angle(steps, dxl_name)
            self.motors[dxl_name].set_goal_position(thresholded_pos)
        time.sleep(1.5)  # Adjust the sleep time as needed for motors to reach the target positions
    
    def deg2steps(self, angle_degrees, resolution=4096, offset=0):
        """
        Converts angle_degrees to steps in resolution.
        
        :param angle_degrees: The angle in degrees to convert.
        :param resolution: The resolution of the motor, default is 4096.
        :param offset: An optional offset to add to the steps, default is 0.
        :return: The equivalent steps for the given angle, considering the resolution and offset.
        """
        # Convert angle degrees to steps
        steps = int((angle_degrees / 360.0) * resolution)
        # Add the offset and wrap around using modulo operation
        result = (steps + offset) % resolution
        return result

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

if __name__ == '__main__':
    robot = MyRobot()
    robot.test()
    time.sleep(2)  # Adjust the sleep time if needed
    robot.go_home()
    time.sleep(2)  # Adjust the sleep time if needed
    new_positions = {'base': 45, 'elbow': 20, 'shoulder': -10}
    robot.set_joint_angles(new_positions)
