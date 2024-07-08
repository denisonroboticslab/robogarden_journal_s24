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
    
    def __init__(self):
        """
        Set up and initialize motors according to MOTOR_LIST
        """
        # Set your list of motors formatted as (<human readable name>, <configured id>, <model>)
        self.motor_list = [('base', 1, self.DYNAMIXEL_MODEL_B), ('shoulder', 2, self.DYNAMIXEL_MODEL_A), ('elbow', 3, self.DYNAMIXEL_MODEL_A)]

        # Define goal positions for each motor
        self.goal_positions = {
            'base': 2534,
            'shoulder': 3764,
            'elbow': 2658,
        }

        # Define home positions for each motor (same as goal positions)
        self.home_positions = self.goal_positions.copy()

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
        # Move each motor to its goal position
        for dxl_name, goal_position in self.goal_positions.items():
            self.motors[dxl_name].set_goal_position(goal_position)
        time.sleep(1.5)  # Adjust the sleep time as needed for motors to reach the target positions

    def go_home(self):
        """
        Move all motors to their home positions
        """
        # Move each motor to its home position
        for dxl_name, home_position in self.home_positions.items():
            self.motors[dxl_name].set_goal_position(home_position)
        time.sleep(1.5)  # Adjust the sleep time as needed for motors to reach the target positions

    def go_there(self, positions):
        """
        Move all motors to specified positions
        :param positions: Dictionary with motor names as keys and target positions as values
        """
        # Move each motor to its specified position
        for dxl_name, position in positions.items():
            self.motors[dxl_name].set_goal_position(position)
        time.sleep(1.5)  # Adjust the sleep time as needed for motors to reach the target positions

if __name__ == '__main__':
    robot = MyRobot()
    robot.test()
    time.sleep(2)  # Adjust the sleep time if needed
    robot.go_home()
    time.sleep(2)  # Adjust the sleep time if needed
    new_positions = {'base': 2000, 'shoulder': 3500, 'elbow': 2000}
    robot.go_there(new_positions)