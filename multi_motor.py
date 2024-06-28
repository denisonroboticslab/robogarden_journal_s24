import time
from dynamixel_python import DynamixelManager

# Define the models for the different motors
DYNAMIXEL_MODEL_A = 'mx-106-2'
DYNAMIXEL_MODEL_B = 'mx-28-2'
USB_PORT = '/dev/ttyUSB0'
BAUD_RATE = 1000000
# Set your list of motors formatted as (<human readable name>, <configured id>, <model>)
MOTOR_LIST = [('base', 1, DYNAMIXEL_MODEL_B), ('shoulder', 2, DYNAMIXEL_MODEL_A), ('elbow', 3, DYNAMIXEL_MODEL_A)]

# Define goal positions for each motor
GOAL_POSITIONS = {
    'base': 600,
    'shoulder': 4000,
    'elbow': 2500,
}

class MyRobot:
    """
    Example of controlling multiple dynamixels with pynamixel
    """
    def __init__(self):
        """
        Set up and initialize motors according to MOTOR_LIST
        """
        self.motors = DynamixelManager(USB_PORT, baud_rate=BAUD_RATE)
        for dxl_name, dxl_id, dxl_model in MOTOR_LIST:
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
        for dxl_name, goal_position in GOAL_POSITIONS.items():
            self.motors[dxl_name].set_goal_position(goal_position)
        time.sleep(1.5)  # Adjust the sleep time as needed for motors to reach the target positions
        
    

if __name__ == '__main__':
    robot = MyRobot()
    robot.test()
