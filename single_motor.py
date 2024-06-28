import time
from dynamixel_python import DynamixelManager

USB_PORT = 'COM3'
BAUD_RATE = 1000000
DYNAMIXEL_MODEL = 'mx-28-2'
ID = 1

def single_motor_example():
    """
    turn on a single dynamixel and sweep it between position 0 and position 1024 three times
    """
    motors = DynamixelManager(USB_PORT, baud_rate=BAUD_RATE)
    testMotor = motors.add_dynamixel('TestMotor', ID, DYNAMIXEL_MODEL)
    motors.init()


    if not testMotor.ping():
        raise BaseException('motor not configured correctly')

    testMotor.set_operating_mode(3)   
    testMotor.set_led(True)
    testMotor.set_torque_enable(True)
    testMotor.set_profile_velocity(2500)

    for i in range(10):
        testMotor.set_goal_position(0)
        print("moving")
        time.sleep(2)
        
        testMotor.set_goal_position(600)
        print("moving")
        time.sleep(2)

    testMotor.set_torque_enable(False)
    testMotor.set_led(False)


if __name__ == '__main__':
    single_motor_example()
