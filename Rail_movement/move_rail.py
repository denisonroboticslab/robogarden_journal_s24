import RPi.GPIO as GPIO
import time

# Define pins
PULSE_PIN = 3
DIR_PIN = 5

# Define directions
RIGHT = False
LEFT = True

# Initialize variables
stepcount = 0
steprate = 200

# Setup GPIO
GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
GPIO.setup(PULSE_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

def move_motor(dwant, direction):
    global stepcount
    GPIO.output(DIR_PIN, direction)
    NSTEP = (dwant * steprate) // 8
    while stepcount < NSTEP:
        GPIO.output(PULSE_PIN, GPIO.HIGH)
        GPIO.output(PULSE_PIN, GPIO.LOW)
        time.sleep(0.002)  # 2 milliseconds delay
        stepcount += 1

def main():
    move_motor(100, LEFT)

if __name__ == "__main__":
    try:
        main()
    finally:
        GPIO.cleanup()  # Clean up all the GPIO pins to their default state
