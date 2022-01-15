import time
from picarx_improved import Picarx


if __name__ == "__main__":
    px = Picarx()
    #set direction servo to 0
    px.set_dir_servo_angle(0)
    # more forward with a velocity of 0.2 for 1 second
    px.forward(0.2)
    time.sleep(1)
    #Stop the bot
    px.stop()
    #See now how much the car deviated, for example, it's 2 degrees
    px.motor_direction_calibration(2)

        
