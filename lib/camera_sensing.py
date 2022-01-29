# camera_sensing
import cv2
try:
    from servo import Servo
    from pwm import PWM
    from pin import Pin
    from adc import ADC
    from filedb import fileDB
    from utils import reset_mcu
    reset_mcu()
    time.sleep(0.01)
except Exception as e:
    print("This computer does not appear to be a PiCar - X system (ezblock is not present ) . Shadowing hardware calls with substitute functions")
    from sim_ezblock import *
from picarx_improved import Picarx
import time
import sys
from sensor_controller import MyController
from camera import Camera

sys.path.append(r'/home/aseem/Downloads/intro2/ROB599/lib')

if __name__ == "__main__":
    # Initialize Camera object
    camera = Camera()
    # Initialize Controller
    controller = MyController()
    # use_video_port=True
    for frame in camera.camera.capture_continuous(camera.rawCapture, format="bgr", use_video_port=True):
        # read image
        img = frame.array
        # detect red in image
        img, img_2, img_3 = camera.color_detect(
            img, 'red')  # Color detection function
        # arrive at decision based on color detection
        decision = camera.decision_function(img, img_2, img_3)
        # run controller with decision output
        controller.steering_control(decision)
        camera.rawCapture.truncate(0)   # Release cache
        k = cv2.waitKey(1) & 0xFF
        # 27 is the ESC key, which means that if you press the ESC key to exit
        if k == 27:
            camera.camera.close()
            break
