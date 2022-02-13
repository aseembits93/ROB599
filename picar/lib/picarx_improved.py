from logdecorator import log_on_start, log_on_end, log_on_error
import logging
import atexit
from datetime import datetime
import time
import sys
# change this path manually
sys.path.append(r'/home/aseem/Downloads/intro2/ROB599/lib')

# Uncomment the following lines if you want to run on pi
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

logging_format = "%(asctime)s:%(message)s"
logging.basicConfig(format=logging_format,
                    level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)
# use the following lines before def function calls
# @log_on_start (logging.DEBUG, "Message when function starts")
# @log_on_error (logging.DEBUG, "Message when function encounters an error before completing")
# @log_on_end (logging.DEBUG, "Message when function ends successfully")


class Picarx(object):
    PERIOD = 4095
    PRESCALER = 10
    TIMEOUT = 0.02

    def __init__(self):
        self.dir_servo_pin = Servo(PWM('P2'))
        self.camera_servo_pin1 = Servo(PWM('P0'))
        self.camera_servo_pin2 = Servo(PWM('P1'))
        self.config_flie = fileDB('/home/pi/.config')
        self.dir_cal_value = int(self.config_flie.get(
            "picarx_dir_servo", default_value=0))
        self.cam_cal_value_1 = int(self.config_flie.get(
            "picarx_cam1_servo", default_value=0))
        self.cam_cal_value_2 = int(self.config_flie.get(
            "picarx_cam2_servo", default_value=0))
        self.dir_servo_pin.angle(self.dir_cal_value)
        self.camera_servo_pin1.angle(self.cam_cal_value_1)
        self.camera_servo_pin2.angle(self.cam_cal_value_2)

        self.left_rear_pwm_pin = PWM("P13")
        self.right_rear_pwm_pin = PWM("P12")
        self.left_rear_dir_pin = Pin("D4")
        self.right_rear_dir_pin = Pin("D5")

        self.S0 = ADC('A0')
        self.S1 = ADC('A1')
        self.S2 = ADC('A2')

        atexit.register(self.cleanup)

        self.motor_direction_pins = [
            self.left_rear_dir_pin, self.right_rear_dir_pin]
        self.motor_speed_pins = [
            self.left_rear_pwm_pin, self.right_rear_pwm_pin]
        self.cali_dir_value = self.config_flie.get(
            "picarx_dir_motor", default_value="[1,1]")
        self.cali_dir_value = [int(i.strip())
                               for i in self.cali_dir_value.strip("[]").split(",")]
        self.cali_speed_value = [0, 0]
        self.dir_current_angle = 0
        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)

    def cleanup(self):
        self.stop()

    def set_motor_speed(self, motor, speed):
        # global cali_speed_value,cali_dir_value
        motor -= 1
        if speed >= 0:
            direction = 1 * self.cali_dir_value[motor]
        elif speed < 0:
            direction = -1 * self.cali_dir_value[motor]
        speed = abs(speed)
        if speed != 0:
            speed = int(speed / 2) + 50
        speed = speed - self.cali_speed_value[motor]
        if direction < 0:
            self.motor_direction_pins[motor].high()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        else:
            self.motor_direction_pins[motor].low()
            self.motor_speed_pins[motor].pulse_width_percent(speed)

    def motor_speed_calibration(self, value):
        # global cali_speed_value,cali_dir_value
        self.cali_speed_value = value
        if value < 0:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
        else:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0

    def motor_direction_calibration(self, motor, value):
        # 0: positive direction
        # 1:negative direction
        # global cali_dir_value
        motor -= 1
        if value == 1:
            self.cali_dir_value[motor] = -1 * self.cali_dir_value[motor]
        self.config_flie.set("picarx_dir_motor", self.cali_dir_value)

    @log_on_start(logging.DEBUG, "Message when function starts")
    @log_on_error(logging.DEBUG, "Message when function encounters an error before completing")
    @log_on_end(logging.DEBUG, "Message when function ends successfully")
    def dir_servo_angle_calibration(self, value):
        # global dir_cal_value
        self.dir_cal_value = value
        # print("calibrationdir_cal_value:", self.dir_cal_value)
        logging.debug("calibrationdir_cal_value:", self.dir_cal_value)
        self.config_flie.set("picarx_dir_servo", "%s" % value)
        self.dir_servo_pin.angle(value)

    @log_on_start(logging.DEBUG, "Message when function starts")
    @log_on_error(logging.DEBUG, "Message when function encounters an error before completing")
    @log_on_end(logging.DEBUG, "Message when function ends successfully")
    def set_dir_servo_angle(self, value):
        # global dir_cal_value
        self.dir_current_angle = value
        angle_value = value + self.dir_cal_value
        logging.debug("angle_value:%f", angle_value)
        logging.debug("set_dir_servo_angle_1:%f", angle_value)
        logging.debug("set_dir_servo_angle_2:%f", self.dir_cal_value)
        self.dir_servo_pin.angle(angle_value)

    @log_on_start(logging.DEBUG, "Message when function starts")
    @log_on_error(logging.DEBUG, "Message when function encounters an error before completing")
    @log_on_end(logging.DEBUG, "Message when function ends successfully")
    def camera_servo1_angle_calibration(self, value):
        # global cam_cal_value_1
        self.cam_cal_value_1 = value
        self.config_flie.set("picarx_cam1_servo", "%s" % value)
        logging.debug("cam_cal_value_1:%f", self.cam_cal_value_1)
        self.camera_servo_pin1.angle(value)

    @log_on_start(logging.DEBUG, "Message when function starts")
    @log_on_error(logging.DEBUG, "Message when function encounters an error before completing")
    @log_on_end(logging.DEBUG, "Message when function ends successfully")
    def camera_servo2_angle_calibration(self, value):
        # global cam_cal_value_2
        self.cam_cal_value_2 = value
        self.config_flie.set("picarx_cam2_servo", "%s" % value)
        logging.debug("picarx_cam2_servo:%f", self.cam_cal_value_2)
        self.camera_servo_pin2.angle(value)

    @log_on_start(logging.DEBUG, "Message when function starts")
    @log_on_error(logging.DEBUG, "Message when function encounters an error before completing")
    @log_on_end(logging.DEBUG, "Message when function ends successfully")
    def set_camera_servo1_angle(self, value):
        # global cam_cal_value_1
        self.camera_servo_pin1.angle(-1*(value + -1*self.cam_cal_value_1))
        logging.debug("self.cam_cal_value_1:%f", self.cam_cal_value_1)
        logging.debug((value + self.cam_cal_value_1))

    @log_on_start(logging.DEBUG, "Message when function starts")
    @log_on_error(logging.DEBUG, "Message when function encounters an error before completing")
    @log_on_end(logging.DEBUG, "Message when function ends successfully")
    def set_camera_servo2_angle(self, value):
        # global cam_cal_value_2
        self.camera_servo_pin2.angle(-1*(value + -1*self.cam_cal_value_2))
        logging.debug("self.cam_cal_value_2:%f", self.cam_cal_value_2)
        logging.debug((value + self.cam_cal_value_2))

    def get_adc_value(self):
        adc_value_list = []
        adc_value_list.append(self.S0.read())
        adc_value_list.append(self.S1.read())
        adc_value_list.append(self.S2.read())
        return adc_value_list

    def set_power(self, speed):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)

    def backward(self, speed):
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            # if abs_current_angle >= 0:
            if abs_current_angle > 40:
                abs_current_angle = 40
            # power_scale = (100 - abs_current_angle) / 100.0
            # logging.debug("power_scale:", power_scale)
            # if (current_angle / abs_current_angle) > 0:
            #     self.set_motor_speed(1, -1*speed)
            #     self.set_motor_speed(2, speed * power_scale)
            # else:
            #     self.set_motor_speed(1, -1*speed * power_scale)
            #     self.set_motor_speed(2, speed)
            self.set_motor_speed(1, -1*speed)
            self.set_motor_speed(2, speed)

        else:
            self.set_motor_speed(1, -1*speed)
            self.set_motor_speed(2, speed)

    def forward(self, speed):
        '''
        We need the ratio of the width and length of the
        car to get an accurate estimate of the speed ratio
        speed ratio is defined as 1-(w/l)*tan(theta) where w is the width of the car,
        l is the length of the car and theta is the steering angle
        '''
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            # if abs_current_angle >= 0:
            if abs_current_angle > 40:
                abs_current_angle = 40
            # power_scale = (100 - abs_current_angle) / 100.0
            # logging.debug("power_scale:", power_scale)
            # if (current_angle / abs_current_angle) > 0:
            #     self.set_motor_speed(1, speed)
            #     self.set_motor_speed(2, -1*speed * power_scale)
            # else:
            #     self.set_motor_speed(1, speed * power_scale)
            #     self.set_motor_speed(2, -1*speed)
            
            ratio = 1-1.3*math.tan(current_angle)
            self.set_motor_speed(1, speed)
            self.set_motor_speed(2, -1*ratio*speed)

        else:
            self.set_motor_speed(1, speed)
            self.set_motor_speed(2, -1*speed)

    def stop(self):
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)

    def Get_distance(self):
        timeout = 0.01
        trig = Pin('D8')
        echo = Pin('D9')

        trig.low()
        time.sleep(0.01)
        trig.high()
        time.sleep(0.000015)
        trig.low()
        pulse_end = 0
        pulse_start = 0
        timeout_start = time.time()
        while echo.value() == 0:
            pulse_start = time.time()
            if pulse_start - timeout_start > timeout:
                return -1
        while echo.value() == 1:
            pulse_end = time.time()
            if pulse_end - timeout_start > timeout:
                return -2
        during = pulse_end - pulse_start
        cm = round(during * 340 / 2 * 100, 2)
        logging.debug(cm)
        return cm


if __name__ == "__main__":
    px = Picarx()
    px.forward(50)
    time.sleep(1)
    px.stop()
    # set_dir_servo_angle(0)
    # time.sleep(1)
    # self.set_motor_speed(1, 1)
    # self.set_motor_speed(2, 1)
    # camera_servo_pin.angle(0)
# set_camera_servo1_angle(cam_cal_value_1)
# set_camera_servo2_angle(cam_cal_value_2)
# set_dir_servo_angle(dir_cal_value)

# if __name__ == "__main__":
#     try:
#         # dir_servo_angle_calibration(-10)
#         while 1:
#             test()
#     finally:
#         stop()
