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
sys.path.append(r'/home/aseem/Downloads/intro2/ROB599/lib')


class MySensor(object):
    '''
    Sensor Class for interfacing with the grayscale sensor
    '''

    def __init__(self, ref=1000):
        '''
        init function
        '''
        self.chn_0 = ADC("A0")
        self.chn_1 = ADC("A1")
        self.chn_2 = ADC("A2")
        self.ref = ref

    def get_grayscale_data(self):
        '''
        Reads the sensor values and returns as a list
        '''
        adc_value_list = []
        adc_value_list.append(self.chn_0.read())
        adc_value_list.append(self.chn_1.read())
        adc_value_list.append(self.chn_2.read())
        return adc_value_list


class MyInterpretor(object):
    def __init__(self, sensitivity=100, polarity=10):
        '''Initialize interpretor object with sensitivity and polarity'''
        self.sensitivity = 100
        self.polarity = 10
        self.magnitude = 0
        self.direction = 0

    def interpret(self, sensor_data):
        '''Function which decides magnitude and direction of steering angle based on grayscale sensor values'''
        left_val = sensor_data[0]
        center_val = sensor_data[1]
        right_val = sensor_data[2]
        # if tape is on left side
        if left_val > self.sensitivity and center_val <= self.sensitivity and right_val <= self.sensitivity:
            self.magnitude = left_val - right_val
            self.direction = -1
        # if tape is on right
        elif left_val > self.sensitivity and center_val <= self.sensitivity and right_val <= self.sensitivity:
            self.magnitude = right_val - left_val
            self.direction = 1
        # if tape in center
        else:
            self.magnitude = 0
            self.direction = 0
        if self.magnitude <= self.polarity:
            self.magnitude = 0
            self.direction = 0


class MyController(object):
    def __init__(self, scaling_factor=10):
        '''initialize controller with picar object and scaling factor'''
        self.px = Picarx()
        self.scaling_factor = scaling_factor

    def steering_control(self, interpretor_output):
        '''steer car in desired direction'''
        self.px.set_dir_servo_angle(interpretor_output)
        self.px.forward(1)


if __name__ == "__main__":
    # Instantiate Sensor, Interpretor and Controller
    sensor_module = MySensor()
    interpretor_module = MyInterpretor()
    controller_module = MyController()
    while True:
        # Read the sensor values
        interpretor_module.interpret(sensor_module.get_grayscale_data())
        # Provide control input for steering
        controller_module.steering_control(
            interpretor_module.magnitude*interpretor_module.direction)
        # sleep for 0.1 sec essentially letting the motor run for that time
        time.sleep(0.1)
        # Stop the motors
        controller_module.px.stop()
