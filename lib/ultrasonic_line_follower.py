#code is only tested on software
from picarx_improved import Picarx
from sim_ezblock import *

from rossros import *
import concurrent.futures

from random import uniform
from time import sleep
import numpy as np

class LineSensor(object):
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

class UltrasonicController:
    """
    ultra simplistic ultra sonic controller
    """

    def __init__(self, thresh=5):
        """
        setting a default threshold of 5 for checking the ultrasonic sensor value
        """
        self.thresh = thresh

    def forward(self, input_val):
        '''return true if value higher than thresh is observed'''
        return input_val > self.thresh

class LineController:
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

#from ultrasonic.py            
class Ultrasonic:
    def __init__(self, trig, echo, timeout=0.02):
        '''I just copied it from the original picar repo, i dont know how it works'''
        self.trig = trig
        self.echo = echo
        self.timeout = timeout

    def _read(self):
        '''I dont really understand how it works'''
        self.trig.low()
        time.sleep(0.01)
        self.trig.high()
        time.sleep(0.00001)
        self.trig.low()
        pulse_end = 0
        pulse_start = 0
        timeout_start = time.time()
        while self.echo.value()==0:
            pulse_start = time.time()
            if pulse_start - timeout_start > self.timeout:
                return -1
        while self.echo.value()==1:
            pulse_end = time.time()
            if pulse_end - timeout_start > self.timeout:
                return -1
        during = pulse_end - pulse_start
        cm = round(during * 340 / 2 * 100, 2)
        return cm

    def read(self, times=10):
        '''No clue about this function'''
        for i in range(times):
            a = self._read()
            if a != -1 or a <= 300:
                return a
        return -1

class LineUltraController:
    """
    Simplistic Controller 
    """

    def __init__(self):
        self.picar = Picarx()

    def forward(self, angle):
        '''Non sensical logic for line and ultrasonic controller'''
        if angle<60:
            self.picar.set_dir_servo_angle(angle)
            self.picar.forward(0.2)
        else:
            print("Stopping Car!")
            self.picar.stop()


def main():
    # Sensor/Controller definitions
    line_sensor = LineSensor()
    line_controller = LineController()
    ultrasonic_sensor = Ultrasonic("D2", "D3")
    ultrasonic_controller = UltrasonicController()
    line_ultra_controller = LineUltraController()

    #timer as mentioned in the course manual
    timer = Timer(timer_bus, 20, 0.01, timer_bus)

    #Bus definitions
    timer_bus = Bus(False, "Timer Bus")
    line_bus = Bus(0.5, "Line Bus")
    ultra_bus = Bus(100, "Ultra Bus")
    angle_bus = Bus(0.0, "Angle Bus")
    
    #Producer definitions
    produce_line = Producer(line_sensor.get_grayscale_data, line_bus, timer_bus, name="Line Sensor")
    produce_ultra = Producer(ultrasonic_sensor.read, ultra_bus, timer_bus, name="Ultra Sensor")

    #Consumer definitions
    consume_line = ConsumerProducer(
        line_controller.interpret, line_bus, angle_bus, 0, timer_bus, name="Line Controller"
    )
    consume_ultra = ConsumerProducer(
        ultrasonic_controller.forward, ultra_bus, 0, timer_bus, name="Ultra Controller"
    )

    run_car = Consumer(
        line_ultra_controller.forward, (angle_bus), 0, timer_bus, name="Car Controller"
    )
    
    #run all consumers and producers concurrently
    runConcurrently(
        [produce_line, consume_line, produce_ultra, consume_ultra, run_car, timer]
    )


if __name__ == "__main__":
    main()
