import math
import time
import cv2
import HiwonderSDK.Board as Board
import HiwonderSDK.Misc as Misc
import HiwonderSDK.PID as PID
import HiwonderSDK.yaml_handle as yaml_handle
import numpy as np
from ArmIK.ArmMoveIK import *
from ArmIK.Transform import *
from armpi_fpv import PID, Misc, apriltag, bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from kinematics import ik_transform
from object_sorting.srv import *

# Camera class definition from armpi codebase


class Camera:
    '''
    Refactored from Armpi codebase for better readiblilty and modularity
    '''

    def __init__(self):
        self.AK = ArmIK()
        self.target_color = None
        self.range_rgb = {
            'red': (25, 25, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255), }
        self.lab_data = yaml_handle.get_yaml_data(
            "/home/aseem/ArmPi/Sensor/HiwonderSDK/lab_config.yaml")
        self.servo_data = []

        self.x_dis = 500
        self.y_dis = 10
        self.Z_DIS = 18
        self.z_dis = self.Z_DIS
        self.x_pid = PID.PID(P=0.1, I=0.00, D=0.008)
        self.y_pid = PID.PID(P=0.00001, I=0, D=0)
        self.z_pid = PID.PID(P=0.005, I=0, D=0)

        self.get_roi = False
        self.detect_color = 'red'
        self.start_pick_up = False
        self.rect = None
        self.size = (640, 480)
        self.rotation_angle = 0
        self.roi = ()
        self.st = True
        self.x_dis = 500
        self.Y_DIS = 0
        self.y_dis = Y_DIS
        self.last_x_dis = x_dis
        self.last_x_dis = y_dis
        self.x_pid = PID.PID(P=0.01, I=0.001, D=0)
        self.y_pid = PID.PID(P=0.00001, I=0, D=0)
        self.tag_x_dis = 500
        self.tag_y_dis = 0
        self.tag_x_pid = PID.PID(P=0.01, I=0.001, D=0)
        self.tag_y_pid = PID.PID(P=0.02, I=0, D=0)
        self.stop_state = 0
        self.move_state = 1
        self.adjust = False
        self.approach = False
        self.rotation_angle = 0
        self.start_move = False
        self.adjust_error = False
        self.last_X, last_Y = 0, 0
        self.box_rotation_angle = 0
        self.last_box_rotation_angle = 0
        self.tag1 = ['tag1', -1, -1, -1, 0]
        self.tag2 = ['tag2', -1, -1, -1, 0]
        self.tag3 = ['tag3', -1, -1, -1, 0]
        self.current_tag = ['tag1', 'tag2', 'tag3']
        self.detect_color = ('red', 'green', 'blue')
        self.count = 0
        self.count2 = 0
        self.count3 = 0
        self.count_d = 0
        self.count_timeout = 0
        self.count_tag_timeout = 0
        self.count_adjust_timeout = 0

    def initMove(self):

        Board.setPWMServoPulse(1, 500, 800)
        Board.setPWMServoPulse(2, 500, 800)
        self.AK.setPitchRangeMoving(
            (0, self.y_dis, self.z_dis), 0, -90, 0, 1500)
        time.sleep(1.5)

    def getAreaMaxContour(self, contours):
        contour_area_temp = 0
        contour_area_max = 0
        areaMaxContour = None

        for c in contours:

            contour_area_temp = math.fabs(cv2.contourArea(c))
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:
                    areaMaxContour = c

        return areaMaxContour, contour_area_max

    def set_rgb(color):
        if color == "red":
            Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
            Board.RGB.show()
        elif color == "green":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
            Board.RGB.show()
        elif color == "blue":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
            Board.RGB.show()
        else:
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
            Board.RGB.show()
