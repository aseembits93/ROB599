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
from camera import Camera

# Tracker function from tracking.py
def run_track(camera_obj, img):
    '''
    I dont know how it works, just copied it from the armpi codebase
    '''
    st = camera_obj.st
    roi = camera_obj.roi
    rect = camera_obj.rect
    get_roi = camera_obj.get_roi
    detect_color = camera_obj.detect_color
    rotation_angle = camera_obj.rotation_angle
    start_pick_up = camera_obj.start_pick_up
    x_dis = camera_obj.x_dis
    y_dis = camera_obj.y_dis
    z_dis = camera_obj.z_dis
    size = (640, 480)
    x_pid = PID.PID(P=0.1, I=0.00, D=0.008)
    y_pid = PID.PID(P=0.00001, I=0, D=0)
    z_pid = PID.PID(P=0.005, I=0, D=0)
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
    area_max = 0
    areaMaxContour = 0
    if not start_pick_up:
        for i in camera_obj.lab_data:
            if i in camera_obj.target_color:
                detect_color = i
                frame_mask = cv2.inRange(frame_lab,
                                         (camera_obj.lab_data[detect_color]['min'][0],
                                             camera_obj.lab_data[detect_color]['min'][1],
                                             camera_obj.lab_data[detect_color]['min'][2]),
                                         (camera_obj.lab_data[detect_color]['max'][0],
                                             camera_obj.lab_data[detect_color]['max'][1],
                                          camera_obj.lab_data[detect_color]['max'][2]))
                opened = cv2.morphologyEx(
                    frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
                closed = cv2.morphologyEx(
                    opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
                contours = cv2.findContours(
                    closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                areaMaxContour, area_max = camera_obj.getAreaMaxContour(
                    contours)
        if area_max > 1000:
            (center_x, center_y), radius = cv2.minEnclosingCircle(areaMaxContour)
            center_x = int(Misc.map(center_x, 0, size[0], 0, img_w))
            center_y = int(Misc.map(center_y, 0, size[1], 0, img_h))
            radius = int(Misc.map(radius, 0, size[0], 0, img_w))
            if radius > 100:
                return img
            rect = cv2.minAreaRect(areaMaxContour)
            box = np.int0(cv2.boxPoints(rect))
            cv2.drawContours(
                img, [box], -1, camera_obj.range_rgb[camera_obj.target_color], 2)
            x_pid.SetPoint = img_w / 2.0
            x_pid.update(center_x)
            dx = x_pid.output
            x_dis += int(dx)

            x_dis = 0 if x_dis < 0 else x_dis
            x_dis = 1000 if x_dis > 1000 else x_dis

            y_pid.SetPoint = 9000
            if abs(area_max - 9000) < 50:
                area_max = 9000
            y_pid.update(area_max)
            dy = y_pid.output
            y_dis += dy
            y_dis = 5.00 if y_dis < 5.00 else y_dis
            y_dis = 10.00 if y_dis > 10.00 else y_dis

            if abs(center_y - img_h/2.0) < 20:
                z_pid.SetPoint = center_y
            else:
                z_pid.SetPoint = img_h / 2.0

            z_pid.update(center_y)
            dy = z_pid.output
            z_dis += dy

            z_dis = 32.00 if z_dis > 32.00 else z_dis
            z_dis = 10.00 if z_dis < 10.00 else z_dis

            target = camera_obj.AK.setPitchRange(
                (0, round(y_dis, 2), round(z_dis, 2)), -90, 0)

            if target:
                servo_data = target[0]
                if st:
                    Board.setBusServoPulse(3, servo_data['servo3'], 1000)
                    Board.setBusServoPulse(4, servo_data['servo4'], 1000)
                    Board.setBusServoPulse(5, servo_data['servo5'], 1000)
                    Board.setBusServoPulse(6, int(x_dis), 1000)
                    time.sleep(1)
                    st = False
                else:
                    Board.setBusServoPulse(3, servo_data['servo3'], 20)
                    Board.setBusServoPulse(4, servo_data['servo4'], 20)
                    Board.setBusServoPulse(5, servo_data['servo5'], 20)
                    Board.setBusServoPulse(6, int(x_dis), 20)
                    time.sleep(0.03)

    return img

#Function for detecting AprilTags
def apriltagDetect(camera_obj, img):
    tag1 = camera_obj.tag1
    tag2 = camera_obj.tag2
    tag3 = camera_obj.tag3
    detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray, return_image=False)

    tag1 = ['tag1', -1, -1, -1, 0]
    tag2 = ['tag2', -1, -1, -1, 0]
    tag3 = ['tag3', -1, -1, -1, 0]
    if len(detections) != 0:
        for i, detection in enumerate(detections):
            corners = np.rint(detection.corners)
            cv2.drawContours(
                img, [np.array(corners, np.int)], -1, (0, 255, 255), 2)

            tag_family = str(detection.tag_family, encoding='utf-8')
            tag_id = int(detection.tag_id)  # Get tag_id

            object_center_x, object_center_y = int(
                detection.center[0]), int(detection.center[1])
            object_angle = int(math.degrees(math.atan2(
                corners[0][1] - corners[1][1], corners[0][0] - corners[1][0])))

            cv2.putText(img, str(tag_id), (object_center_x - 10, object_center_y + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, [0, 255, 255], 2)

            if tag_id == 1:
                tag1 = ['tag1', object_center_x, object_center_y, object_angle]
            elif tag_id == 2:
                tag2 = ['tag2', object_center_x, object_center_y, object_angle]
            elif tag_id == 3:
                tag3 = ['tag3', object_center_x, object_center_y, object_angle]


# Get region of interest 
def getROI(camera_obj, rotation_angle):
    '''
    I dont know how this function works
    '''
    rotate1 = cv2.getRotationMatrix2D(
        (camera_obj.rows*0.5, camera_obj.cols*0.5), int(rotation_angle), 1)
    rotate_rotate1 = cv2.warpAffine(mask2, rotate1, (camera_obj.cols, camera_obj.rows))
    mask_and = cv2.bitwise_and(rotate_rotate1, mask1)
    rotate2 = cv2.getRotationMatrix2D(
        (camera_obj.rows*0.5, camera_obj.cols*0.5), int(-rotation_angle), 1)
    rotate_rotate2 = cv2.warpAffine(mask_and, rotate2, (camera_obj.cols, camera_obj.rows))
    frame_resize = cv2.resize(
        rotate_rotate2, (710, 710), interpolation=cv2.INTER_NEAREST)
    roi = frame_resize[40:280, 184:504]
    return roi

#function for sorting blocks based on color
def color_sort(camera_obj, img, target):
    '''
    I dont know how this function works
    '''
    size = (320, 240)
    last_x = 0
    last_y = 0
    state = None
    x_adjust = 0
    pick_color = ''

    X = camera_obj.X
    Y = camera_obj.Y
    count = camera_obj.count
    state = camera_obj.state
    adjust = camera_obj.adjust
    approach = camera_obj.approach
    x_adjust = camera_obj.x_adjust
    pick_color = camera_obj.pick_color
    current_tag = camera_obj.current_tag
    adjust_error = camera_obj.adjust_error
    x_dis = camera_obj.x_dis
    y_dis = camera_obj.y_dis
    detect_color = camera_obj.detect_color
    count_timeout = camera_obj.count_timeout
    rotation_angle = camera_obj.rotation_angle
    box_rotation_angle = camera_obj.box_rotation_angle
    last_x_dis = camera_obj.last_x_dis
    last_y_dis = camera_obj.last_y_dis
    last_box_rotation_angle = camera_obj.last_box_rotation_angle
    last_x = camera_obj.last_x
    last_y = camera_obj.last_y
    count_d = camera_obj.count_d
    start_move = camera_obj.start_move

    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gray = cv2.cvtColor(frame_resize, cv2.COLOR_BGR2GRAY)
    frame_lab = cv2.cvtColor(frame_resize, cv2.COLOR_BGR2LAB)

    max_area = 0
    color_area_max = None
    areaMaxContour_max = 0
    roi = getROI(rotation_angle)
    for i in camera_obj.color_range:
        if i in target:
            if i in detect_color:
                target_color_range = camera_obj.color_range[i]
                frame_mask1 = cv2.inRange(frame_lab, tuple(
                    target_color_range['min']), tuple(target_color_range['max']))

                frame_mask2 = cv2.bitwise_and(roi, frame_mask1)
                eroded = cv2.erode(
                    frame_mask2, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                dilated = cv2.dilate(
                    eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

                contours = cv2.findContours(
                    dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                areaMaxContour, area_max = camera_obj.getAreaMaxContour(
                    contours)
                if areaMaxContour is not None:
                    if area_max > max_area and area_max > 100:
                        max_area = area_max
                        color_area_max = i
                        areaMaxContour_max = areaMaxContour
    if max_area > 100:
        rect = cv2.minAreaRect(areaMaxContour_max)
        box_rotation_angle = rect[2]
        if box_rotation_angle > 45:
            box_rotation_angle = box_rotation_angle - 90

        box = np.int0(cv2.boxPoints(rect))
        for j in range(4):
            box[j, 0] = int(Misc.map(box[j, 0], 0, size[0], 0, img_w))
            box[j, 1] = int(Misc.map(box[j, 1], 0, size[1], 0, img_h))

        cv2.drawContours(
            img, [box], -1, camera_obj.range_rgb[color_area_max], 2)

        centerX = int(Misc.map(((areaMaxContour_max[areaMaxContour_max[:, :, 0].argmin()][0])[
                      0] + (areaMaxContour_max[areaMaxContour_max[:, :, 0].argmax()][0])[0])/2, 0, size[0], 0, img_w))
        centerY = int(Misc.map((areaMaxContour_max[areaMaxContour_max[:, :, 1].argmin()][0])[
                      1], 0, size[1], 0, img_h))

        if abs(centerX - last_x) <= 5 and abs(centerY - last_y) <= 5 and not start_move:
            count_d += 1
            if count_d > 5:
                count_d = 0
                start_move = True

                led = Led()
                led.index = 0
                led.rgb.r = camera_obj.range_rgb[color_area_max][2]
                led.rgb.g = camera_obj.range_rgb[color_area_max][1]
                led.rgb.b = camera_obj.range_rgb[color_area_max][0]
                camera_obj.rgb_pub.publish(led)
                led.index = 1
                camera_obj.rgb_pub.publish(led)
                time.sleep(0.1)

                if 298 + camera_obj.d_color_map < centerY <= 424 + camera_obj.d_color_map:
                    Y = Misc.map(centerY, 298 + camera_obj.d_color_map,
                                 424 + camera_obj.d_color_map, 0.12, 0.12 - 0.04)
                elif 198 + camera_obj.d_color_map < centerY <= 298 + camera_obj.d_color_map:
                    Y = Misc.map(centerY, 198 + camera_obj.d_color_map,
                                 298 + camera_obj.d_color_map, 0.12 + 0.04, 0.12)
                elif 114 + camera_obj.d_color_map < centerY <= 198 + camera_obj.d_color_map:
                    Y = Misc.map(centerY, 114 + camera_obj.d_color_map,
                                 198 + camera_obj.d_color_map, 0.12 + 0.08, 0.12 + 0.04)
                elif 50 + camera_obj.d_color_map < centerY <= 114 + camera_obj.d_color_map:
                    Y = Misc.map(centerY, 50 + camera_obj.d_color_map, 114 +
                                 camera_obj.d_color_map, 0.12 + 0.12, 0.12 + 0.08)
                elif 0 + camera_obj.d_color_map < centerY <= 50 + camera_obj.d_color_map:
                    Y = Misc.map(centerY, 0 + camera_obj.d_color_map, 50 +
                                 camera_obj.d_color_map, 0.12 + 0.16, 0.12 + 0.12)
                else:
                    Y = 1
        else:
            count_d = 0

        last_x = centerX
        last_y = centerY
        if (not approach or adjust) and start_move:
            detect_color = (color_area_max, )
            camera_obj.x_pid.SetPoint = camera_obj.center_x
            camera_obj.x_pid.update(centerX)
            dx = camera_obj.x_pid.output
            x_dis += dx

            x_dis = 0 if x_dis < 0 else x_dis
            x_dis = 1000 if x_dis > 1000 else x_dis

            if adjust:
                camera_obj.y_pid.SetPoint = camera_obj.color_y_adjust
                start_move = True
                centerY += abs(Misc.map(70*math.sin(math.pi/4)/2, 0, size[0], 0, img_w)*math.sin(
                    math.radians(abs(camera_obj.gripper_rotation) + 45))) + 65*math.sin(math.radians(abs(camera_obj.roll_angle)))
                if Y < 0.12 + 0.04:
                    centerY += camera_obj.d_color_y
                if 0 < centerY - camera_obj.color_y_adjust <= 5:
                    centerY = camera_obj.color_y_adjust
                camera_obj.y_pid.update(centerY)

                dy = camera_obj.y_pid.output
                y_dis += dy
                y_dis = 0.1 if y_dis > 0.1 else y_dis
                y_dis = -0.1 if y_dis < -0.1 else y_dis
            else:
                dy = 0
            if abs(dx) < 0.1 and abs(dy) < 0.0001 and (abs(last_box_rotation_angle - rect[2]) <= 10 or abs(last_box_rotation_angle - rect[2] >= 80)):
                count += 1
                time.sleep(0.01)
                if (adjust and count > 10) or (not adjust and count >= 10):
                    count = 0
                    if adjust:
                        adjust = False
                    else:
                        rotation_angle = 240 * (x_dis - 500)/1000.0
                        X = round(-Y * math.tan(math.radians(rotation_angle)), 4)
                        state = 'color'
                        pick_color = detect_color[0]
                        adjust_error = False
                        approach = True
            else:
                count = 0

            if adjust and (abs(last_x_dis - x_dis) >= 2 or abs(last_y_dis - y_dis) > 0.002):
                position = camera_obj.grasps.grasp_pos.position
                rotation = camera_obj.grasps.grasp_pos.rotation
                target = ik.setPitchRanges(
                    (position.x, position.y + y_dis, position.z), rotation.r, -180, 0)
                if target:
                    servo_data = target[1]
                    bus_servo_control.set_servos(camera_obj.joints_pub, 100, ((
                        3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, int(x_dis))))
                    time.sleep(0.1)
                    last_x_dis = x_dis
                    last_y_dis = y_dis
                else:
                    bus_servo_control.set_servos(
                        camera_obj.joints_pub, 20, ((6, int(x_dis)), ))
            else:
                bus_servo_control.set_servos(
                    camera_obj.joints_pub, 20, ((6, int(x_dis)), ))

            last_box_rotation_angle = rect[2]
    else:
        count_timeout += 1
        if count_timeout > 20:
            adjust_error = True
            count_timeout = 0
            current_tag = ['tag1', 'tag2', 'tag3']
            detect_color = camera_obj.__target_data[0]
    return img


def tag_sort(camera_obj, img, target):
        '''
    I dont know how this function works
    '''
    camera_obj.d_map = 0.015
    camera_obj.tag_map = [425, 384, 346, 310, 272,
                          239, 208, 177, 153, 129, 106, 86, 68, 51]
    global X, Y
    global state
    global count2
    global count3
    global adjust
    global approach
    global start_move
    global current_tag
    global adjust_error
    global last_X, last_Y
    global box_rotation_angle
    global tag_x_dis, tag_y_dis

    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    centerX = target[1]
    centerY = target[2]

    box_rotation_angle = abs(target[3])
    if box_rotation_angle > 90:
        box_rotation_angle -= 90
    if box_rotation_angle > 45:
        box_rotation_angle = box_rotation_angle - 90
    if target[3] < 0:
        box_rotation_angle = -box_rotation_angle

    distance = math.sqrt(pow(centerX - last_X, 2) + pow(centerY - last_Y, 2))
    if distance < 5 and not start_move:
        count2 += 1
        if count2 > 20:
            count2 = 0
            start_move = True
    else:
        count2 = 0
    if (not approach or adjust) and start_move:
        camera_obj.tag_x_pid.SetPoint = camera_obj.center_x
        camera_obj.tag_x_pid.update(centerX)
        dx = camera_obj.tag_x_pid.output
        tag_x_dis += dx
        tag_x_dis = 0 if tag_x_dis < 0 else tag_x_dis
        tag_x_dis = 1000 if tag_x_dis > 1000 else tag_x_dis

        if abs(centerX - last_X) <= 1 and X != -1:
            count3 += 1
            time.sleep(0.01)
            if count3 > 30:
                count3 = 0
                if adjust:
                    adjust = False
                else:
                    current_tag = target[0]

                    if camera_obj.tag_map[1] + camera_obj.d_tag_map < centerY <= camera_obj.tag_map[0] + camera_obj.d_tag_map:
                        Y = Misc.map(centerY, camera_obj.tag_map[1] + camera_obj.d_tag_map,
                                     camera_obj.tag_map[0] + camera_obj.d_tag_map, 0.12 + d_map, 0.12) - 0.005
                    elif camera_obj.tag_map[2] + camera_obj.d_tag_map < centerY <= camera_obj.tag_map[1] + camera_obj.d_tag_map:
                        Y = Misc.map(centerY, camera_obj.tag_map[2] + camera_obj.d_tag_map,
                                     camera_obj.tag_map[1] + camera_obj.d_tag_map, 0.12 + 2*d_map, 0.12 + d_map)
                    elif camera_obj.tag_map[3] + camera_obj.d_tag_map < centerY <= camera_obj.tag_map[2] + camera_obj.d_tag_map:
                        Y = Misc.map(centerY, camera_obj.tag_map[3] + camera_obj.d_tag_map,
                                     camera_obj.tag_map[2] + camera_obj.d_tag_map, 0.12 + 3*d_map, 0.12 + 2*d_map)
                    elif camera_obj.tag_map[4] + camera_obj.d_tag_map < centerY <= camera_obj.tag_map[3] + camera_obj.d_tag_map:
                        Y = Misc.map(centerY, camera_obj.tag_map[4] + camera_obj.d_tag_map,
                                     camera_obj.tag_map[3] + camera_obj.d_tag_map, 0.12 + 4*d_map, 0.12 + 3*d_map)
                    elif camera_obj.tag_map[5] + camera_obj.d_tag_map < centerY <= camera_obj.tag_map[4] + camera_obj.d_tag_map:
                        Y = Misc.map(centerY, camera_obj.tag_map[5] + camera_obj.d_tag_map,
                                     camera_obj.tag_map[4] + camera_obj.d_tag_map, 0.12 + 5*d_map, 0.12 + 4*d_map)
                    elif camera_obj.tag_map[6] + camera_obj.d_tag_map < centerY <= camera_obj.tag_map[5] + camera_obj.d_tag_map:
                        Y = Misc.map(centerY, camera_obj.tag_map[6] + camera_obj.d_tag_map,
                                     camera_obj.tag_map[5] + camera_obj.d_tag_map, 0.12 + 6*d_map, 0.12 + 5*d_map)
                    elif camera_obj.tag_map[7] + camera_obj.d_tag_map < centerY <= camera_obj.tag_map[6] + camera_obj.d_tag_map:
                        Y = Misc.map(centerY, camera_obj.tag_map[7] + camera_obj.d_tag_map,
                                     camera_obj.tag_map[6] + camera_obj.d_tag_map, 0.12 + 7*d_map, 0.12 + 6*d_map)
                    elif camera_obj.tag_map[8] + camera_obj.d_tag_map < centerY <= camera_obj.tag_map[7] + camera_obj.d_tag_map:
                        Y = Misc.map(centerY, camera_obj.tag_map[8] + camera_obj.d_tag_map,
                                     camera_obj.tag_map[7] + camera_obj.d_tag_map, 0.12 + 8*d_map, 0.12 + 7*d_map)
                    elif camera_obj.tag_map[9] + camera_obj.d_tag_map < centerY <= camera_obj.tag_map[8] + camera_obj.d_tag_map:
                        Y = Misc.map(centerY, camera_obj.tag_map[9] + camera_obj.d_tag_map,
                                     camera_obj.tag_map[8] + camera_obj.d_tag_map, 0.12 + 9*d_map, 0.12 + 8*d_map)
                    elif camera_obj.tag_map[10] + camera_obj.d_tag_map < centerY <= camera_obj.tag_map[9] + camera_obj.d_tag_map:
                        Y = Misc.map(centerY, camera_obj.tag_map[10] + camera_obj.d_tag_map,
                                     camera_obj.tag_map[9] + camera_obj.d_tag_map, 0.12 + 10*d_map, 0.12 + 9*d_map)
                    elif camera_obj.tag_map[11] + camera_obj.d_tag_map < centerY <= camera_obj.tag_map[10] + camera_obj.d_tag_map:
                        Y = Misc.map(centerY, camera_obj.tag_map[11] + camera_obj.d_tag_map,
                                     camera_obj.tag_map[10] + camera_obj.d_tag_map, 0.12 + 11*d_map, 0.12 + 10*d_map)
                    elif camera_obj.tag_map[12] + camera_obj.d_tag_map < centerY <= camera_obj.tag_map[11] + camera_obj.d_tag_map:
                        Y = Misc.map(centerY, camera_obj.tag_map[12] + camera_obj.d_tag_map,
                                     camera_obj.tag_map[11] + camera_obj.d_tag_map, 0.12 + 12*d_map, 0.12 + 11*d_map)
                    elif camera_obj.tag_map[13] + camera_obj.d_tag_map < centerY <= camera_obj.tag_map[12] + camera_obj.d_tag_map:
                        Y = Misc.map(centerY, camera_obj.tag_map[13] + camera_obj.d_tag_map,
                                     camera_obj.tag_map[12] + camera_obj.d_tag_map, 0.12 + 13*d_map, 0.12 + 12*d_map)
                    else:
                        Y = 1

                    X = round(-Y * math.tan(math.radians(camera_obj.rotation_angle)), 4)
                    state = 'tag'
                    approach = True
                    adjust_error = False
                    adjust = False
        else:
            count3 = 0

            bus_servo_control.set_servos(
                camera_obj.joints_pub, 20, ((6, int(tag_x_dis)), ))
        last_X, last_Y = centerX, centerY

        return img


if __name__ == '__main__':
    camera_obj = Camera()
    camera_obj.initMove()
    camera_obj.target_color = ('blue')
    cap = cv2.VideoCapture(0)
    while True:
        ret, img = cap.read()
        if ret:
            frame = img.copy()
            Frame = camera_obj.run_track(frame)
            frame_resize = cv2.resize(Frame, (320, 240))
            cv2.imshow('frame', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    camera_obj.camera_close()
    cv2.destroyAllWindows()
