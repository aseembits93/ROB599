import time
from picarx_improved import Picarx


def move_forward(px, speed, angle, run_time=1):
    '''
    Moves the bot forward for 1 sec
    '''
    px.set_dir_servo_angle(angle)
    time.sleep(0.1)
    px.forward(speed)
    time.sleep(run_time)


def k_turn(px, speed, dir='right', angle=45):
    '''
    EXecutes a k turn with a default turning angle of 45 degrees
    '''
    if dir == 'left':
        angle = -angle
    px.move_forward(speed, 1, 45)
    time.sleep(1)
    px.move_forward(-speed, 1, -45)
    time.sleep(1)
    px.move_forward(speed, 1, 45)
    time.sleep(1)


def parallel_parking(px, speed, dir='right', angle=30):
    '''
    Executes a parallel parking maneuver with a default angle of 30
    '''
    if dir == 'left':
        angle = -angle
    px.move_forward(speed, 1, 0)
    time.sleep(1)
    px.move_forward(-speed, 1, angle)
    time.sleep(1)
    px.move_forward(-speed, 1, -angle)
    time.sleep(1)


if __name__ == "__main__":
    px = Picarx()
    while 1:
        key = input("1: Forward, 2: K turn, 3: Parallel Parking, 4: Exit\n")
        if key == '1':
            speed = input("Speed?\n")
            speed = float(speed)
            angle = input("Angle?\n")
            angle = float(angle)
            move_forward(px=px, speed=speed, angle=angle)
        elif key == '2':
            speed = input("Speed?\n")
            speed = float(speed)
            dir = input('left or right?\n')
            k_turn(px=px, speed=speed, dir=dir)
        elif key == '3':
            speed = input("Speed?\n")
            speed = float(speed)
            dir = input('left or right?\n')
            parallel_parking(px=px, speed=speed, dir=dir)
        elif key == '4':
            print("Exiting")
            break
        else:
            print("Wrong keyboard input,try again")
        px.stop()
    px.stop()
