#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/hiwonder/')
import math

class Mecanum:
    # A = 67  # mm
    # B = 59  # mm
    # WHEEL_DIAMETER = 65  # mm

    def __init__(self, board, a=67, b=59, wheel_diameter=65):
        self.a = a
        self.b = b
        self.wheel_diameter = wheel_diameter
        self.velocity = 0
        self.direction = 0
        self.angular_rate = 0
        self.board = board
        print(self.board.get_battery())

    def reset_motors(self):
        self.board.set_motor_duty([[1, 0], [2, 0], [3, 0], [4, 0]])
        self.velocity = 0
        self.direction = 0
        self.angular_rate = 0

    def set_velocity(self, velocity, direction, angular_rate, fake=False):
        """
        Use polar coordinates to control moving
        motor1 v1|  ↑  |v2 motor2
                 |     |
        motor3 v3|     |v4 motor4
        :param velocity: mm/s
        :param direction: Moving direction 0~360deg, 180deg<--- ↑ ---> 0deg
        :param angular_rate:  The speed at which the chassis rotates
        :param fake:
        :return:
        """
        rad_per_deg = math.pi / 180
        vx = velocity * math.cos(direction * rad_per_deg)
        vy = velocity * math.sin(direction * rad_per_deg)
        vp = -angular_rate * (self.a + self.b)
        v1 = int(vx - vy - vp) 
        v2 = int(vx + vy + vp)
        v3 = int(vx + vy - vp)
        v4 = int(vx - vy + vp)
        if fake:
            return
        self.board.set_motor_duty([[1, -v1], [2, v2], [3, -v3], [4, v4]])
        self.velocity = velocity
        self.direction = direction
        self.angular_rate = angular_rate



