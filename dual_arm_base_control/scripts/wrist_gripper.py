#!usr/bin/env python
# -*- coding:utf-8 _*-
"""
@Time   :   2021/7/19  上午9:14
@Author :   HaoWANG, Foshan，China
@Email  :   haowanghk@163.com
@file   :  wrist_gripper.py.py
@Software : Pycharm
@Description:  
"""

import time
import logging
import math

from PCA9685 import PCA9685  # 导入驱动，
import time

INIT_ANGLE = 30

servo_mapping = {'left_wrist': 0, 'left_gripper': 1, 'right_wrist': 2, 'right_gripper': 3}
servo_limit = {'left_wrist': 100, 'left_gripper': 95, 'right_wrist': 100, 'right_gripper': 95}
servo_zero_init = {'left_wrist': 0, 'left_gripper': 80, 'right_wrist': 0, 'right_gripper': 80}
servo_init_G1 = {'left_wrist': 0, 'left_gripper': 50, 'right_wrist': 0, 'right_gripper': 50}
servo_init_G2 = {'left_wrist': 0, 'left_gripper': 80, 'right_wrist': 0, 'right_gripper': 90}
servo_init_G3 = {'left_wrist': 0, 'left_gripper': 60, 'right_wrist': 0, 'right_gripper': 90}
fPWM = 50  # hz


class WristGripperControl():
    def __init__(self, servo_channel, is_debug=True):
        self.is_debug = is_debug
        self.servo_channel = servo_channel

        self.pwm = PCA9685(0x40)  # 设置模块的地址，默认0x40
        self.pwm.setsq(fPWM)  # 设置频率
        self.pwm.init()  # 初始化pca9685
        self.pwm.allinit()  # 把16个通道初始化

    #    pwm.setangle(0,a)

    def all_ending_init(self, motion_group, servo_idx=servo_mapping):
        # left arm wrist and gripper angle init
        # 动作组1： 手腕0-90-0，手爪20-90-20
        self.set_servo_angle(servo_idx.get('left_wrist'), 0)
        self.set_servo_angle(servo_idx.get('left_gripper'), servo_init_G1.get('left_gripper'))
        time.sleep(0.1)
        # right arm wrist and gripper angle init
        self.set_servo_angle(servo_idx.get('right_wrist'), 0)
        self.set_servo_angle(servo_idx.get('right_gripper'), servo_init_G1.get('right_gripper'))
        time.sleep(0.5)

        self.set_servo_angle(servo_idx.get('left_wrist'), 90)
        time.sleep(0.1)
        self.set_servo_angle(servo_idx.get('left_gripper'), 85)
        time.sleep(1)
        self.set_servo_angle(servo_idx.get('left_gripper'), 30)
        time.sleep(0.2)
        self.set_servo_angle(servo_idx.get('left_wrist'), 0)
        time.sleep(0.1)

        # right arm wrist and gripper angle init
        self.set_servo_angle(servo_idx.get('right_wrist'), 60)
        time.sleep(0.5)

        self.set_servo_angle(servo_idx.get('right_gripper'), 85)
        time.sleep(0.5)
        self.set_servo_angle(servo_idx.get('right_gripper'), 30)
        time.sleep(0.2)

        self.set_servo_angle(servo_idx.get('right_wrist'), 0)

        if motion_group == 2:
            # 加紧左右手
            time.sleep(3)
            self.set_servo_angle(servo_idx.get('left_wrist'), 0)
            self.set_servo_angle(servo_idx.get('right_wrist'), 0)
            time.sleep(0.5)
            self.set_servo_angle(servo_idx.get('left_gripper'), servo_init_G2.get('left_gripper'))
            time.sleep(0.2)
            self.set_servo_angle(servo_idx.get('right_gripper'), servo_init_G2.get('right_gripper'))
            time.sleep(0.5)
        elif motion_group == 3:
            # 加紧左右手
            time.sleep(3)

            self.set_servo_angle(servo_idx.get('left_gripper'), servo_init_G3.get('left_gripper'))
            time.sleep(0.5)

            self.set_servo_angle(servo_idx.get('right_gripper'), servo_init_G3.get('right_gripper'))
            time.sleep(0.5)

            # 预备装配作业状态
            self.set_servo_angle(servo_idx.get('left_wrist'), 30)
            time.sleep(0.5)
            self.set_servo_angle(servo_idx.get('right_wrist'), 30)
            time.sleep(0.5)

        if self.is_debug:
            logging.info("self.pwm.setangle(servo_idx.get('XX'), INIT_ANGLE) ,SUCCESS!")

    def get_current_angle(self):
        pass

    def set_servo_angle(self, servo_idx, angle):
        # servo_idx = 0,1,2,3
        if servo_idx == 0:
            if angle > 100: angle = servo_limit.get('left_wrist')
            if angle < 0: angle = 0
            angle = angle + 50
            self.pwm.setangle(servo_idx, angle)
            time.sleep(0.02)
        elif servo_idx == 1:
            if angle > 100: angle = servo_limit.get('left_gripper')
            if angle < 0: angle = 0
            angle = angle + 50
            self.pwm.setangle(servo_idx, angle)
            time.sleep(0.02)
        elif servo_idx == 2:
            if angle > 100: angle = servo_limit.get('right_wrist')
            if angle < 0: angle = 0
            angle = angle + 55
            self.pwm.setangle(servo_idx, angle)
            time.sleep(0.02)
        elif servo_idx == 3:
            if angle > 100: angle = servo_limit.get('right_gripper')
            if angle < 0: angle = 0
            angle = angle + 40
            self.pwm.setangle(servo_idx, angle)
            time.sleep(0.02)
        else:
            logging.error("servo_idx:{} error ! MUST in [ 0,1,2,3]".format(servo_idx))
        if self.is_debug:
            logging.info("gripper_control_node servo idx {} with angle {}".format(servo_idx, angle))

    def rad2angle(self, rad_val):
        # rad to angle cmd : 0-pi
        angle_val = (abs(rad_val) / math.pi) * 180
        return angle_val
