#!usr/bin/env python
# -*- coding:utf-8 _*-
"""
@Time   :   2021/7/13  上午9:51
@Author :   HaoWANG, Foshan，China
@Email  :   haowanghk@163.com
@file   :  MotionGroup.py.py
@Software : Pycharm
@Description:  
"""

import time
import threading
from ServoPose import Pose
import logging
import robot_arm
import os.path
import MotionGroup

from wrist_gripper import servo_zero_init, servo_init_G1, servo_init_G2, servo_init_G3

DEFAULT_PATH = 'path_point/'
DEFAULT_DEBUG_PATH = 'path_point/debug/'


class MotionGroup():
    def __init__(self, arm_name, point_num, time_t=0.1, is_write_point=False, is_read_point=True, is_debug=True,
                 group_id=1):
        self.__arm_name = arm_name
        self.__point_num = point_num
        self.__time_t = time_t
        self.is_write_point = is_write_point
        self.is_read_point = is_read_point
        self.is_debug = is_debug
        self.group_id = group_id
        if self.is_write_point or self.is_read_point:
            self.__buffer = self.generate_path_point_buffer()

    def generate_path_point_buffer(self):
        # 生成用于记录机械臂各关节的示教点数据
        arm_current_angle = [[] for i in range(self.__point_num)]
        return arm_current_angle

    def write_motion_point(self, is_manu_write=True):
        stc_idx = 0
        while is_manu_write:
            key = input('waitKey input (y or q):')
            if key == 'y':
                stc_idx += 1
                # 记录机械臂各关节的示教点数据
                logging.info("--key==ord('y')--READING SERVO ANGLE")
                if self.is_debug:
                    path = DEFAULT_DEBUG_PATH
                    with open('path_debug_' + self.__arm_name.get_current_arm_name_str() + '_' + str(stc_idx) + '.txt',
                              'w') as f:
                        f.write('{} Motion Frame         Servo_id[]          angle'.format(self.__arm_name))
                        for it in range(self.__point_num):
                            self.__buffer[it] = self.__arm_name.get_arm_current_angle()
                            servo_list = self.__arm_name.get_arm_servo_list()
                            f.write(
                                'Frame id:{}     SERVO_ID:{}     angle:{}'.format(it, servo_list,
                                                                                  self.__buffer[it]) + '\r')
                            logging.info('Debug is {} file path is {}'.format(self.is_debug, path))
                            time.sleep(self.__time_t)
                else:
                    path = DEFAULT_PATH
                    with open(
                            'path_data_' + self.__arm_name.get_current_arm_name_str() + '_' + str(self.group_id) + '_' + str(
                                    stc_idx) + '.txt',
                            'w') as f:
                        init_angle = 30
                        zero_angle = 0
                        target_angle_1 = 53
                        target_angle_2 = 90

                        for it in range(self.__point_num):
                            tmp = init_angle
                            zero_tmp = zero_angle
                            alg = target_angle_2
                            if self.group_id == 1:
                                if stc_idx == 1:
                                    self.__buffer[it] = self.__arm_name.get_arm_current_angle()
                                    if it in range(0, 90):
                                        zero_tmp = zero_tmp + 1
                                        f.write("{},{},{},{},{},{},{},{},{},{}".format(self.__buffer[it][0],
                                                                                       self.__buffer[it][1],
                                                                                       self.__buffer[it][2],
                                                                                       zero_tmp,
                                                                                       self.__buffer[it][3],
                                                                                       self.__buffer[it][4],
                                                                                       self.__buffer[it][5],
                                                                                       zero_tmp,
                                                                                       init_angle,
                                                                                       init_angle
                                                                                       ) + '\r')
                                    elif it in range(90, 120):
                                        tmp = tmp + 2
                                        f.write("{},{},{},{},{},{},{},{},{},{}".format(self.__buffer[it][0],
                                                                                       self.__buffer[it][1],
                                                                                       self.__buffer[it][2],
                                                                                       90,
                                                                                       self.__buffer[it][3],
                                                                                       self.__buffer[it][4],
                                                                                       self.__buffer[it][5],
                                                                                       90,
                                                                                       tmp,
                                                                                       tmp
                                                                                       ) + '\r')
                                    else:
                                        alg = alg - 3
                                        f.write("{},{},{},{},{},{},{},{},{},{}".format(self.__buffer[it][0],
                                                                                       self.__buffer[it][1],
                                                                                       self.__buffer[it][2],
                                                                                       alg,
                                                                                       self.__buffer[it][3],
                                                                                       self.__buffer[it][4],
                                                                                       self.__buffer[it][5],
                                                                                       alg,
                                                                                       alg + 1,
                                                                                       alg + 1
                                                                                       ) + '\r')
                                    time.sleep(self.__time_t)

                                elif stc_idx == 2:
                                    self.__buffer[it] = self.__arm_name.get_arm_current_angle()
                                    if it in range(0, 90):
                                        zero_tmp = zero_tmp + 1
                                        f.write("{},{},{},{},{},{},{},{},{},{}".format(self.__buffer[it][0],
                                                                                       self.__buffer[it][1],
                                                                                       self.__buffer[it][2],
                                                                                       zero_tmp,
                                                                                       self.__buffer[it][3],
                                                                                       self.__buffer[it][4],
                                                                                       self.__buffer[it][5],
                                                                                       zero_tmp,
                                                                                       servo_init_G2.get(
                                                                                           'left_gripper'),
                                                                                       init_angle + 20
                                                                                       ) + '\r')
                                    else:
                                        f.write("{},{},{},{},{},{},{},{},{},{}".format(self.__buffer[it][0],
                                                                                       self.__buffer[it][1],
                                                                                       self.__buffer[it][2],
                                                                                       50,
                                                                                       self.__buffer[it][3],
                                                                                       self.__buffer[it][4],
                                                                                       self.__buffer[it][5],
                                                                                       50,
                                                                                       target_angle_1 - 10,
                                                                                       target_angle_1 + 20
                                                                                       ) + '\r')
                                    time.sleep(self.__time_t)

                                elif stc_idx == 3:
                                    self.__buffer[it] = self.__arm_name.get_arm_current_angle()
                                    if it in range(0, 50):
                                        f.write("{},{},{},{},{},{},{},{},{},{}".format(self.__buffer[it][0],
                                                                                       self.__buffer[it][1],
                                                                                       self.__buffer[it][2],
                                                                                       45,
                                                                                       self.__buffer[it][3],
                                                                                       self.__buffer[it][4],
                                                                                       self.__buffer[it][5],
                                                                                       45,
                                                                                       init_angle + 10,
                                                                                       init_angle - 10
                                                                                       ) + '\r')
                                    else:
                                        f.write("{},{},{},{},{},{},{},{},{},{}".format(self.__buffer[it][0],
                                                                                       self.__buffer[it][1],
                                                                                       self.__buffer[it][2],
                                                                                       0,
                                                                                       self.__buffer[it][3],
                                                                                       self.__buffer[it][4],
                                                                                       self.__buffer[it][5],
                                                                                       0,
                                                                                       init_angle + 20,
                                                                                       init_angle + 30
                                                                                       ) + '\r')
                                    time.sleep(self.__time_t)
                            elif self.group_id == 2:
                                self.__buffer[it] = self.__arm_name.get_arm_current_angle()
                                if it in range(0, 50):
                                    zero_tmp = zero_tmp + 1
                                    f.write("{},{},{},{},{},{},{},{},{},{}".format(self.__buffer[it][0],
                                                                                   self.__buffer[it][1],
                                                                                   self.__buffer[it][2],
                                                                                   zero_tmp,
                                                                                   self.__buffer[it][3],
                                                                                   self.__buffer[it][4],
                                                                                   self.__buffer[it][5],
                                                                                   0,
                                                                                   servo_init_G2.get('left_gripper'),
                                                                                   servo_init_G2.get('right_gripper')
                                                                                   ) + '\r')
                                else:
                                    f.write("{},{},{},{},{},{},{},{},{},{}".format(self.__buffer[it][0],
                                                                                   self.__buffer[it][1],
                                                                                   self.__buffer[it][2],
                                                                                   0,
                                                                                   self.__buffer[it][3],
                                                                                   self.__buffer[it][4],
                                                                                   self.__buffer[it][5],
                                                                                   0,
                                                                                   servo_init_G2.get('left_gripper'),
                                                                                   servo_init_G2.get('right_gripper')
                                                                                   ) + '\r')
                                time.sleep(self.__time_t)
                            elif self.group_id == 3:
                                self.__buffer[it] = self.__arm_name.get_arm_current_angle()
                                if it in range(0, 50):
                                    zero_tmp = zero_tmp + 1
                                    f.write("{},{},{},{},{},{},{},{},{},{}".format(self.__buffer[it][0],
                                                                                   self.__buffer[it][1],
                                                                                   self.__buffer[it][2],
                                                                                   zero_tmp,
                                                                                   self.__buffer[it][3],
                                                                                   self.__buffer[it][4],
                                                                                   self.__buffer[it][5],
                                                                                   zero_tmp,
                                                                                   servo_init_G3.get('left_gripper'),
                                                                                   servo_init_G3.get('right_gripper')
                                                                                   ) + '\r')
                                else:
                                    f.write("{},{},{},{},{},{},{},{},{},{}".format(self.__buffer[it][0],
                                                                                   self.__buffer[it][1],
                                                                                   self.__buffer[it][2],
                                                                                   30,
                                                                                   self.__buffer[it][3],
                                                                                   self.__buffer[it][4],
                                                                                   self.__buffer[it][5],
                                                                                   30,
                                                                                   servo_init_G3.get('left_gripper'),
                                                                                   servo_init_G3.get('right_gripper')
                                                                                   ) + '\r')
                                time.sleep(self.__time_t)

                            elif self.group_id == 4:
                                
                                self.__buffer[it] = self.__arm_name.get_arm_current_angle()
                                f.write("{},{},{},{},{},{},{},{},{},{}".format(self.__buffer[it][0],
                                                                               self.__buffer[it][1],
                                                                               self.__buffer[it][2],
                                                                               servo_zero_init.get('left_wrist'),
                                                                               self.__buffer[it][3],
                                                                               self.__buffer[it][4],
                                                                               self.__buffer[it][5],
                                                                               servo_zero_init.get('right_wrist'),
                                                                               servo_zero_init.get('left_gripper'),
                                                                               servo_zero_init.get('right_gripper')
                                                                               ) + '\r')
                                time.sleep(self.__time_t)
                # # 标志位置假
                # is_manu_write = False

            elif key == 'q':
                print("key==Q, Processing Quit!")
                return self.__buffer

            else:
                logging.warning('waitKey input (y or q): {}'.format(key))

    def read_motion_point(self, file_name_list):
        # put the motion path in different file
        traj_points = []
        servo_angle_cmd = []

        for it in file_name_list:
            if os.path.isfile(it):
                with open(it, 'r') as file:
                    lines = file.readlines()
                    traj_points.append(lines)
                    for line in lines:
                        line = line.strip()  # 移除行尾换行符
                        angle = line.split(',')  # 分割字符串，以‘，’为分割
                        for it in range(len(angle)):
                            angle[it] = float(angle[it])  # 强制类型转换

                        servo_angle_cmd.append(angle)

            else:
                logging.warning("File name {} is not exist!".format(file_name_list[it]))

        return traj_points, servo_angle_cmd

    def set_point_num(self, num):
        # set path point num
        if num in range(10000):
            self.__point_num = num
            return True
        else:
            logging.warning("path point num MUST < 10,000")
            return False

    def set_sampling_frequency(self, freq=10):
        # set sampling frequency N/Sec
        if freq in range(50):
            self.__time_t = round(1 / freq, 2)
            return True
        else:
            self.__time_t = 0.1
            logging.warning("freq should be in range (50)!")
            return False
