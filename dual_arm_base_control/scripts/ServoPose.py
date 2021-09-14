#!usr/bin/env python
# -*- coding:utf-8 _*-
"""
@Time   :   2021/4/22  下午12:41
@Author :   HaoWANG, Foshan，China
@Email  :   haowanghk@163.com
@file   :  ServoPose.py
@Software : Pycharm
@Description:  
"""
import logging

# 单圈模式下的单个舵机伺服驱动参数设置类

# 舵机活动角度空间，C-space，
# MAX and MIN SERVO_ID: LEFT0,1,2
LEFT_SERVO_ID = [0,1,2]
LEFT_SERVO_MAX_ANGLE = [168,85,165]
LEFT_SERVO_MIN_ANGLE = [48,-15,60]

LEFT_SERVO_ZERO_ANGLE = [139,50,72]



#  RIGHT SERVO_ID 4,5,6
RIGHT_SERVO_ID = [4,5,6]
RIGHT_SERVO_MAX_ANGLE = [88,-10,180]
RIGHT_SERVO_MIN_ANGLE = [-32,-110,68]

RIGHT_SERVO_ZERO_ANGLE = [0,-78,83]


class Pose(object):
    # 舵机编号x的pose指令信息：角度值、模式、周期、速度、加减速度、舵机功率参数等
    # EXP: POSE = {'angle': None, 'is_mturn': False, 'interval': None, 'velocity': None,
    #         't_acc': 20, 't_dec': 20, 'power': 0,
    #         'mean_dps': 50.0}


    def __init__(self, POSE, SERVO_ID):
        self.__servo_id = SERVO_ID
        self.__pose = POSE
        self.__angle = POSE.get('angle')
        self.__is_mturn = POSE.get('is_mturn')
        self.__iterval = POSE.get('interval')
        self.__velocity = POSE.get('velocity')
        self.__t_acc = POSE.get('t_acc')
        self.__t_dec = POSE.get('t_dec')
        self.__power = POSE.get('power')
        self.__mean_dps = POSE.get('mean_dps')


    def get_value(self, key):
        value = self.__pose.get(key)
        return value

    def get_servo_id(self):
        return self.__servo_id

    # 输入合法性检测
    def check_key(self,key):
        POSE_KEY_LIST = ['angle', 'is_mturn', 'interval', 'velocity', 't_acc', 't_dec', 'power', 'mean_dps']
        if key in POSE_KEY_LIST:
            return self.get_value(self.get_value(key=key))
        else:
            logging.error("Key is not in  POSE_KEY_LIST: 'angle', 'is_mturn', 'interval', 'velocity', 't_acc', 't_dec', 'power', 'mean_dps'")
            return False

    # 角度值数据输入合法性检测，C-SPACE上下限滤波
    def check_angle_range(self,value):

        if self.__servo_id in LEFT_SERVO_ID:
            if value in range(LEFT_SERVO_MIN_ANGLE[self.__servo_id],
                              LEFT_SERVO_MAX_ANGLE[self.__servo_id]):
                return value

            else:
                logging.warning("The SERVO_ID {} angle value {} is out of range!".format(self.__servo_id, value))
                if value > LEFT_SERVO_MAX_ANGLE[self.__servo_id]:
                    value = LEFT_SERVO_MAX_ANGLE[self.__servo_id]
                elif value < LEFT_SERVO_MIN_ANGLE[self.__servo_id]:
                    value = LEFT_SERVO_MIN_ANGLE[self.__servo_id]

                return value

        elif self.__servo_id in RIGHT_SERVO_ID:
            if value in range(RIGHT_SERVO_MIN_ANGLE[self.__servo_id-4],
                              RIGHT_SERVO_MAX_ANGLE[self.__servo_id-4]):
                return value
            else:
                # logging.warning("The SERVO_ID: {} angle value: {} is out of range: {}!".format(self.__servo_id, value))
                if value > RIGHT_SERVO_MAX_ANGLE[self.__servo_id-4]:
                    value = RIGHT_SERVO_MAX_ANGLE[self.__servo_id-4]
                elif value < RIGHT_SERVO_MIN_ANGLE[self.__servo_id-4]:
                    value = RIGHT_SERVO_MIN_ANGLE[self.__servo_id-4]
                return value
        else:

            logging.error("The SERVO_ID: {} is not found!".format(self.__servo_id))

            return False

    # 赋值舵机ID的pose及角度参数值
    def set_pose_value(self, instance, value):
        if self.check_key(instance) != False:
            temp =  self.check_angle_range(value)
            if temp != False:
                self.__pose[instance] = temp
                return True
            else:
                logging.error("ERROR:self.check_angle_range(value) == False")
                return False
        else:
            logging.error("ERROR: self.check_key(instance) == False")
            return False
    # 输出舵机角度的上下限
    def logging_servo_bend_info(self):
        if self.__servo_id in LEFT_SERVO_ID:
            logging.info("SERVO_ID {} [MIN angle: {} ] \[MAX angle :{}]".format(self.__servo_id,
                                                                          LEFT_SERVO_MIN_ANGLE[self.__servo_id],
                                                                          LEFT_SERVO_MAX_ANGLE[self.__servo_id]))
        elif self.__servo_id in RIGHT_SERVO_ID:
            logging.info("SERVO_ID {} [MIN angle: {} ] \[MAX angle :{}]".format(self.__servo_id,
                                                                          RIGHT_SERVO_MIN_ANGLE[self.__servo_id-4],
                                                                          RIGHT_SERVO_MAX_ANGLE[self.__servo_id-4]))

        else:
            logging.error("The SERVO_ID: {} is not found!".format(self.__servo_id))



# testing demo
# POSE_SERVO_1 = {'servo_id': 2, 'angle': 100, 'is_mturn': False, 'interval': None, 'velocity': 10,
#                 't_acc': 10, 't_dec': 20, 'power': 0,
#                 'mean_dps': 50.0}
# demo = Pose(POSE=POSE_SERVO_1)
# print(demo.get_value(key='servo_id'))
