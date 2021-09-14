#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@Copyright:  Jihua Lab 2021.
@File    :   robot_arm.py
@Author :   HaoWANG, Foshan，China
@Contact :   wanghao@jihualab.com
@License :   JHL ( MIT License)

@Description:  


@Create Time   :   2021/4/19  08:32
@Modify Time      @Author    @Version    @Desciption
------------      -------    --------    -----------
2021/4/16 16:09   wanghao      1.1.0         None
2021/4/20 18:47   wanghao      1.2.0         None
2021/7/15 18:47   wanghao      1.6.2         None
'''

# import lib

import struct
import serial
import logging

from uservo import UsInit, UartServoManager, UartServoInfo

# global const definition
CONTROL_MODE_LIST = ["ANGLE", "WHEEL", "DAMPING", "CIRCLE"]

LEFT_ARM_NAME_LIST = ["left_arm", "Left_arm", "left arm", "L_arm", "LEFT_ARM"]

RIGHT_ARM_NAME_LIST = ["right_arm", "Right_arm", "right arm", "R_arm", "RIGHT_ARM"]

DUAL_ARM_NAME_LIST = ["dual_arm", "Dual_arm", "dual arm", "D_arm", "DUAL_ARM"]

# arm DOF
C_SINGLE_ARM_DOF = 3
C_DUAL_ARM_DOF = 2 * C_SINGLE_ARM_DOF

# left and right arm servo default angle and zero_position
# left servo id idex: 0,1,2
LEFT_DEFAULT_ANGLE = \
    [{'Servo_id': 0, 'upper_angle': 168, 'lower_angle': 48, 'zero_angle': 140, 'velocity': 15, 't_acc_dft': 30,
      't_dec_dft': 30, 'power': 8000, 'mean_dps': 200},
     {'Servo_id': 1, 'upper_angle': 85, 'lower_angle': -15, 'zero_angle': 45, 'velocity': 15, 't_acc_dft': 30,
      't_dec_dft': 30,
      'power': 8000, 'mean_dps': 200},
     {'Servo_id': 2, 'upper_angle': 60, 'lower_angle': 165, 'zero_angle': 155, 'velocity': 15, 't_acc_dft': 30,
      't_dec_dft': 30,
      'power': 8000, 'mean_dps': 200}]
# right servo id idex: 4,5,6
# -4.6, -76.5, 160.9
RIGHT_DEFAULT_ANGLE = \
    [{'Servo_id': 4, 'upper_angle': -32, 'lower_angle': 88, 'zero_angle': -4.6, 'velocity': 15, 't_acc_dft': 30,
      't_dec_dft': 30,
      'power': 8000, 'mean_dps': 200},
     {'Servo_id': 5, 'upper_angle': -110, 'lower_angle': -10, 'zero_angle': -76.5, 'velocity': 15, 't_acc_dft': 30,
      't_dec_dft': 30,
      'power': 8000, 'mean_dps': 200},
     {'Servo_id': 6, 'upper_angle': 68, 'lower_angle': 180, 'zero_angle': 171, 'velocity': 15, 't_acc_dft': 30,
      't_dec_dft': 30,
      'power': 8000, 'mean_dps': 200}]
# right servo id idex: 0,1,2,4,5,6
DUAL_ARM_DEFAULT_ANGLE = LEFT_DEFAULT_ANGLE + RIGHT_DEFAULT_ANGLE

LEFT_ARM_SERVO_IDX = [0,1,2]
RIGHT_ARM_SERVO_IDX = [4,5,6]

# 创建单机械臂n-Dof对象
class ArmMotionControl(object):
    SERVO_ID = None  # 舵机的ID号
    SERVO_BAUDRATE = 115200  # 舵机的波特率

    # SERVO_PORT_NAME = '/dev/ttyUSB0'  # 舵机串口号
    # linux port
    # 初始化并自动获取舵机串口号

    def __init__(self,
                 arm_name,
                 dof,
                 CONTROL_MODE="DAMPING",
                 with_wrist=False,
                 is_debug=False,
                 damping_power=500,
                 using_default_init_pose=True):
        # Class ArmMotionControl consructor

        # publsic parameter init
        self.uservo_obj = None
        self.is_debug = is_debug
        self.using_default_init_pose = using_default_init_pose

        # private parameter init
        self.__arm_name = arm_name
        self.__arm_dof = dof
        self.__control_mode = CONTROL_MODE
        self.__servo_port = None
        self.__with_wrist_or_not = with_wrist
        # self.__servo_buffer = servo_buffer()
        self.__serial_id = None
        self.__servo_list = None
        self.__arm_servo_list = None
        self.__power = damping_power  # 单位mW
        self.__is_initialized = False

        self.__init_pose = None
        self.default_angle_pose = None

        if self.__arm_name in DUAL_ARM_NAME_LIST:
            self.default_dof = C_DUAL_ARM_DOF
        else:
            self.default_dof = C_SINGLE_ARM_DOF

        if self.using_default_init_pose:
            POSE_DEFAULT = {'servo_id': None, 'angle': 100, 'is_mturn': False, 'interval': 1000, 'velocity': 30,
                            't_acc': 30, 't_dec': 30, 'power': 1000, 'mean_dps': 200.0}
            self.default_angle_pose = POSE_DEFAULT

    def creat_uservo_obj(self, SERVO_BAUDRATE=SERVO_BAUDRATE):
        # 舵机相关参数配置初始化接口API
        # 初始化日志、初始化舵机端口名称及地址、初始化舵机管理器类对象、执行端口扫描及异常检测
        uservo_init = UsInit(logger_level="INFO", scan_servo_port=True)
        uservo_init.set_logging_mode()

        # 获取端口名称及地址
        SERVO_PORT_NAME = uservo_init.get_servo_port_name()
        if SERVO_PORT_NAME:
            self.__set_servo_port(SERVO_PORT_NAME)
        else:
            logging.error("SERVO_PORT_NAME 端口初始化失败：{}"
                          .format(SERVO_PORT_NAME))
            return False

        # 端口、波特率等参数初始化舵机串口serial类对象，uart
        uart = serial.Serial(port=SERVO_PORT_NAME, baudrate=SERVO_BAUDRATE, \
                             parity=serial.PARITY_NONE, stopbits=1, \
                             bytesize=8, timeout=0)

        try:
            # 初始化舵机管理器
            if self.is_debug and uart:
                logging.info('-----------------' +
                             '\n' +
                             'Success!初始化舵机管理器,执行端口扫描：')
                logging.info("开始扫描端口{} 是否打开: {} "
                             .format(SERVO_PORT_NAME,
                                     uart.isOpen()))
            # 创建并初始化舵机管理器类对象
            uservo_obj = UartServoManager(uart, is_scan_servo=False, is_debug=self.is_debug)
            self.__assm_serial_servo_obj(uservo_obj)
            logging.info("{} creat uservo manager obj(SERVO_BAUDRATE= {} ) Success!" + '\n'.format(self.__arm_name,
                                                                                                   self.SERVO_BAUDRATE))

        except serial.SerialException as serialErr:
            logging.error("serial.SerialException：uart.isOpen() --》 {}"
                          .format(uart.isOpen()))
            print(serialErr)

        finally:
            logging.info("端口扫描及端口初始化完成" + '\n'
                                          "-----------------")
        return self.uservo_obj

    def robot_arm_angle_pose_init(self, angle_vel=10):
        # 机械臂的角度模式初始化必须在机械臂阻尼模式初始化之后进行，安全模式，不能修改。
        # POSE_SERVO_1 = {'servo_id': 2, 'angle': 100, 'is_mturn': False, 'interval': None, 'velocity': 10,
        #                 't_acc': 10, 't_dec': 20, 'power': 0,
        #                 'mean_dps': 50.0}
        FLAG = False

        if self.__is_initialized and self.using_default_init_pose and (angle_vel in range(10, 50)):

            if self.__arm_name in LEFT_ARM_NAME_LIST:
                for SERVO_ID in self.__arm_servo_list:
                    left_servo_angle_init_ls = {'Servo_id': SERVO_ID,
                                                'angle': LEFT_DEFAULT_ANGLE[SERVO_ID].get("zero_angle"),
                                                'is_mturn': False,
                                                'interval': LEFT_DEFAULT_ANGLE[SERVO_ID].get("interval"),
                                                'velocity': angle_vel,
                                                't_acc': LEFT_DEFAULT_ANGLE[SERVO_ID].get("t_acc_dft"),
                                                't_dec': LEFT_DEFAULT_ANGLE[SERVO_ID].get("t_dec_dft"),
                                                'power': self.__power,
                                                'mean_dps': LEFT_DEFAULT_ANGLE[SERVO_ID - 4].get("mean_dps")
                                                }
                    self.__init_pose.append(left_servo_angle_init_ls)
                FLAG = True

            elif self.__arm_name in RIGHT_ARM_NAME_LIST:
                for SERVO_ID in self.__arm_servo_list:
                    right_servo_angle_init_ls = {'Servo_id': SERVO_ID,
                                                 'angle': RIGHT_DEFAULT_ANGLE[SERVO_ID - 4].get("zero_angle"),
                                                 'velocity': angle_vel,
                                                 'is_mturn': False,
                                                 'interval': RIGHT_DEFAULT_ANGLE[SERVO_ID].get("interval"),
                                                 't_acc': RIGHT_DEFAULT_ANGLE[SERVO_ID - 4].get("t_acc_dft"),
                                                 't_dec': RIGHT_DEFAULT_ANGLE[SERVO_ID - 4].get("t_dec_dft"),
                                                 'power': self.__power,
                                                 'mean_dps': RIGHT_DEFAULT_ANGLE[SERVO_ID - 4].get("mean_dps")
                                                 }

                    self.__init_pose.append(right_servo_angle_init_ls)
                FLAG = True

            elif self.__arm_name in DUAL_ARM_NAME_LIST:
                for SERVO_ID in self.__arm_servo_list:
                    if SERVO_ID in LEFT_ARM_SERVO_IDX:
                        left_servo_angle_init_ls = {'Servo_id': SERVO_ID,
                                                    'angle': LEFT_DEFAULT_ANGLE[SERVO_ID].get("zero_angle"),
                                                    'is_mturn': False,
                                                    'interval': LEFT_DEFAULT_ANGLE[SERVO_ID].get("interval"),
                                                    'velocity': angle_vel,
                                                    't_acc': LEFT_DEFAULT_ANGLE[SERVO_ID].get("t_acc_dft"),
                                                    't_dec': LEFT_DEFAULT_ANGLE[SERVO_ID].get("t_dec_dft"),
                                                    'power': self.__power,
                                                    'mean_dps': LEFT_DEFAULT_ANGLE[SERVO_ID - 4].get("mean_dps")
                                                    }
                        self.__init_pose.append(left_servo_angle_init_ls)
                    elif SERVO_ID in RIGHT_ARM_SERVO_IDX:
                        right_servo_angle_init_ls = {'Servo_id': SERVO_ID,
                                                     'angle': RIGHT_DEFAULT_ANGLE[SERVO_ID - 4].get("zero_angle"),
                                                     'velocity': angle_vel,
                                                     'is_mturn': False,
                                                     'interval': RIGHT_DEFAULT_ANGLE[SERVO_ID].get("interval"),
                                                     't_acc': RIGHT_DEFAULT_ANGLE[SERVO_ID - 4].get("t_acc_dft"),
                                                     't_dec': RIGHT_DEFAULT_ANGLE[SERVO_ID - 4].get("t_dec_dft"),
                                                     'power': self.__power,
                                                     'mean_dps': RIGHT_DEFAULT_ANGLE[SERVO_ID - 4].get("mean_dps")
                                                     }

                        self.__init_pose.append(right_servo_angle_init_ls)


                    dual_arm_servo_angle_init_ls = left_servo_angle_init_ls + right_servo_angle_init_ls


                FLAG = True

            else:
                logging.error("Robot arm name error! Please check the definintion of the LEFT and RIGHT arm!")
                FLAG = False

        return FLAG

    def __assm_serial_servo_obj(self, serial_servo_obj):
        # 私有成员函数
        # 初始化机械臂 总线舵机管理器 类对象
        # 返回true false
        if serial_servo_obj is not None:
            self.uservo_obj = serial_servo_obj
        else:
            print(serial.SerialException)

    def __set_servo_port(self, servo_port):
        # 私有成员函数
        # 设置总线舵机端口值
        # 如： /dev/ttyUSB0，返回true false
        if servo_port is not None:
            self.__servo_port = servo_port
            return True
        else:
            print(serial.SerialException)
            return False

    def servo_buffer(self, is_scan_servo=True):
        # 读取端口所有舵机id值，并赋值类对象私有成员
        if is_scan_servo and self.__servo_port != None:
            logging.info('-----------------'
                         '开始扫描端口{} 舵机'
                         .format(self.__servo_port))
            self.uservo_obj.scan_servo()
            servo_list = list(self.uservo_obj.servos.keys())
            if servo_list == None:
                logging.error('扫描端口{} 舵机ID错误！请查看舵机端口地址或电源是否上电！'
                              .format(self.__servo_port))
                return False
            else:
                logging.info('扫描端口{} 舵机ID完成' + '\n'
                                               '------------------' + '\n'
                                                                      '开始端口舵机ID在线检测'
                             .format(self.__servo_port))
                # 舵机端口扫描成功后将舵机列表整体赋值
                self.__set_all_servo_id(servo_list=servo_list)

                # 舵机ping连通性检测
                for SERVO_ID in self.__servo_list:
                    is_online = self.uservo_obj.ping(SERVO_ID, with_logging_info=self.is_debug)
                    if is_online == True:
                        print("舵机ID={} 是否在线: {}".format(SERVO_ID, is_online))

                if self.__arm_dof != self.default_dof:
                    logging.warning("{} 机械臂端口舵机ID列表初始化警告： 舵机数{} ！= 自由度数{}"
                                    .format(self.__arm_name,
                                            servo_list.__len__(),
                                            self.__arm_dof))

                elif self.__arm_name in LEFT_ARM_NAME_LIST:
                    # left arm servo_id list init
                    arm_servo_list = servo_list[:C_SINGLE_ARM_DOF]
                    self.__set_arm_servo_id(servo_list=arm_servo_list)


                elif self.__arm_name in RIGHT_ARM_NAME_LIST:
                    # right arm  servo_id list init
                    arm_servo_list = servo_list[C_SINGLE_ARM_DOF:]
                    self.__set_arm_servo_id(servo_list=arm_servo_list)

                elif self.__arm_name in DUAL_ARM_NAME_LIST:
                    # dual arm  servo_id list init
                    arm_servo_list = servo_list[:]
                    self.__set_arm_servo_id(servo_list=arm_servo_list)

                else:
                    logging.warning("Undefine error! 是否 {} 机械臂端口舵机ID初始化失败： 舵机数{} ！= 自由度数{}"
                                    .format(self.__arm_name,
                                            servo_list.__len__(),
                                            self.__arm_dof))
                    return False

                return True

        else:
            logging.error('扫描端口{} 舵机ID错误！请查看舵机端口地址或电源是否上电！'
                          .format(self.__servo_port))
            return False

    def __set_all_servo_id(self, servo_list):
        # 设置总线舵机端口值
        # 如： /dev/ttyUSB0，返回true false
        if servo_list is not None:
            self.__servo_list = servo_list
            logging.info("双臂机器人关节舵机ID初始化成功：{}".format(self.__arm_name))
        else:
            print(serial.SerialException)

    def __set_arm_servo_id(self, servo_list):
        # 设置总线舵机端口值
        # 如： /dev/ttyUSB0，返回true false
        if servo_list is not None:
            self.__arm_servo_list = servo_list
            logging.info("{} 机械臂端口舵机ID初始化成功".format(self.__arm_name))
        else:
            print(serial.SerialException)

    def logging_arm_info(self):
        servo_list = self.__arm_servo_list
        if self.is_debug:
            logging.info("当前机械臂信息：\n"
                         "ArmName： {}      DoF: {}    SerialPort: {}   SerialServo_ID: {}".
                         format(self.__arm_name,
                                self.__arm_dof,
                                self.__servo_port,
                                servo_list))
            arm_servo_state_info = []
            for SERVO_ID in servo_list:
                arm_servo_state_info.append(self.get_servo_status(SERVO_ID=SERVO_ID))
            logging.info("输出当前各个关节舵机信息：{}".format(arm_servo_state_info))

        else:
            logging.warning("没有管理员权限，无法访问舵机参数信息！")

    def arm_state_init(self, force_angle_init=False):
        # 初始化单机械臂的控制模式：默认初始化为阻尼模式
        # 循环设置每个舵机角度、速度和加速度，并反馈初始化进度及结果

        if (self.__is_initialized == False) and self.__control_mode in CONTROL_MODE_LIST and self.uservo_obj:
            if self.__control_mode in ["DAMPING", "WHEEL"]:
                # 初始值默认为阻尼模式，阻尼模式下的功率, 单位mW
                logging.info(" 执行初始化操作，默认阻尼模式，阻尼模式下的功率 ： {} 单位mW".format(self.__power))
                for SERVO_ID in self.__arm_servo_list:
                    self.uservo_obj.set_damping(SERVO_ID, self.__power)
                    if self.is_debug:
                        logging.info(" 初始化机械臂工作模式：{}   舵机编号ID：{}      功率：{} mW"
                                     .format(self.__control_mode,
                                             SERVO_ID,
                                             self.__power))
                self.__is_initialized = True
                return True
            elif force_angle_init and (self.__control_mode == "ANGLE" or self.__control_mode == "CIRCLE"):
                self.set_all_servo_mode(CONTROL_MODE="CIRCLE")
                logging.info(" 执行初始化操作，角度模式下的功率 ： {} 单位mW".format(self.__power))
                # logging.warning("机械臂初始化失败：请检查是否舵机串口失效或控制模式设置有误。")
        else:
            logging.error("机械臂初始化失败：请检查是否重复初始化或舵机串口失效或控制模式设置有误。")
            self.__is_initialized = False
            return False

    def set_damping_power(self, damping_power):
        if damping_power in range(0, 5000):
            self.__power = damping_power

    def set_all_servo_mode(self, const_vel=10, CONTROL_MODE="DAMPING"):
        # mode\power\angle\tempture\velo\acc

        if CONTROL_MODE in CONTROL_MODE_LIST:
            self.__control_mode = CONTROL_MODE
            if self.__control_mode in ["DAMPING", "WHEEL"]:
                # 初始值默认为阻尼模式，阻尼模式下的功率, 单位mW

                for SERVO_ID in self.__servo_list:
                    self.uservo_obj.set_damping(SERVO_ID, power=self.__power)
                    if self.is_debug:
                        logging.info("机械臂设置工作模式：{}   舵机编号ID：{}      功率：{} mW"
                                     .format(self.__control_mode,
                                             SERVO_ID,
                                             self.__power))

                return True

            elif self.__control_mode == "CIRCLE":
                # [单圈模式]设置舵机角度为.0°, 设置转速默认为50 °/s, 加速时间默认100ms, 减速时间默认100ms"
                if self.__arm_name in LEFT_ARM_NAME_LIST:
                    # 执行左臂角度模式初始化参数设定，初始速度10，初始角度为各个关节的中性位置
                    # 舵机ID 0，1，2
                    for SERVO_ID in self.__arm_servo_list:
                        if self.__is_debug:
                            logging.info("机械臂设置工作模式：{}   舵机编号ID：{}  关节角度信息：{} "
                                         .format(self.__control_mode,
                                                 SERVO_ID,
                                                 self.LEFT_DEFAULT_ANGLE[SERVO_ID].get("zero_angle")))

                        self.uservo_obj.set_servo_angle(servo_id=LEFT_DEFAULT_ANGLE[SERVO_ID].get('Servo_id'),
                                                        angle=LEFT_DEFAULT_ANGLE[SERVO_ID].get('zero_angle'),
                                                        interval=2000,
                                                        velocity=LEFT_DEFAULT_ANGLE[SERVO_ID].get('velocity'),
                                                        t_acc=LEFT_DEFAULT_ANGLE[SERVO_ID].get('t_acc_dft'),
                                                        t_dec=LEFT_DEFAULT_ANGLE[SERVO_ID].get('t_dec_dft'),
                                                        power=LEFT_DEFAULT_ANGLE[SERVO_ID].get('power'))
                        self.uservo_obj.wait()  # 等待舵机静止

                    return True

                elif self.__arm_name in RIGHT_ARM_NAME_LIST:
                    # 执行右臂角度模式初始化参数设定，初始速度10，初始角度为各个关节的中性位置
                    # 舵机ID 4,5,6

                    for SERVO_ID in self.__arm_servo_list:
                        if self.is_debug:
                            logging.info("机械臂设置工作模式：{}   舵机编号ID：{}  关节角度信息：{} "
                                         .format(self.__control_mode,
                                                 SERVO_ID,
                                                 RIGHT_DEFAULT_ANGLE[SERVO_ID - 4].get("zero_angle")))

                        # set servo angle mode and parameters

                        self.uservo_obj.set_servo_angle(servo_id=RIGHT_DEFAULT_ANGLE[SERVO_ID - 4].get('Servo_id'),
                                                        angle=RIGHT_DEFAULT_ANGLE[SERVO_ID - 4].get('zero_angle'),
                                                        interval=2000,
                                                        velocity=RIGHT_DEFAULT_ANGLE[SERVO_ID - 4].get('velocity'),
                                                        t_acc=RIGHT_DEFAULT_ANGLE[SERVO_ID - 4].get('t_acc_dft'),
                                                        t_dec=RIGHT_DEFAULT_ANGLE[SERVO_ID - 4].get('t_dec_dft'),
                                                        power=RIGHT_DEFAULT_ANGLE[SERVO_ID].get('power'))
                    return True

                elif self.__arm_name in DUAL_ARM_NAME_LIST:
                    # 执行dual_arm角度模式初始化参数设定，初始速度10，初始角度为各个关节的中性位置
                    # 舵机ID 0,1,2,4,5,6

                    for SERVO_ID in self.__arm_servo_list:
                        if self.is_debug:
                            logging.info("机械臂设置工作模式：{}   舵机编号ID：{}  关节角度信息：{} "
                                         .format(self.__control_mode,
                                                 SERVO_ID,
                                                 DUAL_ARM_DEFAULT_ANGLE[0].get("zero_angle")))

                        # set servo angle mode and parameters
                        if SERVO_ID in LEFT_ARM_SERVO_IDX:
                            self.uservo_obj.set_servo_angle(servo_id=LEFT_DEFAULT_ANGLE[SERVO_ID].get('Servo_id'),
                                                            angle=LEFT_DEFAULT_ANGLE[SERVO_ID].get('zero_angle'),
                                                            interval=2000,
                                                            velocity=LEFT_DEFAULT_ANGLE[SERVO_ID].get('velocity'),
                                                            t_acc=LEFT_DEFAULT_ANGLE[SERVO_ID].get('t_acc_dft'),
                                                            t_dec=LEFT_DEFAULT_ANGLE[SERVO_ID].get('t_dec_dft'),
                                                            power=LEFT_DEFAULT_ANGLE[SERVO_ID].get('power'))
                        elif SERVO_ID in RIGHT_ARM_SERVO_IDX:
                            self.uservo_obj.set_servo_angle(servo_id=RIGHT_DEFAULT_ANGLE[SERVO_ID - 4].get('Servo_id'),
                                                            angle=RIGHT_DEFAULT_ANGLE[SERVO_ID - 4].get('zero_angle'),
                                                            interval=2000,
                                                            velocity=RIGHT_DEFAULT_ANGLE[SERVO_ID - 4].get('velocity'),
                                                            t_acc=RIGHT_DEFAULT_ANGLE[SERVO_ID - 4].get('t_acc_dft'),
                                                            t_dec=RIGHT_DEFAULT_ANGLE[SERVO_ID - 4].get('t_dec_dft'),
                                                            power=RIGHT_DEFAULT_ANGLE[SERVO_ID - 4].get('power'))

                    return True

                else:
                    logging.warning("WARNING: Robot Arm idx {} is not fund, Please check ! ".format(self.__arm_name))
                    return False

        else:
            logging.warning("CONTROL_MODE Set Error :{} "
                            "or SerialServo not initialized : {} ".format(CONTROL_MODE,
                                                                          self.uservo_obj))
            return False

    def set_single_servo_pose(self, Pose):
        # 单个舵机、单圈模式： SERVO_ID power\angle\velocity\acc
        angle = Pose.get_value("angle")
        interval = Pose.get_value('interval')
        velocity = Pose.get_value("velocity")
        t_acc = Pose.get_value("t_acc")
        t_dec = Pose.get_value("t_dec")
        power = Pose.get_value('power')
        mean_dps = Pose.get_value('mean_dps')
        if self.uservo_obj:
            self.uservo_obj.set_servo_angle(Pose.get_servo_id(),
                                            velocity=velocity,
                                            interval=interval,
                                            angle=angle,
                                            t_acc=t_acc,
                                            t_dec=t_dec,
                                            power=power,
                                            mean_dps=mean_dps)
        else:
            logging.error(serial.SerialException)

    def get_arm_current_angle(self, is_get_pose=True):

        arm_current_angle_list = []  # 空表
        if is_get_pose:
            # 读取左\右机械臂当前舵机角度： SERVO_ID power\angle\velocity\acc
            for servo_id in self.__arm_servo_list:
                arm_current_angle_list.append(
                    self.uservo_obj.query_servo_angle(servo_id))  # 将当前机械臂各关节角度值存储

                if self.is_debug:
                    logging.info("获取{}机械臂的关节角度：".format(self.__arm_name))
                    # print(arm_current_angle_list)

        return arm_current_angle_list

    def read_single_servo_angle(self, SERVO_ID):
        # mode\power\angle\tempture\velo\acc
        """响应查询单个舵机角度"""
        angle_servo_id = self.uservo_obj.query_servo_angle(SERVO_ID)
        if self.is_debug:
            logging.info("当前舵机ID: {}    角度: {:4.1f} °".format(SERVO_ID, angle_servo_id), end='\n')
        return angle_servo_id

    def read_arm_all_servo_angle(self):
        # mode\power\angle\tempture\velo\acc
        """更新左右机械臂所有的舵机角度"""

        left_arm_angle = []  # 查询结果
        right_arm_angle = []
        dual_arm_angle = []
        if self.__arm_name in LEFT_ARM_NAME_LIST:
            LEFT_ARM = [0, 1, 2]  # 左右机械臂舵机ID
            for SERVO_ID in LEFT_ARM:
                left_arm_angle.append(self.read_single_servo_angle(SERVO_ID=SERVO_ID))
                return left_arm_angle
        elif self.__arm_name in RIGHT_ARM_NAME_LIST:
            RIGHT_ARM = [4, 5, 6]
            for SERVO_ID in RIGHT_ARM:
                right_arm_angle.append(self.read_single_servo_angle(SERVO_ID=SERVO_ID))
                return right_arm_angle
        elif self.__arm_name in DUAL_ARM_NAME_LIST:
            DUAL_ARM  = [0,1,2,4,5,6]
            for SERVO_ID in DUAL_ARM:
                dual_arm_angle.append(self.read_single_servo_angle(SERVO_ID=SERVO_ID))
                return dual_arm_angle


    def read_single_servo_voltage(self, SERVO_ID):
        # mode\power\angle\tempture\velo\acc
        """响应查询单个舵机电压"""
        servo_voltage_list = []  # 空表
        # 数据表定义
        ADDRESS_VOLTAGE = 1  # 总线电压值的地址

        # 内存表读取
        # 注: 因为每个数据位数据格式各不相同
        # 因此读取得到的是字节流
        voltage_bytes = self.uservo_obj.read_data(SERVO_ID, ADDRESS_VOLTAGE)
        # 数据解析
        # 电压的数据格式为uint16_t,单位: mV
        # 关于struct的用法，请参阅官方手册: https://docs.python.org/3/library/struct.html
        voltage = struct.unpack('<H', voltage_bytes)
        logging.info("当前舵机ID: {}    电压: {:4.1f} °".format(SERVO_ID, voltage), end='\r')
        return voltage

    def read_all_servo_voltage(self, arm):
        # mode\power\angle\tempture\velo\acc
        """更新左右机械臂所有的舵机电压"""
        if arm in ["LEFT", "left", "Left"]:
            LEFT_ARM = [0, 1, 2]  # 左右机械臂舵机ID
        elif arm in ["Right", "RIGHT", "right"]:
            RIGHT_ARM = [3, 4, 5]
        arm_servo_voltage = [[]]
        left_arm_servo_voltage = []  # 查询结果
        right_arm_servo_voltage = []

        # time delay or waiting process has been applied in uservo.py member functions
        for SERVO_ID in LEFT_ARM:
            left_arm_servo_voltage.append(self.read_single_servo_voltage(SERVO_ID=SERVO_ID))

        for SERVO_ID in RIGHT_ARM:
            right_arm_servo_voltage.append(self.read_single_servo_voltage(SERVO_ID=SERVO_ID))

        # write the angle data of left and right arm
        arm_servo_voltage.append(left_arm_servo_voltage)
        arm_servo_voltage.append(right_arm_servo_voltage)

        return arm_servo_voltage

    def get_servo_status(self, SERVO_ID):
        '''打印舵机状态'''
        servo_status = {'Servo_id': SERVO_ID, 'voltage': 0.0, 'current': 0.0, 'power': 0.0, 'temp': 0.0}
        # 读取温度
        voltage = self.uservo_obj.query_voltage(SERVO_ID)
        # 读取电流
        current = self.uservo_obj.query_current(SERVO_ID)
        # 读取功率
        power = self.uservo_obj.query_power(SERVO_ID)
        # 读取温度
        temp = self.uservo_obj.query_temperature(SERVO_ID)

        # 写入舵机状态数据
        servo_status['voltage'] = voltage
        servo_status['current'] = current
        servo_status['power'] = power
        servo_status['temp'] = temp
        # 打印日志文件
        if self.is_debug:
            logging.info(
                "Servo idx {} Voltage: {:4.1f}V; Current: {:4.1f}A; Power: {:4.1f}W; T: {:2.0f}℃".format(SERVO_ID,
                                                                                                         voltage,
                                                                                                         current, power,
                                                                                                         temp))
        return servo_status

    def get_arm_dof(self):
        # get current robot arm dof info
        if self.__arm_name:
            return self.__arm_dof
        else:
            return 0

    def get_arm_servo_list(self):
        return self.__arm_servo_list

    def get_current_arm_name_str(self):
        return self.__arm_name
