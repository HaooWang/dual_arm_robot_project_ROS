#!usr/bin/env python
# -*- coding:utf-8 _*-
"""
@Time   :   2021/6/21  上午9:59
@Author :   HaoWANG, Foshan，China
@Email  :   haowanghk@163.com
@file   :  testing_demo.py.py
@Software : Pycharm
@Description:
"""

import uservo
import serial
import time

from MotionGroup import MotionGroup
from ServoPose import Pose
import logging

from wrist_gripper import WristGripperControl

from robot_arm import ArmMotionControl

POSE_NUM = 200  # default pose number

exitFlag = 0  # threading control flag

SHORT_TIME_DELAY = 0.02  # time delay para
MEDIA_TIME_DELAY = 0.05
LONG_TIME_DELAY = 0.1

VEL = {'Very_low_spped': 10, 'LOW_SPEED': 18, 'MID_SPEED': 45, 'HIGH_SPEED': 60}
CONTROL_MODE = {'Teach': True, 'Repeat': False}

DUAL_ARM_INIT_POSE = [140.0, 45.0, 155.0, 0, -4.6, -76.5, 171.0, 0, 0, 0]

ending_servo_idx = [0, 1, 2, 3]

MOTION_GROUP = [1, 2, 3, 4]


def generate_pose_vtc(angle, vel, servo_id, t_acc=35, t_dec=35, power=8000, mean_dps=80.0):
    pose_tmp = {'angle': angle, 'is_mturn': False, 'interval': 300,
                'velocity': vel, 't_acc': t_acc,
                't_dec': t_dec, 'power': power,
                'mean_dps': mean_dps}
    gen_pose = Pose(POSE=pose_tmp, SERVO_ID=servo_id)
    return gen_pose


def main(motion_group, teach_mode):
    pose_sd = {'Servo_id': 0, 'angle': None, 'is_mturn': False, 'interval': None, 'velocity': 10, 't_acc': 20,
               't_dec': 20,
               'power': 5000, 'mean_dps': 50.0}

    # data init

    left_arm_current_pose = [[] for i in range(POSE_NUM)]  # 视角轨迹点默认值100
    right_arm_current_pose = [[] for i in range(POSE_NUM)]  # 视角轨迹点默认值100

    # dual_arm
    dual_arm = ArmMotionControl(arm_name="dual_arm", dof=6, CONTROL_MODE="DAMPING", with_wrist=False, is_debug=True,
                                damping_power=1000)
    wrist_gripper = WristGripperControl(servo_channel=ending_servo_idx, is_debug=True)

    # 初始化舵机管理器类对象、串口端口地址
    dual_arm.creat_uservo_obj(SERVO_BAUDRATE=115200)
    if dual_arm.servo_buffer(is_scan_servo=True):
        dual_arm.logging_arm_info()
    dual_arm_servo = dual_arm.get_arm_servo_list()

    # dual arm and gripper init
    dual_arm.arm_state_init(force_angle_init=False)

    dual_arm.set_all_servo_mode(CONTROL_MODE="CIRCLE")
    wrist_gripper.all_ending_init(motion_group)
    time.sleep(2)

    if teach_mode == 1:
        # mode = 1 执行 teach
        # key = input('waitKey input (y or q):')
        # 记录
        if motion_group == 1:
            dual_arm.set_all_servo_mode(CONTROL_MODE="DAMPING")
            dual_arm_current_pose = MotionGroup(dual_arm, point_num=POSE_NUM, time_t=0.1, is_write_point=True,
                                                is_read_point=True, is_debug=False, group_id=1).write_motion_point()
        elif motion_group == 2:
            dual_arm.set_all_servo_mode(CONTROL_MODE="DAMPING")
            dual_arm_current_pose = MotionGroup(dual_arm, point_num=500, time_t=0.1, is_write_point=True,
                                                is_read_point=True, is_debug=False, group_id=2).write_motion_point()
        elif motion_group == 3:
            dual_arm.set_all_servo_mode(CONTROL_MODE="DAMPING")
            dual_arm_current_pose = MotionGroup(dual_arm, point_num=500, time_t=0.1, is_write_point=True,
                                                is_read_point=True, is_debug=False, group_id=3).write_motion_point()
        else:
            dual_arm.set_all_servo_mode(CONTROL_MODE="DAMPING")
            dual_arm_current_pose = MotionGroup(dual_arm, point_num=POSE_NUM, time_t=0.1, is_write_point=True,
                                                is_read_point=True, is_debug=False, group_id=4).write_motion_point()


    elif teach_mode == 0:
        if motion_group == 1:
            # motion group 1
            file_name = ["path_data_dual_arm_1_1.txt", "path_data_dual_arm_1_2.txt", "path_data_dual_arm_1_3.txt"]

            # 读取左右臂运动轨迹点数据
            traj_points_dual_arm, dual_arm_current_pose = MotionGroup(dual_arm, point_num=POSE_NUM, time_t=0.1,
                                                                      is_write_point=False,
                                                                      is_read_point=True, is_debug=False,
                                                                      group_id=1).read_motion_point(
                file_name_list=file_name)
        elif motion_group == 2:
            # motion group 2
            file_name = ["path_data_dual_arm_2_1.txt"]

            # 读取左右臂运动轨迹点数据
            traj_points_dual_arm, dual_arm_current_pose = MotionGroup(dual_arm, point_num=POSE_NUM, time_t=0.1,
                                                                      is_write_point=False,
                                                                      is_read_point=True,
                                                                      is_debug=False, group_id=2).read_motion_point(
                file_name_list=file_name)
        elif motion_group == 3:
            # motion group 2
            file_name = ["path_data_dual_arm_3_1.txt"]

            # 读取左右臂运动轨迹点数据
            traj_points_dual_arm, dual_arm_current_pose = MotionGroup(dual_arm, point_num=POSE_NUM, time_t=0.1,
                                                                      is_write_point=False,
                                                                      is_read_point=True,
                                                                      is_debug=False, group_id=3).read_motion_point(
                file_name_list=file_name)

        elif motion_group == 4:
            # motion group 4
            file_name = ["path_data_dual_arm_4_1.txt"]

            # 读取左右臂运动轨迹点数据
            traj_points_dual_arm, dual_arm_current_pose = MotionGroup(dual_arm, point_num=POSE_NUM, time_t=0.1,
                                                                      is_write_point=False,
                                                                      is_read_point=True,
                                                                      is_debug=False, group_id=4).read_motion_point(
                file_name_list=file_name)


    else:
        logging.error('ERROR! teach_mode == {}: NOT in [0,1]'.format(teach_mode))
        return

    while True:
        keyInput = input('waitKey input (c or x):')
        if keyInput == 'x':
            dual_arm.set_all_servo_mode(CONTROL_MODE="CIRCLE")
            logging.warning('Interrupt:  The waitKey input (c or x): {}'.format(keyInput))
            break
        elif keyInput == 'c':
            # dual_arm再现

            for it in dual_arm_current_pose[:10]:
                # wrist and gripper control
                 # trajectory control
                    for SERVO_ID in dual_arm_servo:
                        dual_arm.set_single_servo_pose(
                            Pose=generate_pose_vtc(angle=it[SERVO_ID], vel=VEL.get('Very_LOW_SPEED'),t_acc=1500,t_dec=800,power=5000,mean_dps=15, servo_id=SERVO_ID))
                    # 左臂手腕手爪位置伺服，0（IO5）-》it-3，1(IO6)->it-[8]
                    wrist_gripper.set_servo_angle(servo_idx=0, angle=it[3])
                    wrist_gripper.set_servo_angle(servo_idx=1, angle=it[8])

                    # 右臂手腕手爪位置伺服，2（IO12）-》it-7，3(IO13)->it-[9]
                    wrist_gripper.set_servo_angle(servo_idx=2, angle=it[7])
                    wrist_gripper.set_servo_angle(servo_idx=3, angle=it[9])
                    time.sleep(MEDIA_TIME_DELAY)
            for it in dual_arm_current_pose[10:]:

                    for SERVO_ID in dual_arm_servo:
                        dual_arm.set_single_servo_pose(
                            Pose=generate_pose_vtc(angle=it[SERVO_ID], vel=VEL.get('LOW_SPEED'),t_acc=30,t_dec=35,mean_dps=85, servo_id=SERVO_ID))
                    # 左臂手腕手爪位置伺服，0（IO5）-》it-3，1(IO6)->it-[8]
                    wrist_gripper.set_servo_angle(servo_idx=0, angle=it[3])
                    wrist_gripper.set_servo_angle(servo_idx=1, angle=it[8])

                    # 右臂手腕手爪位置伺服，2（IO12）-》it-7，3(IO13)->it-[9]
                    wrist_gripper.set_servo_angle(servo_idx=2, angle=it[7])
                    wrist_gripper.set_servo_angle(servo_idx=3, angle=it[9])
                    time.sleep(MEDIA_TIME_DELAY)
        else:
            logging.warning('waitKey input (c or x): {}'.format(keyInput))


# main function
if __name__ == '__main__':
    # 创建一个logger_debug
    import sys

    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)  # Log等级总开关-debug
    print("Logging Set Info: logger.setLevel(logging.DEBUG)  # Log等级总开关-debug")

    motion_group = int(sys.argv[1])
    teach_mode = int(sys.argv[2])

    if motion_group in MOTION_GROUP:
        try:
            main(motion_group, teach_mode)
        except KeyboardInterrupt:

            print("Shutting down robot arm debug.")
