# -*- coding: UTF-8 -*-
from PCA9685 import PCA9685 #导入驱动，
import time
pwm=PCA9685(0x40)#设置模块的地址，默认0x40
pwm.setsq(50)#设置频率
pwm.init()#初始化pca9685
pwm.allinit()#把16个通道初始化


# 机械臂载荷测试，手爪关闭角度122，手腕垂直140-145
# 2021年9月7日，下午第一次测试，左ID 0，1 右ID 2，3
# 2021年9月13日，改进舵机参数及曲柄长度参数，重新测试
angle=120
servo_pin=3
pwm.setangle(1,angle)
pwm.setangle(3,122)
pwm.setangle(2,143)
while 1:
    a=input("input:")
    a=int(a)
    #pwm.setpwm(0,0,a)
    pwm.setangle(servo_pin,a)
    time.sleep(1)
    
