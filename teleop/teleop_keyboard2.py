#!/usr/bin/env python
# -*- coding: utf-8 -*
# =====================================================================================
#        COPYRIGHT NOTICE
#        Copyright (c) 2013  HUST-Renesas Lab
#        ALL rights reserved.
#        
#        @file     teleop
#        @brief    robot keyboard control
#        @version  0.1
#        @date     2013/5/23 15:34:40
#        @author   Hu Chunxu , huchunxu@hust.edu.cn
# ==================================================================================
#  @0.1    Hu Chunxu    2013/5/23   create orignal file
# =====================================================================================
import  os
import  sys
import  tty, termios

import roslib; roslib.load_manifest('smartcar_teleop')
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# 全局变量
cmd = Twist()
pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

def keyboardLoop():
    #初始化
    rospy.init_node('smartcar_teleop')
    # Set rospy to exectute a shutdown function when exiting       
    rate = rospy.Rate(rospy.get_param('~hz', 10))

    #速度变量
    walk_vel_ = rospy.get_param('walk_vel', 0.2)
    run_vel_ = rospy.get_param('run_vel', 0.2)
    yaw_rate_ = rospy.get_param('yaw_rate', 0.05)
    yaw_rate_run_ = rospy.get_param('yaw_rate_run', 0.05)

    max_tv = walk_vel_
    max_rv = yaw_rate_

    #显示提示信息
    print "Reading from keyboard"
    print "Use WASD keys to control the robot"
    print "Press Caps to move faster"
    print "Press q to quit"

    #读取按键循环
    while not rospy.is_shutdown():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
		#不产生回显效果
        old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
        try :
            tty.setraw( fd )
            ch = sys.stdin.read( 1 )
        finally :
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        if ch == 'w':
            max_tv = walk_vel_
            speed = 1
            turn = 0
        elif ch == 's':
            max_tv = walk_vel_
            speed = -1
            turn = 0
        elif ch == 'a':
            max_rv = yaw_rate_
            speed = 0
            turn = 1
        elif ch == 'd':
            max_rv = yaw_rate_
            speed = 0
            turn = -1
        elif ch == 'W':
            max_tv = run_vel_
            speed = 1
            turn = 0
        elif ch == 'S':
            max_tv = run_vel_
            speed = -1
            turn = 0
        elif ch == 'A':
            max_rv = yaw_rate_run_
            speed = 0
            turn = 1
        elif ch == 'D':
            max_rv = yaw_rate_run_
            speed = 0
            turn = -1
        elif ch == 'q':
            exit()
        else:
            max_tv = walk_vel_
            max_rv = yaw_rate_
            speed = 0
            turn = 0

        #发送消息
        cmd.linear.x = speed * max_tv;
        cmd.angular.z = turn * max_rv;
        pub.publish(cmd)
        rate.sleep()
		#停止机器人
        #stop_robot();

def stop_robot():
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    pub.publish(cmd)

if __name__ == '__main__':
    try:
        keyboardLoop()
    except rospy.ROSInterruptException:
        pass
