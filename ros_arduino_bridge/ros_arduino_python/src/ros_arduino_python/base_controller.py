#!/usr/bin/env python

"""
    A base controller class for the Arduino microcontroller
    
    Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses
"""
import roslib; roslib.load_manifest('ros_arduino_python')
import rospy
import os

from math import sin, cos, pi
from geometry_msgs.msg import Quaternion, Twist, Pose
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int32
 
""" Class to receive Twist commands and publish Odometry data """
class BaseController:
    def __init__(self, arduino, base_frame, name="base_controllers"):
        self.arduino = arduino
        self.name = name
        self.base_frame = base_frame
        self.rate = float(rospy.get_param("~base_controller_rate", 10))
        self.timeout = rospy.get_param("~base_controller_timeout", 0.7)
        self.stopped = False
        self.debugPID=True
        self.odom_angular_scale_correction=1.85
        self.odom_linear_scale_correction = 1.01
 
        pid_params = dict()
        pid_params['wheel_diameter'] = rospy.get_param("~wheel_diameter", "") 
        pid_params['wheel_track'] = rospy.get_param("~wheel_track", "")
        pid_params['encoder_resolution'] = rospy.get_param("~encoder_resolution", "") 
        pid_params['gear_reduction'] = rospy.get_param("~gear_reduction", 1.0)
        pid_params['Aleft_Kp'] = rospy.get_param("~Aleft_Kp", 20)
        pid_params['Aleft_Kd'] = rospy.get_param("~Aleft_Kd", 12)
        pid_params['Aleft_Ki'] = rospy.get_param("~Aleft_Ki", 0)
        pid_params['Aleft_Ko'] = rospy.get_param("~Aleft_Ko", 50)
        pid_params['Aright_Kp'] = rospy.get_param("~Aright_Kp", 20)
        pid_params['Aright_Kd'] = rospy.get_param("~Aright_Kd", 12)
        pid_params['Aright_Ki'] = rospy.get_param("~Aright_Ki", 0)
        pid_params['Aright_Ko'] = rospy.get_param("~Aright_Ko", 50)
        pid_params['Bleft_Kp'] = rospy.get_param("~Bleft_Kp", 20)
        pid_params['Bleft_Kd'] = rospy.get_param("~Bleft_Kd", 12)
        pid_params['Bleft_Ki'] = rospy.get_param("~Bleft_Ki", 0)
        pid_params['Bleft_Ko'] = rospy.get_param("~Bleft_Ko", 50)
        pid_params['Bright_Kp'] = rospy.get_param("~Bright_Kp", 20)
        pid_params['Bright_Kd'] = rospy.get_param("~Bright_Kd", 12)
        pid_params['Bright_Ki'] = rospy.get_param("~Bright_Ki", 0)
        pid_params['Bright_Ko'] = rospy.get_param("~Bright_Ko", 50)
        
        self.accel_limit = rospy.get_param('~accel_limit', 1)
        self.motors_reversed = rospy.get_param("~motors_reversed", False)
        
        if self.debugPID:
            self.AleftEncoderPub = rospy.Publisher('AleftEncoder', Int32, queue_size=10)
            self.ArightEncoderPub = rospy.Publisher('ArightEncoder', Int32, queue_size=10)
            self.BleftEncoderPub = rospy.Publisher('BleftEncoder', Int32, queue_size=10)
            self.BrightEncoderPub = rospy.Publisher('BrightEncoder', Int32, queue_size=10)
            self.AleftPidoutPub = rospy.Publisher('AleftPidout', Int32, queue_size=10)
            self.ArightPidoutPub = rospy.Publisher('ArightPidout', Int32, queue_size=10)
            self.BleftPidoutPub = rospy.Publisher('BleftPidout', Int32, queue_size=10)
            self.BrightPidoutPub = rospy.Publisher('BrightPidout', Int32, queue_size=10)
            self.AleftVelPub = rospy.Publisher('AleftVel', Int32, queue_size=10)
            self.ArightVelPub = rospy.Publisher('ArightVel', Int32, queue_size=10)
            self.BleftVelPub = rospy.Publisher('BleftVel', Int32, queue_size=10)
            self.BrightVelPub = rospy.Publisher('BrightVel', Int32, queue_size=10)
       
        # Set up PID parameters and check for missing values
        self.setup_pid(pid_params)
            
        # How many encoder ticks are there per meter?
        self.ticks_per_meter = self.encoder_resolution * self.gear_reduction  / (self.wheel_diameter * pi) 
        self.ticks_per_meter= self.ticks_per_meter / self.odom_linear_scale_correction
        # What is the maximum acceleration we will tolerate when changing wheel speeds?
        self.max_accel = self.accel_limit * self.ticks_per_meter / self.rate
                
        # Track how often we get a bad encoder count (if any)
        self.bad_encoder_count = 0
                        
        now = rospy.Time.now()    
        self.then = now # time for determining dx/dy
        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = now + self.t_delta

        # Internal data        
        self.enc_Aleft = None            # encoder readings
        self.enc_Aright = None
        self.enc_Bleft = None            
        self.enc_Bright = None
        
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0                     # rotation in radians

        self.v_Aleft = 0
        self.v_Aright = 0
        self.v_Bleft = 0
        self.v_Bright = 0

        self.v_des_Aleft = 0             # cmd_vel setpoint
        self.v_des_Aright = 0
        self.v_des_Bleft = 0             
        self.v_des_Bright = 0

        self.last_cmd_vel = now

        # Subscriptions
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)
        
        # Clear any old odometry info
        self.arduino.reset_encoders()
        
        # Set up the odometry broadcaster
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=5)
        self.odomBroadcaster = TransformBroadcaster()
        
        rospy.loginfo("Started base controller for a base of " + str(self.wheel_track) + "m wide with " + str(self.encoder_resolution) + " ticks per rev")
        rospy.loginfo("Publishing odometry data at: " + str(self.rate) + " Hz using " + str(self.base_frame) + " as base frame")
        
    def setup_pid(self, pid_params):
        # Check to see if any PID parameters are missing
        missing_params = False
        for param in pid_params:
            if pid_params[param] == "":
                print("*** PID Parameter " + param + " is missing. ***")
                missing_params = True
        
        if missing_params:
            os._exit(1)
                
        self.wheel_diameter = pid_params['wheel_diameter']
        #scale wheel_track
        self.wheel_track = pid_params['wheel_track']
        self.wheel_track = self.wheel_track / self.odom_angular_scale_correction
        
        self.encoder_resolution = pid_params['encoder_resolution']
        self.gear_reduction = pid_params['gear_reduction']
        
        self.Aleft_Kp = pid_params['Aleft_Kp']
        self.Aleft_Kd = pid_params['Aleft_Kd']
        self.Aleft_Ki = pid_params['Aleft_Ki']
        self.Aleft_Ko = pid_params['Aleft_Ko']
        self.Aright_Kp = pid_params['Aleft_Kp']
        self.Aright_Kd = pid_params['Aleft_Kd']
        self.Aright_Ki = pid_params['Aleft_Ki']
        self.Aright_Ko = pid_params['Aleft_Ko']
        self.Bleft_Kp = pid_params['Bleft_Kp']
        self.Bleft_Kd = pid_params['Bleft_Kd']
        self.Bleft_Ki = pid_params['Bleft_Ki']
        self.Bleft_Ko = pid_params['Bleft_Ko']
        self.Bright_Kp = pid_params['Aleft_Kp']
        self.Bright_Kd = pid_params['Aleft_Kd']
        self.Bright_Ki = pid_params['Aleft_Ki']
        self.Bright_Ko = pid_params['Aleft_Ko']
 

        
        self.arduino.update_pid(self.Aleft_Kp, self.Aleft_Kd, self.Aleft_Ki, self.Aleft_Ko,self.Aright_Kp, self.Aright_Kd, self.Aright_Ki, self.Aright_Ko,self.Bleft_Kp, self.Bleft_Kd, self.Bleft_Ki, self.Bleft_Ko,self.Bright_Kp, self.Bright_Kd, self.Bright_Ki, self.Bright_Ko)

    def poll(self):
        if self.debugPID:
            try:
                AleftPidin,ArightPidin,BleftPidin,BrightPidin=self.arduino.get_pidin()
                self.AleftEncoderPub.publish(AleftPidin)
                self.ArightEncoderPub.publish(ArightPidin)
                self.BleftEncoderPub.publish(BleftPidin)
                self.BrightEncoderPub.publish(BrightPidin)
            except:
                rospy.logerr("getPidin except count:")
                return

            try:
                AleftPidout,ArightPidout,BleftPidout,BrightPidout=self.arduino.get_pidout()
                self.AleftPidoutPub.publish(AleftPidout)
                self.ArightPidoutPub.publish(ArightPidout)
                self.BleftPidoutPub.publish(BleftPidout)
                self.BrightPidoutPub.publish(BrightPidout)
            except:
                rospy.logerr("getPindout except count:")
                return

        now = rospy.Time.now()
        if now > self.t_next:
            # Read the encoders
            try:
                Aleft_enc, Aright_enc,Bleft_enc, Bright_enc = self.arduino.get_encoder_counts()
            except:
                self.bad_encoder_count += 1
                rospy.logerr("Encoder exception count: " + str(self.bad_encoder_count))
                return
                            
            dt = now - self.then
            self.then = now
            dt = dt.to_sec()
            
            # Calculate odometry
            if self.enc_Aleft == None:
                dAright = 0
                dAleft = 0
                dBright = 0
                dBleft = 0
            else:
                dAright = (Aright_enc - self.enc_Aright) / self.ticks_per_meter
                dAleft = (Aleft_enc - self.enc_Aleft) / self.ticks_per_meter
                dBright = (Bright_enc - self.enc_Bright) / self.ticks_per_meter
                dBleft = (Bleft_enc - self.enc_Bleft) / self.ticks_per_meter

            dright=dAright
            dleft=dAleft

            self.enc_Aright = Aright_enc
            self.enc_Aleft = Aleft_enc
            self.enc_Bright = Bright_enc
            self.enc_Bleft = Bleft_enc
            
            dxy_ave = (dright + dleft) / 2.0
            dth = (dright - dleft) / self.wheel_track 
            vxy = dxy_ave / dt
            vth = dth / dt
                
            if (dxy_ave != 0):
                dx = cos(dth) * dxy_ave
                dy = -sin(dth) * dxy_ave
                self.x += (cos(self.th) * dx - sin(self.th) * dy)
                self.y += (sin(self.th) * dx + cos(self.th) * dy)
    
            if (dth != 0):
                self.th += dth 
    
            quaternion = Quaternion()
            quaternion.x = 0.0 
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2.0)
            quaternion.w = cos(self.th / 2.0)
           # quaternion = createQuaternionMsgFromRollPitchYaw(0,0,self.th)


    
            # Create the odometry transform frame broadcaster.
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0), 
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame,
                "odom"
                )
    
            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.child_frame_id = self.base_frame
            odom.header.stamp = now
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.twist.twist.linear.x = vxy
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = vth

            self.odomPub.publish(odom)
            
            if now > (self.last_cmd_vel + rospy.Duration(self.timeout)):
                self.v_des_Aleft = 0
                self.v_des_Aright = 0
                self.v_des_Bleft = 0
                self.v_des_Bright = 0
                
            if self.v_Aleft < self.v_des_Aleft:
                self.v_Aleft += self.max_accel
                if self.v_Aleft > self.v_des_Aleft:
                    self.v_Aleft = self.v_des_Aleft
            else:
                self.v_Aleft -= self.max_accel
                if self.v_Aleft < self.v_des_Aleft:
                    self.v_Aleft = self.v_des_Aleft
            
            if self.v_Aright < self.v_des_Aright:
                self.v_Aright += self.max_accel
                if self.v_Aright > self.v_des_Aright:
                    self.v_Aright = self.v_des_Aright
            else:
                self.v_Aright -= self.max_accel
                if self.v_Aright < self.v_des_Aright:
                    self.v_Aright = self.v_des_Aright

            if self.v_Bleft < self.v_des_Bleft:
                self.v_Bleft += self.max_accel
                if self.v_Bleft > self.v_des_Bleft:
                    self.v_Bleft = self.v_des_Bleft
            else:
                self.v_Bleft -= self.max_accel
                if self.v_Bleft < self.v_des_Bleft:
                    self.v_Bleft = self.v_des_Bleft

            if self.v_Bright < self.v_des_Bright:
                self.v_Bright += self.max_accel
                if self.v_Bright > self.v_des_Bright:
                    self.v_Bright = self.v_des_Bright
            else:
                self.v_Bright -= self.max_accel
                if self.v_Bright < self.v_des_Bright:
                    self.v_Bright = self.v_des_Bright

            # Set motor speeds in encoder ticks per PID loop
            if not self.stopped:
                self.arduino.drive(self.v_Aleft, self.v_Aright,self.v_Bleft, self.v_Bright)
                if self.debugPID:
                    self.AleftVelPub.publish(self.v_Aleft)
                    self.ArightVelPub.publish(self.v_Aright)
                    self.BleftVelPub.publish(self.v_Bleft)
                    self.BrightVelPub.publish(self.v_Bright)

            self.t_next = now + self.t_delta
            
    def stop(self):
        self.stopped = True
        self.arduino.drive(0, 0, 0, 0)
            
    def cmdVelCallback(self, req):
        # Handle velocity-based movement requests
        self.last_cmd_vel = rospy.Time.now()
        
        x = req.linear.x         # m/s
        th = req.angular.z       # rad/s

        if x == 0:
            # Turn in place
            Aright = th * self.wheel_track *10 / 2.0
            Bright=Aright
            Aleft = -Aright
            Bleft=Aleft
        elif th == 0:
            # Pure forward/backward motion
            Aleft = Aright = x
            Bleft = Bright = x
        else:
            # Rotation about a point in space
            Aleft = x - th * self.wheel_track * 10  / 2.0
            Bleft=Aleft
            Aright = x + th * self.wheel_track * 10 / 2.0
            Bright=Aright
            
        self.v_des_Aleft = int(Aleft * self.ticks_per_meter / self.arduino.PID_RATE)
        self.v_des_Aright = int(Aright * self.ticks_per_meter / self.arduino.PID_RATE)
        self.v_des_Bleft=int(Bleft * self.ticks_per_meter / self.arduino.PID_RATE)
        self.v_des_Bright=int(Bright * self.ticks_per_meter / self.arduino.PID_RATE)
