#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import numpy as np
import cv2
#import time

from geometry_msgs.msg import Twist
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

# BURGER_MAX_LIN_VEL = 1
# BURGER_MAX_ANG_VEL = 3

# WAFFLE_MAX_LIN_VEL = 0.26
# WAFFLE_MAX_ANG_VEL = 1.82

# LIN_VEL_STEP_SIZE = 0.1
# ANG_VEL_STEP_SIZE = 0.2

msg = """
Control Your TurtleBot3!
---------------------------
 
 ________________________________
|           Quit zone             |
|________________________________ |
|                |                |
|                |                |
|                |                |
|   Left zone    | Right zone     |
|________________|________________|

Forward: face in the middle and the face square heightand width smaller than that of the cam sceen
Backward: face in the middle and the face square heightand width bigger than that of the cam sceen
Rotate Left: entire face need to be in the left zone
Rotate Right: entire face need to be in the right zone
Stop: face undetected
 
Quit: your face need to reach the quit zone 
"""

e = """
Communications Failed
"""
face_cascede = cv2.CascadeClassifier('cascades/data/haarcascade_frontalface_alt2.xml')
camera_id = 0
cap = cv2.VideoCapture(camera_id)


# def getKey():
#     if os.name == 'nt':
#       if sys.version_info[0] >= 3:
#         return msvcrt.getch().decode()
#       else:
#         return msvcrt.getch()

#     tty.setraw(sys.stdin.fileno())
#     rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
#     if rlist:
#         key = sys.stdin.read(1)
#     else:
#         key = ''

#     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
#     return key

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

# def makeSimpleProfile(output, input, slop):
#     if input > output:
#         output = min( input, output + slop )
#     elif input < output:
#         output = max( input, output - slop )
#     else:
#         output = input

#     return output

# def constrain(input, low, high):
#     if input < low:
#       input = low
#     elif input > high:
#       input = high
#     else:
#       input = input

#     return input

# def checkLinearLimitVelocity(vel):
#     if turtlebot3_model == "burger":
#       vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
#     elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
#       vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
#     else:
#       vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

#     return vel

# def checkAngularLimitVelocity(vel):
#     if turtlebot3_model == "burger":
#       vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
#     elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
#       vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
#     else:
#       vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

#     return vel

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('turtlebot3_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    turtlebot3_model = rospy.get_param("model", "burger")

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    try:
        print(msg)
        while(1):  
            key = ''          
            ret, frame = cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascede.detectMultiScale(gray, scaleFactor=1.5, minNeighbors=5)
            #draw middle line
            (hi, wi) = frame.shape[:2]
            centertop = (int(wi/2), 0)
            centerbot = (int(wi/2), int(hi))
            limReg = (int(wi/2), int(hi/2))
            outlline = cv2.line(frame, (0, int(hi/10)), (wi, int(hi/10)), (255, 0, 0), 5)
            
            if len(faces) == 0:
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel))
            for (x, y, w, h) in faces:
            # print(x,y,w,h)
                color = (0, 255, 0)  # BGR 0-255
                stroke = 2
                end_cord_x = x + w
                end_cord_y = y + h
                cv2.rectangle(frame, (x, y), (end_cord_x, end_cord_y), color, stroke)        
                img = cv2.line(frame, centertop, centerbot, (0, 255, 0), 10)
                
                if (y < int(hi/10)):
                    key = '\x03' 
                elif (x < centertop[0]) & (end_cord_x < centertop[0]) & (y > int(hi/10)):
                    target_angular_vel = 0.5 #checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                    status = status + 1
                    print(vels(target_linear_vel,target_angular_vel))
                elif (x > centertop[0]) & (end_cord_x > centertop[0]) & (y > int(hi/10)):
                    target_angular_vel = -0.5 #checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                    status = status + 1
                    print(vels(target_linear_vel,target_angular_vel))
                elif (x < centertop[0]) & (end_cord_x > centertop[0]) & (y > int(hi/10)):
                    if (w < limReg[0]) & (h < limReg[1]):
                        target_linear_vel = 0.2 #checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                        status = status + 1
                        print(vels(target_linear_vel,target_angular_vel))
                    if (w > limReg[0]) & (h > limReg[1]):
                        target_linear_vel = -0.2 #checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                        status = status + 1
                        print(vels(target_linear_vel,target_angular_vel))
                
            if ret:
                cv2.imshow("Cam", frame)
            if cv2.waitKey(1) & (key == '\x03'):
                break   
                
            
            if status == 20 :
                print(msg)
                status = 0

            twist = Twist()

            control_linear_vel = target_linear_vel
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = target_angular_vel
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
            
            pub.publish(twist)
            #time.sleep(0.5)
            
        cap.release()
        cv2.destroyAllWindows()
    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
