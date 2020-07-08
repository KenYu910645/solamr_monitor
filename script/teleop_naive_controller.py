#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Ini.
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

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import sys, select, termios, tty

from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from math import pi as PI
import math 
import tf 

KP = 4
KI = 0.01
L = 0.76

msg = """
Control Your SOLamr!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly
swtich mode : 't' is crab steer, 'b' is heading adjustment
Switch between solamr 1 / solamr_ 2 
/ control both using same command : 1 / 2 / 3
CTRL-C to quit
"""
obstacle = {'a', 's', 'd', 'f'}

moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
           }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }

BLOCKER_OMEGA = 0.0525

speed = .5
turn = 1
x = 0
th = 0
status = 0
count = 0
target_speed = 0
target_turn = 0
control_speed = 0
control_turn = 0
mode = "crab_steer"
shelf_theta = 0.0
target_theta = 0.0

class RosTeleop:
    
    def __init__(self, bot_ns):
        self.robot_ns = bot_ns
        self.locker_ang = 0
        self.locker_dir = 0  # Locking direction, 1 = CW, -1 = CCW
        self.locker_state = False
        self.blocker_init = False
        self.vel_pub = rospy.Publisher('/cmd_vel_{0}'.format(self.robot_ns), Twist, queue_size=5)
        self.theta = 0.0 

    def imu_cb(self, theta):
        self.theta = theta.data


    def lockerAngleUpdate(self, angle):
        self.locker_ang = angle.process_value

    def lockAction(self, key_pressed):

        current_angle = float(round(self.locker_ang, 3) % PI)
        residual = 0
        #print("current angle : {0}".format(current_angle))

        
        if key_pressed == 't' :
            self.locker_dir = 0
            self.locker_state = True
            return self.locker_ang
         
        elif key_pressed == 'b' :
            self.locker_state = True
            if self.locker_dir == 0 :
                if not self.blocker_init:
                    self.blocker_init = True
                    self.locker_dir = 1
                else : 
                    self.locker_dir = (current_angle - PI/2)/abs(current_angle - PI/2)
            else : self.locker_dir *= -1  
        
        if self.locker_state : 
            if self.locker_dir == 1: 
                residual = PI - current_angle
            elif self.locker_dir == -1:
                residual = current_angle - 0

            if residual > BLOCKER_OMEGA:
                return self.locker_ang + self.locker_dir*BLOCKER_OMEGA
            else : 
                self.locker_state = False
                return self.locker_ang
        else : 
            return self.locker_ang

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def speedCtl(key_pressed):
    global x, th, status, count, target_speed, target_turn, control_speed, control_turn, speed, turn, mode 

    if key_pressed in moveBindings.keys():
        x = moveBindings[key_pressed][0]
        th = moveBindings[key_pressed][1]
        count = 0
    elif key_pressed in speedBindings.keys():
        speed = speed * speedBindings[key_pressed][0]
        turn = turn * speedBindings[key_pressed][1]
        count = 0

        # print(vels(speed,turn))
        if (status == 14):
            print(msg)
        status = (status + 1) % 15
    elif key_pressed == ' ' or key_pressed == 'k' :
        x = 0
        th = 0
        control_speed = 0
        control_turn = 0
    
    elif key_pressed == 't': # into heading adjustment mode 
        mode = "heading_adjustment"
    elif key_pressed == 'b': # into crab_steer
        mode = "crab_steer"


    else:
        count = count + 1
        if count > 4:
            x = 0
            th = 0
        if (key_pressed == '\x03'):
            return "break"

    target_speed = speed * x
    target_turn = turn * th

    if target_speed > control_speed:
        control_speed = min( target_speed, control_speed + 0.007 )
    elif target_speed < control_speed:
        control_speed = max( target_speed, control_speed - 0.007 )
    else:
        control_speed = target_speed

    if target_turn > control_turn:
        control_turn = min( target_turn, control_turn + 0.1 )
    elif target_turn < control_turn:
        control_turn = max( target_turn, control_turn - 0.1 )
    else:
        control_turn = target_turn

    twist = Twist()
    twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn

    return twist

def cal_theta_error(goal, current):
    '''
    '''
    error = goal - current # distance you need to chase 
    if abs(error) > PI: # too far to chase , change side 
        if current < 0:
            t1 = abs(-PI-current)
        else: 
            t1 = abs( PI-current)
        if goal < 0:
            t2 = abs(-PI-goal)
        else: 
            t2 = abs( PI-goal)
        error = math.copysign((t1+t2) , current)
        if abs(error) > PI:
            print ("[ERROR] UNEXPECTED THETA VALUE")
    return error

def shelf_odom_cb(odom):
    global shelf_theta 
    # transfer x,y,z,w to RPY
    quaternion = (
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    
    # print ("yaw : " + str(euler[2]))
    # print ("")
    shelf_theta = euler[2]

if __name__=="__main__":
    global target_theta

    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('naive_teleop', anonymous=False)
    
    solamr_1 = RosTeleop("_car1") 
    solamr_2 = RosTeleop("_car2") 
    try:
        print(msg)
        sum_error1 = 0.0
        sum_error2 = 0.0
        while(1):
            key = getKey()

            # SPEED CONTROL SECTION
            if speedCtl(key) == 'break': 
                break
            else: 
                twist = speedCtl(key)
            # print (twist)

            #--- parse twist -----# 
            # cmd_rot = 0.0 # for heading_adjustment only 
            if mode == "crab_steer":
                if twist.angular.z != 0: 
                    target_theta += twist.angular.z * 0.01 
            elif mode == "heading_adjustment":
                target_theta = shelf_theta + PI/2
            
            # normarlize target theta
            if target_theta > PI:
                target_theta -= PI*2
            elif target_theta < -PI:
                target_theta += PI*2

            #--- Chase target theta ---# 
            print ("target_theta: " + str(target_theta))
            print ("solamr_1.theta: " + str(solamr_1.theta))
            print ("solamr_2.theta: " + str(solamr_2.theta))
            print ("shelf_theta : " + str(shelf_theta))
            print ("sum_error1 : " + str(sum_error1))

            cmd_rot = 0.0
            if mode=="heading_adjustment":
                cmd_rot = twist.angular.z 
            
            error1 = cal_theta_error(target_theta, solamr_1.theta)
            sum_error1 += error1
            twist.angular.z = error1 * KP + sum_error1 * KI
            if mode=="heading_adjustment" :# and twist.angular.z < 0.1:
                twist.linear.x =    (L/2) * cmd_rot
            solamr_1.vel_pub.publish(twist)
            
            error2 = cal_theta_error(target_theta, solamr_1.theta)
            sum_error2 += error2
            twist.angular.z = error2 * KP + sum_error2 * KI
            if mode=="heading_adjustment" : # and twist.angular.z < 0.1:
                twist.linear.x =  - (L/2) * cmd_rot
            solamr_2.vel_pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        solamr_1.vel_pub.publish(twist)
        solamr_2.vel_pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
