#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from AirSimClient import *
from std_msgs.msg import String
from dbw_mkz_msgs.msg import SteeringCmd
from dbw_mkz_msgs.msg import ThrottleCmd
from dbw_mkz_msgs.msg import BrakeCmd
from dbw_mkz_msgs.msg import GearCmd
from dbw_mkz_msgs.msg import SteeringReport
from dbw_mkz_msgs.msg import GearReport

from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

# connect to the AirSim simulator 
client = CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = CarControls()
lastThrottle = 0
throttleChangeMargin = 0.01

# Dataspeed msg definitions:
# http://docs.ros.org/indigo/api/dbw_mkz_msgs/html/index-msg.html

def steering_cb(steering_cmd_msg):
    rospy.loginfo(rospy.get_caller_id() + "Steering cmd: %s", steering_cmd_msg.steering_wheel_angle_cmd)
    car_controls.steering = steering_cmd_msg.steering_wheel_angle_cmd / 8.2

def throttle_cb(throttle_cmd_msg):
    # Assuming the ros message is a percent type
    rospy.loginfo(rospy.get_caller_id() + "Throttle cmd: %s", throttle_cmd_msg.pedal_cmd)
    lastThrottle = car_controls.throttle
    if (lastThrottle > throttleChangeMargin and throttle_cmd_msg.pedal_cmd == 0.0):
        pass
    else:
        car_controls.throttle = throttle_cmd_msg.pedal_cmd * 1000.0

def brake_cb(brake_cmd_msg):
    # Assuming the brake command is torque type, in Nm
    rospy.loginfo(rospy.get_caller_id() + "Brake cmd: %s", brake_cmd_msg.pedal_cmd)
    car_controls.brake = brake_cmd_msg.pedal_cmd / 3250.0

def gear_cb(gear_cmd_msg):
    rospy.loginfo(rospy.get_caller_id() + "Gear cmd: %s", gear_cmd_msg.cmd.gear)
    car_controls.gear = gear_cmd_msg.cmd.gear

def bridge():
    # Publishers
    imu_pub = rospy.Publisher('imu/data', Imu, queue_size=1)
    gps_pub = rospy.Publisher('navsat/fix', NavSatFix, queue_size=1)
    steering_report_pub = rospy.Publisher('vehicle/steering_report', SteeringReport, queue_size=1)
    gear_report_pub = rospy.Publisher('vehicle/gear_report', GearReport, queue_size=1)
    
    # Subscribers
    rospy.Subscriber("vehicle/steering_cmd", SteeringCmd, steering_cb)
    rospy.Subscriber("vehicle/throttle_cmd", ThrottleCmd, throttle_cb)
    rospy.Subscriber("vehicle/brake_cmd", BrakeCmd, brake_cb)
    rospy.Subscriber("vehicle/gear_cmd", GearCmd, gear_cb)
    
    rospy.init_node('airsim_bridge', anonymous=True)

    rate = rospy.Rate(50) # 50hz
    while not rospy.is_shutdown():
        # Get a sensor report
        car_state = client.getCarState()
        
        # update the controls
        client.setCarControls(car_controls)

        # Gear report 
        print("Gear %d" % (car_state.gear))
        # GearReport gear_report_msg 
        # gear_report_msg.state.gear = 
        
        rate.sleep()

if __name__ == '__main__':
    try:
        bridge()
    except rospy.ROSInterruptException:
        pass
