#!/usr/bin/env python3
import numpy as np
import os

import rospy
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

import yaml

class WheelController:
    def __init__(self, wheelconfigfile):
        with open(wheelconfigfile) as f:
            self.wheelconfig = yaml.safe_load(f)

        controllername = self.wheelconfig["controller_name"]

        self.present_wheel_angle = [0.0 for wheel in self.wheelconfig["wheels"]]
        self.send_length = 10

        self.publisher = rospy.Publisher('{}/command'.format(controllername), JointTrajectory, queue_size=1)
        rospy.Subscriber("cmd_vel", Twist , self.com_vel_callback)
        rospy.Subscriber("joint_states", JointState, self.joint_callback)

    def joint_callback(self, data):
        for i, wheel in enumerate(self.wheelconfig["wheels"]):
            for j in range(len(data.name)):
                if data.name[j] == wheel["name"]:
                    self.present_wheel_angle[i] = data.position[j]

    def com_vel_callback(self, data):
        w_list = []
        for wheel in self.wheelconfig["wheels"]:
            v = data.linear.x * np.dot([0,1], wheel["rot_axis"][0:2]) + data.linear.y * np.dot([-1,0], wheel["rot_axis"][0:2])
            trans_xy = wheel["translation"][0:2]
            norm_trans_xy = np.linalg.norm(trans_xy)
            v -= norm_trans_xy*data.angular.z * np.dot(wheel["rot_axis"][0:2], trans_xy/norm_trans_xy)
            w = v/wheel["wheel_radius"]
            w_list.append(w)
        pub_msg = JointTrajectory()
        for wheel in self.wheelconfig["wheels"]:
            pub_msg.joint_names.append(wheel["name"])
        for j in range(1, self.send_length+1):
            point = JointTrajectoryPoint()
            for i,w in enumerate(w_list) :
                point.positions.append(w*j + self.present_wheel_angle[i])
            point.time_from_start=rospy.Time.from_sec(j)
            pub_msg.points.append(point)
        self.publisher.publish(pub_msg)

if __name__=='__main__':
    rospy.init_node("wheel_controller")
    wheelconfigfile = rospy.get_param('~wheelconfigfile')
    wc = WheelController(wheelconfigfile)

    rospy.spin()
