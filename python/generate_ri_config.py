#!/bin/python3
import argparse
import numpy
import os
import sys

from distutils.util import strtobool

try:
    import cnoid.Body
    import cnoid.Util
except ImportError:
    import sys
    import shutil
    choreonoid_bin_path = shutil.which('choreonoid')
    if choreonoid_bin_path is None:
        print('Error: choreonoid is not found.', file=sys.stderr)
        sys.exit(1)
    choreonoid_bin_dir_path = os.path.dirname(choreonoid_bin_path)
    choreonoid_share_path = os.path.join(choreonoid_bin_dir_path, '../share')
    chorenoid_ver = [dirname[dirname.find('choreonoid-')+len('choreonoid-'):] for dirname in os.listdir(choreonoid_share_path) if dirname.find('choreonoid-') != -1]
    if len(chorenoid_ver) > 0:
        chorenoid_ver = chorenoid_ver[0]
    else :
        chorenoid_ver = None
    choreonoid_python_path = os.path.join(choreonoid_bin_dir_path, '../lib/choreonoid-{}/python'.format(chorenoid_ver))
    print(choreonoid_python_path)
    if choreonoid_python_path is None or not os.path.exists(choreonoid_python_path):
        print('Error: choreonoid_python_path not found.', file=sys.stderr)
        sys.exit(1)
    sys.path.append(choreonoid_python_path)
    import cnoid.Body
    import cnoid.Util


if __name__=='__main__':
    parser = argparse.ArgumentParser(
            prog='generate_ri_config.py', # プログラム名
            usage='Demonstration of cnoid_dump_model', # プログラムの利用方法

            add_help=True, # -h/–help オプションの追加
            )
    parser.add_argument('--bodyfile', type=str, default="robotname.body")
    parser.add_argument('--use_wheel', type=strtobool, default=False)
    parser.add_argument('--controller_name', type=str, default="joint_controller")
    parser.add_argument('--wheeljoints', nargs="*", type=str, default=[])
    
    args = parser.parse_args()
    fname = args.bodyfile
    if not os.path.isfile(str(fname)):
        print("File is not exist.", file=sys.stderr)
        print("Please check file : {}".format(fname), file=sys.stderr)
        exit(1)
    rbody = cnoid.Body.BodyLoader().load(str(fname))
    if rbody is None:
        print("File is broken.", file=sys.stderr)
        print("Please check file : {}".format(fname), file=sys.stderr)
        exit(1)
    
    rbody.updateLinkTree()
    rbody.initializePosition()
    rbody.calcForwardKinematics()

    joint_list = []

    num_link = rbody.getNumLinks()
    num_joint = rbody.getNumJoints()
    num_device = rbody.getNumDevices()

    for idx in range(num_joint):
        joint = rbody.getJoint(idx)
        joint_list.append(joint)

    robotname = rbody.getModelName()
    
    print("robot_model:")
    print("  name: {}".format(robotname))
    print("  url: 'file:///{}'".format(os.path.abspath(args.bodyfile)))
    print("")
    print("{}mobile_base:".format("" if args.use_wheel else "# "))
    print("{}  type: geometry_msgs/Twist".format("" if args.use_wheel else "# "))
    print("{}  topic: /{}/cmd_vel".format("" if args.use_wheel else "# ", robotname))
    print("{}  baselink: Root".format("" if args.use_wheel else "# "))
    print("")
    print("joint_groups:")
    print("  -")
    print("    name: default")
    print("    topic: /{}/{}/command".format(robotname, args.controller_name))
    print("    # type: 'action' or 'command'")
    print("    type: command")
    print("    joint_names: {}".format([j.jointName for j in joint_list if j.jointName not in args.wheeljoints]))
    print("")
    print("devices:")
    print("  -")
    print("    topic: /{}/joint_states".format(robotname))
    print("    class: JointState")
    print("    name: joint_state")
    print("  -")
    print("    topic: /{}/{}/state".format(robotname, args.controller_name))
    print("    class: JointTrajectoryState")
    print("    name: joint_trajectory_state")
    for idx in range(num_device):
        dev = rbody.getDevice(idx)
        print("  -")
        print("    topic: /{}/{}/value".format(robotname, dev.getName()))
        print("    type: {}".format("std_msgs/Float64" if dev.getName().lower().find('color')<0 else "std_msgs/ColorRGBA"))
        print("    name: {}".format(dev.getName()))
        print("    rate: 10")
