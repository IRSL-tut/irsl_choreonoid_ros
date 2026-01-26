#!/bin/python3
import argparse
import os
import sys

from distutils.util import strtobool

from irsl_choreonoid.robot_util import RobotModelWrapped as RobotModel
import irsl_choreonoid.cnoid_util as iu

def print_config(robot_name, model_file, joint_names, controller_name, use_mobile, devices, output=None):
    """
    Args:
        output (optional) : output file-stream. If None, sys.stdout is used
    """
    if output is None:
        output = sys.stdout

    text=f"""\
robot_model:
  name: {robot_name}
  url: '{model_file}'

{use_mobile}mobile_base:
{use_mobile}  type: geometry_msgs/Twist
{use_mobile}  topic: /{robot_name}/cmd_vel
{use_mobile}  baselink: Root

joint_groups:
  -
    name: default
    topic: /{robot_name}/{controller_name}/command
    # type: 'action' or 'command'
    type: command
    joint_names: {joint_names}

devices:
  -
    topic: /{robot_name}/joint_states
    class: JointState
    name: joint_state
  -
    topic: /{robot_name}/{controller_name}/state
    class: JointTrajectoryState
    name: joint_trajectory_state
"""
    ### prints
    print(text, end='', file=output)

    ### devices
    for dev in devices:
        fmt = "std_msgs/Float64" if dev.getName().lower().find('color') < 0 else "std_msgs/ColorRGBA"
        dev_text = f"""\
  -
    topic: /{robot_name}/{dev.getName()}/value
    type: {fmt}
    name: {dev.getName()}
    rate: 10
"""
        print(dev_text, end='', file=output)

if __name__=='__main__':
    parser = argparse.ArgumentParser(
            prog='generate_ri_config.py', # プログラム名
            usage='Demonstration of cnoid_dump_model', # プログラムの利用方法

            add_help=True, # -h/–help オプションの追加
            )
    parser.add_argument('--bodyfile', type=str, default="robotname.body")
    parser.add_argument('--use_wheel', type=strtobool, default=False)
    parser.add_argument('--controller_name', type=str, default="trajectory_controller")

    args = parser.parse_args()
    fname = args.bodyfile
    rbody = iu.loadRobot(fname)
    robot = RobotModel(rbody)

    robot_name  = robot.robot.getModelName()
    model_file  = f'file:///{os.path.abspath(args.bodyfile)}'
    joint_names = robot.jointNames
    use_mobile  = "" if args.use_wheel else "# "
    devices     = robot.deviceList
    print_config(robot_name, model_file, joint_names, args.controller_name, use_mobile, devices)
