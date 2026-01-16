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


def print_config(robot_name, model_file, joint_names, controller_name, use_mobile, devices, output=None):
    """
    Args:
        output (optional) : output file-stream. If None, sys.stdout is used
    """
    if output is None:
        import sys
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

    robot_name  = rbody.getModelName()
    model_file  = f'file:///{os.path.abspath(args.bodyfile)}'
    joint_names = [ j.jointName for j in joint_list if j.jointName not in args.wheeljoints ]
    use_mobile  = "" if args.use_wheel else "# "
    devices     = [ rbody.getDevice(idx) for idx in range(num_device) ]
    print_config(robot_name, model_file, joint_names, args.controller_name, use_mobile, devices)
