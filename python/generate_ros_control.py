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

def list_string_esc(lst, offset=""):
    res = ""
    for l in lst[:-1]:
        res += '{}- "{}"\n'.format(offset, l)
    res += '{}- "{}"'.format(offset, lst[-1])
    return res
def list_string(lst, offset=""):
    res = ""
    for l in lst[:-1]:
        res += '{}- {}\n'.format(offset, l)
    res += '{}- {}'.format(offset, lst[-1])
    return res

def print_config(joint_names, controller_name, output=None):
    """
    Args:
        output (optional) : output file-stream. If None, sys.stdout is used
    """
    if output is None:
        import sys
        output = sys.stdout
    str_joints = list_string_esc(joint_names, offset='    ')
    text=f"""\
joint_state_controller:
  type: joint_state_controller/JointStateController
  publish_rate: 50
  joints:
{str_joints}
{controller_name}:
  type: position_controllers/JointTrajectoryController
  joints:
{str_joints}
"""
    ###
    print(text, end='', file=output)

if __name__=='__main__':
    parser = argparse.ArgumentParser(
            prog='generate_ros_control.py', # プログラム名
            usage='', # プログラムの利用方法
            add_help=True, # -h/–help オプションの追加
            )
    parser.add_argument('--bodyfile', type=str, default="robotname.body")
    parser.add_argument('--controller_name', type=str, default="joint_controller")

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

    num_joint = rbody.getNumJoints()
    joint_list = []
    for idx in range(num_joint):
        joint = rbody.getJoint(idx)
        joint_list.append(joint)
    joint_names = [j.jointName for j in joint_list]

    ###
    print_config(joint_names, args.controller_name)
