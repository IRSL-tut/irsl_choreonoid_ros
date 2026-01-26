#!/bin/python3
import argparse
import sys

import irsl_choreonoid.cnoid_util as iu
from generate_utils import get_jointnamelist

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
    parser.add_argument('--controller_name', type=str, default="trajectory_controller")

    args = parser.parse_args()
    fname = args.bodyfile
    rbody = iu.loadRobot(fname)

    joint_names = get_jointnamelist(rbody)
    ###
    print_config(joint_names, args.controller_name)
