#!/bin/python3
import argparse
import sys

from irsl_choreonoid.robot_util import RobotModelWrapped as RobotModel
import irsl_choreonoid.cnoid_util as iu

def print_config(joint_names, output=None):
    """
    Args:
        output (optional) : output file-stream. If None, sys.stdout is used
    """
    if output is None:
        output = sys.stdout
    for jointname in joint_names:
        print(f'- "{jointname}"', file=output)

if __name__=='__main__':
    parser = argparse.ArgumentParser(
            prog='generate_jointlist.py', # プログラム名
            usage='', # プログラムの利用方法
            add_help=True, # -h/–help オプションの追加
            )
    parser.add_argument('--bodyfile', type=str, default="robotname.body")
    
    args = parser.parse_args()
    fname = args.bodyfile

    rbody = iu.loadRobot(fname)
    robot = RobotModel(rbody)
    joint_names = robot.jointNames
    print_config(joint_names)
