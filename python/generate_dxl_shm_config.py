#!/bin/python3
import argparse
import sys

from irsl_choreonoid.robot_util import RobotModelWrapped as RobotModel

def print_config(joint_names, output=None):
    """
    Args:
        output (optional) : output file-stream. If None, sys.stdout is used
    """
    if output is None:
        output = sys.stdout
    text = f"""\
# This file is configlation for irsl_dynamixel_hardware_shm

dynamixel_hardware_shm:
  port_name: /dev/ttyUSB0
  baud_rate: 1000000
  joint:
"""
    print(text, file=output)
    for jointname in joint_names:
        print("    #{}:".format(jointname), file=output)
        print("    - { ID: XX, DynamixelSettings: { Return_Delay_Time: 0, Operating_Mode: 3 } }", file=output)
        

if __name__=='__main__':
    parser = argparse.ArgumentParser(
            prog='generate_irsl_shm_config.py', # プログラム名
            usage='', # プログラムの利用方法
            add_help=True, # -h/–help オプションの追加
            )
    parser.add_argument('--bodyfile', type=str, default="robotname.body")
    
    args = parser.parse_args()
    fname = args.bodyfile
    robot = RobotModel.loadModel(fname)
    joint_names = robot.jointNames

    print_config(joint_names)
