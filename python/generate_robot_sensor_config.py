#!/usr/bin/python3
import argparse
import numpy
import os
import sys

from irsl_choreonoid.robot_util import RobotModelWrapped as RobotModel


def print_config(devices, output=None):
    """
    Args:
        output (optional) : output file-stream. If None, sys.stdout is used
    """
    if output is None:
        output = sys.stdout
    print("I2CHubPublisher:", file=output)
    print("    'address': '0x70'", file=output)
    for idx, dev in enumerate(devices):
        print("    '{}':".format(idx), file=output)
        print("        address: 'XXXX' # Input sensor address. color sensor is 0x70. Other sensor is 0x29.", file=output)
        print("        name: XXXX # input sensor type. ex. ColorSensorPublisher, TOFPublisher ", file=output)
        print("        topic_name: XXXX/value # Input sensor topic name", file=output)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="generate controller config",  # プログラム名
        usage="",  # プログラムの利用方法
        add_help=True,  # -h/–help オプションの追加
    )
    parser.add_argument("--bodyfile", type=str, default="robotname.body")

    args = parser.parse_args()
    fname = args.bodyfile
    robot = RobotModel.loadModel(fname)

    devices = robot.deviceList
    print_config(devices)
