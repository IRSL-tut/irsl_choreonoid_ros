#!/usr/bin/python3
import argparse
import numpy
import os
import sys

import irsl_choreonoid.cnoid_util as iu

from generate_utils import get_devicelist

def print_config(device_list, output=None):
    """
    Args:
        output (optional) : output file-stream. If None, sys.stdout is used
    """
    if output is None:
        output = sys.stdout
    print("I2CHubPublisher:")
    print("    'address': '0x70'")
    for idx, dev in enumerate(device_list):
        print("    '{}':".format(idx))
        print("        address: 'XXXX' # Input sensor address. color sensor is 0x70. Other sensor is 0x29.")
        print("        name: XXXX # input sensor type. ex. ColorSensorPublisher, TOFPublisher " )
        print("        topic_name: XXXX/value # Input sensor topic name")

if __name__=='__main__':
    parser = argparse.ArgumentParser(
            prog='generate controller config', # プログラム名
            usage='', # プログラムの利用方法
            add_help=True, # -h/–help オプションの追加
            )
    parser.add_argument('--bodyfile', type=str, default="robotname.body")
    
    args = parser.parse_args()
    fname = args.bodyfile
    rbody = iu.loadRobot(fname)
    
    rbody.updateLinkTree()
    rbody.initializePosition()
    rbody.calcForwardKinematics()
    
    device_list = get_devicelist(rbody)
    print_config(device_list)
    