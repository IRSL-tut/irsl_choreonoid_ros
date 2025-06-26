#!/usr/bin/python3
import argparse
import numpy
import os
import sys

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
            prog='generate controller config', # プログラム名
            usage='', # プログラムの利用方法
            add_help=True, # -h/–help オプションの追加
            )
    parser.add_argument('--bodyfile', type=str, default="robotname.body")
    
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
    
    num_device = rbody.getNumDevices()

    print("I2CHubPublisher:")
    print("    'address': '0x70'")
    for idx in range(num_device):
        dev = rbody.getDevice(idx)
        print("    '{}':".format(idx))
        print("        address: 'XXXX' # Input sensor address. color sensor is 0x70. Other sensor is 0x29.")
        print("        name: XXXX # input sensor type. ex. ColorSensorPublisher, TOFPublisher " )
        print("        topic_name: XXXX/value # Input sensor topic name")