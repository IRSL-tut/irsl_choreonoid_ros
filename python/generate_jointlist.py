#!/bin/python3
import argparse
import sys

import irsl_choreonoid.cnoid_util as iu

from generate_utils import get_jointnamelist

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
    joint_names = get_jointnamelist(rbody)
    print_config(joint_names)
