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
            prog='cnoid_dump_model.py', # プログラム名
            usage='Demonstration of cnoid_dump_model', # プログラムの利用方法

            add_help=True, # -h/–help オプションの追加
            )
    parser.add_argument('--bodyfile', type=str, default="robotname.body")
    parser.add_argument('--controllername', type=str, default="joint_controller")
    parser.add_argument('--robotname', type=str, default="")
    parser.add_argument('--jointsuffix', type=str, default="")
    parser.add_argument('--wheeljoints', nargs="*", type=str, default=[])
    parser.add_argument('--wheelcontrollername', type=str, default="wheel_controller")

    args = parser.parse_args()
    fname = args.bodyfile
    controllername = args.controllername
    wheelcontrollername = args.wheelcontrollername
    joint_suffix = args.jointsuffix
    
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
    wheel_joint_list = []
    
    num_link = rbody.getNumLinks()
    num_joint = rbody.getNumJoints()

    for idx in range(num_joint):
        joint = rbody.getJoint(idx)
        if joint.jointName in args.wheeljoints:
            wheel_joint_list.append(joint)
        else :
            joint_list.append(joint)

    robotname = args.robotname if args.robotname != "" else rbody.getModelName()
    print("joint_state_controller:")
    print("  type: joint_state_controller/JointStateController")
    print("  publish_rate: 50")
    if len(joint_list)>0:
        print('%s:'%controllername)
        print('  type: "position_controllers/JointTrajectoryController"')
        print("  joints:")
        for joint in joint_list:
            print('    - %s%s'%(joint.jointName, joint_suffix))
        print("  gains:")
        for joint in joint_list:
            print('    %s%s:'%(joint.jointName, joint_suffix))
            print('      p: 100')
            print('      i: 10')
            print('      d: 1')
    if len(wheel_joint_list)>0:
        print('%s:'%wheelcontrollername)
        print('  type: "position_controllers/JointTrajectoryController"')
        print("  joints:")
        for joint in wheel_joint_list:
            print('    - %s%s'%(joint.jointName, joint_suffix))
        print("  gains:")
        for joint in wheel_joint_list:
            print('    %s%s:'%(joint.jointName, joint_suffix))
            print('      p: 100')
            print('      i: 10')
            print('      d: 1')
