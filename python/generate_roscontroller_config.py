import argparse
import numpy
import os

try:
    import cnoid.Body
    import cnoid.Util
except ImportError:
    import sys
    import shutil
    choreonoid_path = os.path.join(os.path.dirname(shutil.which('choreonoid')), '../lib/choreonoid-1.8/python') if shutil.which('choreonoid') is not None else None
    if choreonoid_path is None:
        print('Error: choreonoid_path not found.', file=sys.stderr)
        sys.exit(1)
    sys.path.append(choreonoid_path)
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

    args = parser.parse_args()
    fname = args.bodyfile
    controllername = args.controllername
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

    num_link = rbody.getNumLinks()
    num_joint = rbody.getNumJoints()

    for idx in range(num_joint):
        joint = rbody.getJoint(idx)
        joint_list.append(joint)

    robotname = args.robotname if args.robotname != "" else rbody.getModelName()
    if len(joint_list)>0:
        print('%s:'%robotname)
        print('  %s:'%controllername)
        print('    type: "position_controllers/JointTrajectoryController"')
        print("    joints:")
        for joint in joint_list:
            print('      - %s%s'%(joint.getName(), joint_suffix))
        print("    gains:")
        for joint in joint_list:
            print('      %s%s:'%(joint.getName(), joint_suffix))
            print('        p: 100')
            print('        i: 10')
            print('        d: 1')
