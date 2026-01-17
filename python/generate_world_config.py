#!/usr/bin/python3
import pathlib
import os
import argparse
import yaml
import sys

try:
    import cnoid.Body
    import cnoid.Util
    import cnoid.IRSLCoords
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
    import cnoid.IRSLCoords

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        prog='generage world setting file',  # プログラム名
        usage='Generate world setting for choreonoidros',  # プログラムの利用方法
        add_help=True,  # -h/–help オプションの追加
    )
    parser.add_argument('bodyfile', type=str, default="robotname.body")
    parser.add_argument('--robotname', type=str, default="")
    parser.add_argument('--offsetx', type=float, default="0.0")
    parser.add_argument('--offsety', type=float, default="0.0")
    parser.add_argument('--offsetz', type=float, default="0.0")
    parser.add_argument('--joint_controller_name', type=str, default="joint_controller")
    parser.add_argument('--wheel_controller_name', type=str, default="wheel_controller")
    parser.add_argument('--wheeljoints', nargs="*", type=str, default=[])
    args = parser.parse_args()

    fname = str(args.bodyfile)
    if not os.path.isfile(str(fname)):
        print("File is not exist.", file=sys.stderr)
        print("Please check file : {}".format(fname), file=sys.stderr)
        exit(1)

    rbody = cnoid.Body.BodyLoader().load(str(args.bodyfile))
    if rbody is None:
        print("File is broken.", file=sys.stderr)
        print("Please check file : {}".format(fname), file=sys.stderr)
        exit(1)

    rbody.updateLinkTree()
    rbody.initializePosition()
    rbody.calcForwardKinematics()
    
    root_pose = rbody.getRootLink().getPosition()
    root_position = root_pose[0:3,3].tolist()
    root_orientation = cnoid.IRSLCoords.coordinates(root_pose).getRotationAngle().tolist()

    joint_list = []

    num_link = rbody.getNumLinks()
    num_joint = rbody.getNumJoints()
    num_device = rbody.getNumDevices()

    for idx in range(num_joint):
        joint = rbody.getJoint(idx)
        joint_list.append(joint)
    
    p = pathlib.Path(args.bodyfile)
    bodyfile_path = str(p.resolve())
    robotname = args.robotname if args.robotname != "" else rbody.getModelName()

    joint_controller_joints = sorted([j.jointName for j in joint_list if j.jointName not in args.wheeljoints])
    wheel_controller_joints = sorted([j.jointName for j in joint_list if j.jointName in args.wheeljoints])

    world_config = {'robot':
                    {
                        'model': bodyfile_path,
                        'name': robotname,
                        'initial_coords': {'pos': root_position,
                                           'aa': root_orientation},
                        'initial_joint_angles': [joint.q for joint in joint_list],
                        'fix': True if len(wheel_controller_joints)==0 else False,
                        'BodyROSItem': {'name_space': robotname},
                        'ROSControlItem': {'name_space': robotname}
                    },
                    'world':
                    {
                        'World': {'name': 'MyWorld'},
                        'Simulator': {
                            'type': 'AISTSimulator',
                            'name': 'AISTSim'
                        },
                        'GLVision': None,
                        'Camera': {
                            'lookEye': [0.0, 3.0, 1.7],
                            'lookUp': [0.0, 0.0, 1.0],
                            'lookAtCenter': [0.0, 0.0, 0.7]
                        },
                        'WorldROS': None,
                        'ROS': {
                            'generate': {'robot': bodyfile_path,
                                         'name_space': robotname,
                                         'controllers': [
                                         ]
                                         }
                        }
                    },
                    'objects': [
                        {
                            'model': 'choreonoid://share/model/misc/floor.body',
                            'name': 'MyFloor',
                            'fix': True
                        }
                    ]
                    }
    
    if len(joint_controller_joints) > 0:
        world_config['world']['ROS']['generate']['controllers'].append({'name': args.joint_controller_name, 'type': 'position', 'joints': joint_controller_joints })
    if len(wheel_controller_joints) > 0:
        world_config['world']['ROS']['generate']['controllers'].append({'name':args.wheel_controller_name, 'type':'position', 'joints': wheel_controller_joints })
    print(yaml.dump(world_config, indent=2, sort_keys=False))
