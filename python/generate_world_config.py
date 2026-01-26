#!/usr/bin/python3
import sys
import pathlib
import argparse
import yaml

from irsl_choreonoid.robot_util import RobotModelWrapped as RobotModel
import irsl_choreonoid.cnoid_util as iu

def print_config(robot_name, bodyfile_path, offset, joint_names, controller_name, 
                 controller_type='effort', output=None):
    """
    Args:
        output (optional) : output file-stream. If None, sys.stdout is used
    """
    if output is None:
        output = sys.stdout
    num_joint = len(joint_names)
    world_config = {'robot':
                    {
                        'model': bodyfile_path,
                        'name': robot_name,
                        'initial_coords': {'pos': offset},
                        'initial_joint_angles': [0 for _ in range(num_joint)],
                        'fix': True,
                        'BodyROSItem': {'name_space': robot_name},
                        'ROSControlItem': {'name_space': robot_name}
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
                                         'name_space': robot_name,
                                         'controllers': [
                                             {
                                                 'name': controller_name,
                                                 'type': controller_type,
                                                 'P': 100,
                                                 'D': 0.5,
                                                 'joints': sorted([jn for jn in joint_names])
                                             }
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
    ###
    print(yaml.dump(world_config, indent=2, sort_keys=False), file=output)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        prog='generage world setting file',  # プログラム名
        usage='Generate world setting for choreonoidros',  # プログラムの利用方法
        add_help=True,  # -h/–help オプションの追加
    )
    parser.add_argument('--bodyfile', type=str, default="robotname.body")
    parser.add_argument('--robotname', type=str, default="")
    parser.add_argument('--offsetx', type=float, default="0.0")
    parser.add_argument('--offsety', type=float, default="0.0")
    parser.add_argument('--offsetz', type=float, default="0.0")
    parser.add_argument('--joint_controller_name', type=str, default="trajectory_controller")
    args = parser.parse_args()

    fname = str(args.bodyfile)
    rbody = iu.loadRobot(fname)
    robot = RobotModel(rbody)
    joint_names = robot.jointNames

    p = pathlib.Path(args.bodyfile)
    bodyfile_path = str(p.resolve())
    robot_name = args.robotname if args.robotname != "" else robot.robot.getModelName()

    print_config(robot_name, bodyfile_path, [args.offsetx, args.offsety, args.offsetz],
                 joint_names, args.joint_controller_name)
