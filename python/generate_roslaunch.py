#!/usr/bin/python3
import argparse
import numpy
import os
import sys

from distutils.util import strtobool

from irsl_choreonoid.robot_util import RobotModelWrapped as RobotModel


def print_config(robotname, urdffile,  controllers, devices, demo_base_dir, worldsettings, output=None):
    """
    Args:
        output (optional) : output file-stream. If None, sys.stdout is used
    """
    if output is None:
        output = sys.stdout

    output_str = f"""
<launch>
  <arg name="demo_base_dir" default="{demo_base_dir}"/>
  <!-- choreonoid -->
  <arg name="worldsettings" default="{worldsettings}" />
  <arg name="robot_name" default="{robotname}"/>'
  <!-- ros_control -->
  <arg name="model" default="$(arg demo_base_dir)/{urdffile}"/>
  <arg name="controllers" default="{controllers}" />

  <include file="$(find irsl_choreonoid_ros)/launch/run_sim_with_setup_cnoid.launch">
    <arg name="controllers" value="$(arg controllers)" />
    <arg name="setup_cnoid" value="$(arg demo_base_dir)/$(arg worldsettings)" />
    <arg name="model_file" value="$(arg model)" />')
    <arg name="model_namespace" value="{robotname}" />
    <arg name="control_namespace" value="{robotname}" />
  </include>\n"""
    output_str +='  <group ns="$(arg robot_name)" >\n'
    for idx, dev in enumerate(devices):
        if dev.getName().lower().find('tof')>=0 or dev.getName().lower().find('ultra')>=0 :
            output_str += '    <node name="tof_converter_node_{}" pkg="irsl_choreonoid_ros" type="tof_converter_node.py" output="screen">\n'.format(idx)
            output_str += '      <remap from="input_points" to="{}/point_cloud"/>\n'.format(dev.getName())
            output_str += '      <remap from="output_sensor_data" to="{}/value"/>\n'.format(dev.getName())
            output_str += '    </node>\n'
            output_str += '\n'
        elif dev.getName().lower().find('color')>=0:
            output_str += '    <node name="colorsensor_converter_node_{}" pkg="irsl_choreonoid_ros" type="colorsensor_converter_node.py" output="screen">\n'.format(idx)
            output_str += '      <remap from="input_image" to="{}/color/image_raw"/>\n'.format(dev.getName())
            output_str += '      <remap from="output_sensor_data" to="{}/value"/>\n'.format(dev.getName())
            output_str += '    </node>\n'
            output_str += '\n'
    output_str += "  </group>\n"
    output_str += "</launch>\n"
    print(output_str, file=output)

if __name__=='__main__':
    parser = argparse.ArgumentParser(
            prog='generate_roslaunch.py', # プログラム名
            usage='Demonstration of cnoid_dump_model', # プログラムの利用方法

            add_help=True, # -h/–help オプションの追加
            )
    parser.add_argument('--bodyfile', type=str, required=True)
    parser.add_argument('--controllers', type=str, default='trajectory_controller joint_state_controller')
    parser.add_argument('--demo_base_dir', type=str, default='/userdir')
    parser.add_argument('--urdffile', type=str, required=True)
    parser.add_argument('--roscontrolfile', type=str)
    parser.add_argument('--worldsettings', type=str)
    
    args = parser.parse_args()
    fname = args.bodyfile
    robot = RobotModel.loadModel(fname)
    robotname = robot.robot.getModelName()
    devices     = robot.deviceList
    
    print_config(robotname, args.urdffile, args.controllers, devices, args.demo_base_dir, args.worldsettings)