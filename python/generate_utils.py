#!/bin/python3


def get_jointlist(rbody):
    num_joint = rbody.getNumJoints()
    joints = [rbody.getJoint(idx) for idx in range(num_joint)]
    return joints


def get_jointnamelist(rbody):
    joints = get_jointlist(rbody)
    jointnames = [j.jointName for j in joints]
    return jointnames


def get_devicelist(rbody):
    num_device = rbody.getNumDevices()
    return [rbody.getDevice(idx) for idx in range(num_device)]
