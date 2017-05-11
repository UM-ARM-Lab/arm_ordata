#! /usr/bin/python

import time
import openravepy as rave
import IPython

import math
import numpy as np

def plot_axes(env, trans, lengths = (0.25, 0.25, 0.25), thickness = 0.005):
    red = np.array([1, 0, 0, 1])
    green = np.array([0, 1, 0, 1])
    blue = np.array([0, 0, 1, 1])
    plot_handles = []
    plot_handles.append(env.drawarrow(trans[0:3, 3], trans[0:3, 3] + trans[0:3, 0] * lengths[0], thickness, red))
    plot_handles.append(env.drawarrow(trans[0:3, 3], trans[0:3, 3] + trans[0:3, 1] * lengths[1], thickness, green))
    plot_handles.append(env.drawarrow(trans[0:3, 3], trans[0:3, 3] + trans[0:3, 2] * lengths[2], thickness, blue))
    return plot_handles


def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)


if __name__ == "__main__":
    env = rave.Environment()  # create openrave environment

    env.SetViewer('qtcoin')  # attach viewer (optional)
    env.GetViewer().SetCamera([
        [ 0.,  0.,  1., -3.6],
        [-1.,  0.,  0.,  0.],
        [ 0., -1.,  0.,  1.1],
        [ 0.,  0.,  0.,  1.]
    ])

    robot_single_arm = env.ReadRobotURI('robots/iiwa7.robot.xml')
    env.Add(robot_single_arm, True)

    ikmodel = rave.databases.inversekinematics.InverseKinematicsModel(robot=robot_single_arm, iktype=rave.IkParameterization.Type.Transform6D)

    if not ikmodel.load():
        ikmodel.autogenerate()  # autogenerate if one doesn't exist

    rot_y_pi_over2 = rave.matrixFromAxisAngle(np.array([1.0, 0.0, 0.0]) * np.pi / 2.0)
    trans = np.array([
        [1., 0., 0., 0.526],
        [0., 1., 0., 0.0],
        [0., 0., 1., 0.74],
        [0., 0., 0., 1.]
    ])
    target_pose = trans.dot(rot_y_pi_over2)
    ee_target_plot = plot_axes(env, target_pose)

    manip = robot_single_arm.GetManipulators()[0]
    target_config = manip.FindIKSolution(target_pose, rave.IkFilterOptions.CheckEnvCollisions)
    robot_single_arm.SetDOFValues(target_config)
    ee_plot = plot_axes(env, robot_single_arm.GetLink('iiwa_link_ee').GetTransform())

    IPython.embed()
