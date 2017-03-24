#! /usr/bin/python

import time
import openravepy as rave
import IPython

import math
import numpy as np


def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)


if __name__ == "__main__":
    env = rave.Environment()  # create openrave environment
    # attach viewer (optional)
    env.SetViewer('qtcoin')
    # env.SetViewer('InteractiveMarker')
    # env.SetViewer('RViz')
    env.GetViewer().SetCamera([
        [ 0.,  0.,  1., -3.6],
        [-1.,  0.,  0.,  0.],
        [ 0., -1.,  0.,  1.1],
        [ 0.,  0.,  0.,  1.]
    ])

    # rospack = rospkg.RosPack() # Get a object that can find file paths to packages in ROS
    # urdf_package_path = rospack.get_path('arm_ordata')
    # srdf_package_path = rospack.get_path('dual_arm_moveit_config')

    urdf_module = rave.RaveCreateModule(env, 'urdf')

    with env:
        name = urdf_module.SendCommand('LoadURI package://arm_ordata/data/robots/dual_arm_robot.urdf package://dual_arm_moveit_config/config/dual_arm_robot.srdf')
        robot = env.GetRobot(name)

    IPython.embed()
