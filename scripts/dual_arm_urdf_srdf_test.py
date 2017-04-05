#! /usr/bin/env python

import time
import openravepy as rave
import rospkg
import IPython

import math
import numpy as np


if __name__ == "__main__":
    rave.misc.InitOpenRAVELogging()
    env = rave.Environment()  # create openrave environment

    # http://wiki.ros.org/Packages#Python
    rospack = rospkg.RosPack()
    path = rospack.get_path('arm_ordata')
    env.Load(path + '/data/wiktor_test.env.xml')
    time.sleep(0.1)

    # rave.RaveSetDebugLevel(rave.DebugLevel.Debug)  # set output level to debug

    collision_checker = rave.RaveCreateCollisionChecker(env, 'ode')
    env.SetCollisionChecker(collision_checker)

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

    urdf_module = rave.RaveCreateModule(env, 'urdf')

    with env:
        name = urdf_module.SendCommand('LoadURI package://wiktor_description/urdf/wiktor.urdf package://wiktor_moveit_config/config/wiktor.srdf')
        robot = env.GetRobot(name)
    time.sleep(1)

    robot.SetActiveManipulator('right_arm')
    manip = robot.GetActiveManipulator()

    ikmodel = rave.databases.inversekinematics.InverseKinematicsModel(robot, iktype=rave.IkParameterizationType.Transform6D)

    if not ikmodel.load():
        ikmodel.autogenerate()

    with env:
        robot.SetActiveDOFs(manip.GetArmIndices())
        goal_pose = np.array([[ 0., 0., 1.,  0.7],
                              [ 0., 1., 0.,  -0.35],
                              [-1., 0., 0.,  0.55],
                              [ 0., 0., 0.,  1.]])

        # IPython.embed()
        target_config = manip.FindIKSolutions(goal_pose, rave.IkFilterOptions.CheckEnvCollisions)
        manip_problem = rave.interfaces.BaseManipulation(robot)
        arm_traj = manip_problem.MoveManipulator(goal = target_config[0], execute = False, outputtrajobj = True)
        rave.planningutils.RetimeActiveDOFTrajectory(arm_traj, robot, hastimestamps = False)

    raw_input("Press enter to execute trajectory...")

    with env:
        robot.GetController().SetPath(arm_traj)

    robot.WaitForController(0)

    print "Final pose\n", manip.GetEndEffectorTransform()

    IPython.embed()
