#! /usr/bin/python

import time
import openravepy as rave
import IPython

import math
import numpy as np


if __name__ == "__main__":
    rave.misc.InitOpenRAVELogging()
    env = rave.Environment()  # create openrave environment
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

    robot.SetActiveManipulator('left_arm')
    manip = robot.GetActiveManipulator()

    ikmodel = rave.databases.inversekinematics.InverseKinematicsModel(robot, iktype=rave.IkParameterizationType.Transform6D)

    if not ikmodel.load():
        ikmodel.autogenerate()

    with env:
        robot.SetActiveDOFs(manip.GetArmIndices())
        goal_pose = np.array([[1, 0, 0, 0.8],
                              [0, 1, 0, 0],
                              [0, 0, 1, 1.2],
                              [0, 0, 0, 1]])

        # IPython.embed()
        target_config = manip.FindIKSolution(goal_pose, rave.IkFilterOptions.CheckEnvCollisions)
        manip_problem = rave.interfaces.BaseManipulation(robot)
        arm_traj = manip_problem.MoveManipulator(goal = target_config, execute = False, outputtrajobj = True)
        rave.planningutils.RetimeActiveDOFTrajectory(arm_traj, robot, hastimestamps = False, maxvelmult = 0.1, maxaccelmult = 0.1, plannername = 'ParabolicTrajectoryRetimer')


    time.sleep(1)

    robot.GetController().SetPath(arm_traj)
    robot.WaitForController(0)

    print "Final pose\n", manip.GetEndEffectorTransform()

    IPython.embed()
