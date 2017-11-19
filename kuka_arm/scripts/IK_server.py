#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
        #
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # Create Modified DH parameters
        #
        dh_params = {alpha0:  0,    a0:  0,     d1: 0.75,
                     alpha1: -pi/2, a1:  0.35,  d2: 0,    q2: q2-pi/2,
                     alpha2:  0,    a2:  1.25,  d3: 0,
                     alpha3: -pi/2, a3: -0.054, d4: 1.50,
                     alpha4:  pi/2, a4:  0,     d5: 0,
                     alpha5: -pi/2, a5:  0,     d6: 0,
                     alpha6:  0,    a6:  0,     d7: 0.303, q7: 0}

        # Define Modified DH Transformation matrix
        #
        def T_mat(alpha, a, d, q):
                T = Matrix([[cos(q),              -sin(q),               0,           a             ],
                            [sin(q) * cos(alpha),  cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                            [sin(q) * sin(alpha),  cos(q) * sin(alpha),  cos(alpha),  cos(alpha) * d],
                            [0,                    0,                    0,           1             ]])
                return T

        # Create individual transformation matrices
        #
        T0_1 = T_mat(alpha0, a0, d1, q1).subs(dh_params)
        T1_2 = T_mat(alpha1, a1, d2, q2).subs(dh_params)
        T2_3 = T_mat(alpha2, a2, d3, q3).subs(dh_params)
        T3_4 = T_mat(alpha3, a3, d4, q4).subs(dh_params)
        T4_5 = T_mat(alpha4, a4, d5, q5).subs(dh_params)
        T5_6 = T_mat(alpha5, a5, d6, q6).subs(dh_params)
        T6_EE = T_mat(alpha6, a6, d7, q7).subs(dh_params)

        T0_2 = T0_1 * T1_2
        T0_3 = T0_2 * T2_3
        T0_4 = T0_3 * T3_4
        T0_5 = T0_4 * T4_5
        T0_6 = T0_5 * T5_6
        T0_EE = T0_6 * T6_EE

        # Define rotation matrix
        #
        r, p, y = symbols('r p y')

        R_x = Matrix([[ 1, 0,       0     ],
                      [ 0, cos(r), -sin(r)],
                      [ 0, sin(r),  cos(r)]])

        R_y = Matrix([[ cos(p), 0,  sin(p)],
                      [ 0,      1,  0     ],
                      [-sin(p), 0,  cos(p)]])

        R_z = Matrix([[ cos(y), -sin(y), 0],
                      [ sin(y),  cos(y), 0],
                      [ 0,       0,      1]])

        R = R_z * R_y * R_x

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            #
            R_EE = R.evalf(subs={r:roll, p:pitch, y:yaw})
            R_corr = R.evalf(subs={r:0, p:-pi/2, y:pi})

            R_EE = R_EE * R_corr.transpose()

            # Calculate joint angles using Geometric IK method
            #
            EE = Matrix([ [px], [py], [pz] ])
            WC = EE - 0.303 * R_EE[:,2]

            theta1 = (atan2(WC[1], WC[0])).evalf()

            # SSS triangle for theta2 and theta3
            side_a = 1.50
            side_b = sqrt(np.square(sqrt(np.square(WC[0]) + np.square(WC[1])) - 0.35) + np.square(WC[2] - 0.75))
            side_c = 1.25

            angle_a = acos((np.square(side_b) + np.square(side_c) - np.square(side_a)) / (2 * side_b * side_c))
            angle_b = acos((np.square(side_a) + np.square(side_c) - np.square(side_b)) / (2 * side_a * side_c))
            angle_c = acos((np.square(side_a) + np.square(side_b) - np.square(side_c)) / (2 * side_a * side_b))

            theta2 = (pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(np.square(WC[0]) + np.square(WC[1])) - 0.35)).evalf()
            theta3 = (pi /2 - (angle_b + 0.036)).evalf() # 0.036 accounts for sag in link4 of -0.054m
            
            # Euler angles from rotation matrix            
            R0_3 = T0_3[0:3, 0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})            

            R3_6 = R0_3.T * R_EE

            theta4 = (atan2( R3_6[2, 2], -R3_6[0, 2])).evalf()
            theta5 = (atan2(sqrt(np.square(R3_6[0,2]) + np.square(R3_6[2,2])), R3_6[1,2])).evalf()
            theta6 = (atan2(-R3_6[1, 1],  R3_6[1, 0])).evalf()
            
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
