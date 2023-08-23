#!/usr/bin/env python

import rospy
import roslib
import argparse
import numpy as np
import math
import os
import sys
import string
import time
import random
import tf
from sensor_msgs.msg import Image
import baxter_interface
from baxter_interface import CHECK_VERSION
from moveit_commander import conversions
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import std_srvs.srv
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
import matplotlib.pyplot as plt
import matplotlib as mpl
from scipy import interpolate

class MyDmp(object):

    def __init__(self, start=0, goal=1):
        # fixed params
        self.alpha_z = 25.0
        self.beta_z = self.alpha_z / 4
        self.alpha_g = self.alpha_z / 2
        self.alpha_x = self.alpha_z / 3
        self.alpha_v = self.alpha_z
        self.beta_v = self.beta_z
        # adjustable params
        self.start = start
        self.goal = goal
        self.dt = 0.001
        self.tau = 1.0
        # state
        self.z = 0.0
        self.dz = 0.0
        self.y = 0.0
        self.dy = 0.0
        self.ddy = 0.0
        self.x = 1.0
        self.dx = 0.0
        self.v = 0.0
        self.dv = 0.0
        self.g = 0.0
        self.dg = 0.0
        # else
        # orginal amplitude
        self.A = 0.0
        # goal amplitude
        self.dG = 0.0
        # scale factor
        self.s = 1.0
        # nf
        self.weights = 0.0
        self.num_bf = 40
        self.centers = self.get_centers()
        self.d = self.get_d()
        self.nf = 0.0
        # state arr
        self.x_arr = 0.0
        self.v_arr = 0.0
        self.g_arr = 0.0
        # regular size
        self.size_reg = 1000

    def set_num_bf(self, num_bf):
        self.num_bf = num_bf
        self.centers = self.get_centers()
        self.d = self.get_d()

    def reset_states(self):
        # state
        self.z = 0.0
        self.dz = 0.0
        self.y = self.start
        self.dy = 0.0
        self.ddy = 0
        self.x = 1.0
        self.dx = 0.0
        self.v = 0.0
        self.dv = 0.0
        self.g = self.start
        self.dg = 0.0

    # calculate the derivative
    def calc_derv(self, data):
        derv = np.diff(data) / self.dt
        derv = np.append(derv, 0.0)
        return derv

    # calculate the nonlinear function from a demo
    def calc_nf(self, file_name):
        # regular size
        size_reg = self.size_reg
        # resize the data
        demo = np.loadtxt(file_name)
        size_demo = np.size(demo)
        index = np.linspace(1, size_reg, size_demo)
        f_demo = interpolate.interp1d(index, demo, kind='quadratic')
        index = np.linspace(1, size_reg, size_reg)
        pos = f_demo(index)
        # pos = np.loadtxt(file_name)
        # vec = pos[:,1]
        # acc = pos[:,2]
        # pos = pos[:,0]
        # calc vec. & acc.
        vec = self.calc_derv(pos)
        acc = self.calc_derv(vec)
        # adjust params
        self.start = pos[0]
        self.goal = pos[-1]
        self.reset_states()

        # calc states arrays
        self.x_arr = np.zeros(size_reg)
        self.v_arr = np.zeros(size_reg)
        self.g_arr = np.zeros(size_reg)
        self.v_arr[0] = self.v
        self.x_arr[0] = self.x
        self.g_arr[0] = self.g
        for i in range(1, size_reg):
            (self.v_arr[i], self.x_arr[i]) = self.run_vsystem()
            self.g_arr[i] = self.run_gsystem()

        # calc nf
        self.dG = self.goal - self.start
        self.A = np.max(pos) - np.min(pos)
        self.s = 1.0
        amp = self.s
        self.nf = (acc / pow(self.tau, 2) - self.alpha_z * (self.beta_z * (self.g_arr - pos) - vec / self.tau)) / amp
        # np.savetxt(file_name+'s'+'.txt',self.x_arr)
        # np.savetxt(file_name+'f'+'.txt',self.nf)
        return self.nf

    # learn the features from a demo
    def learn_weights_from_file(self, file_name):
        # regular size
        size_reg = self.size_reg
        self.calc_nf(file_name)
        # the gaussians
        # transport
        self.x_arr.shape = (np.size(self.x_arr), 1)
        self.v_arr.shape = (np.size(self.v_arr), 1)
        self.g_arr.shape = (np.size(self.g_arr), 1)
        self.centers.shape = (np.size(self.centers), 1)
        self.d.shape = (np.size(self.d), 1)
        self.nf.shape = (np.size(self.nf), 1)

        tmp_a = pow(
            np.dot(self.x_arr, np.ones((1, self.num_bf))) - np.dot(np.ones((size_reg, 1)), np.transpose(self.centers)),
            2)
        tmp_b = np.dot(np.ones((self.size_reg, 1)), np.transpose(self.d))
        psi = np.exp(-0.5 * tmp_a * tmp_b)
        sx2 = np.transpose(np.sum(np.dot(pow(self.v_arr, 2), np.ones((1, self.num_bf))) * psi, 0))
        sxtd = np.transpose(np.sum(np.dot(self.v_arr * self.nf, np.ones((1, self.num_bf))) * psi, 0))
        self.weights = sxtd / (sx2 + 1.0E-10)
        print ("learned weights: %s" % self.weights)
        # transport
        self.x_arr.shape = (1, np.size(self.x_arr))
        self.v_arr.shape = (1, np.size(self.v_arr))
        self.g_arr.shape = (1, np.size(self.g_arr))
        self.centers.shape = (1, np.size(self.centers))
        self.d.shape = (1, np.size(self.d))
        self.nf.shape = (1, np.size(self.nf))

        return self.weights

    # load the features from file
    def get_weights_from_file(self, file_name):
        self.weights = np.loadtxt(file_name)
        print ("The weights array is %s" % self.weights)
        self.num_bf = np.size(self.weights)
        print ("Its size (the number of Gaussians) is %d" % self.num_bf)
        self.centers = self.get_centers()
        self.d = self.get_d()
        return self.weights

    # calc centers
    def get_centers(self):
        t = np.linspace(0, 1, self.num_bf) * 0.5
        c = (1.0 + self.alpha_z / 2.0 * t) * np.exp(-self.alpha_z / 2.0 * t)
        print ("The centers is %s" % c)
        return c

    # calc variances
    def get_d(self):
        d = pow(np.diff(self.centers * 0.55), 2)
        d = 1 / (np.append(d, d[-1]))
        print ("The variances is %s" % d)
        return d

    # canonical system
    def run_vsystem(self):
        self.dv = (self.alpha_v * (self.beta_v * (0.0 - self.x) - self.v)) * self.tau
        self.dx = self.v * self.tau
        self.x = self.dx * self.dt + self.x
        self.v = self.dv * self.dt + self.v
        return (self.v, self.x)

    # goal system
    def run_gsystem(self):
        self.dg = self.alpha_g * (self.goal - self.g)
        self.g = self.dg * self.dt + self.g
        return self.g

    # transformation system
    def run(self, current):
        psi = np.exp(-0.5 * pow((self.x - self.centers), 2) * self.d)
        amp = self.s
        # calculate nonlinear function
        f = np.sum(self.v * self.weights * psi) / np.sum(psi + 1.0E-10) * amp
        # print "The value of nonlinear function is: %s" % f

        # update z
        self.dz = (self.alpha_z * (self.beta_z * (self.g - self.y) - self.z) + f) * self.tau
        self.dy = self.z * self.tau
        self.ddy = self.dz * self.tau
        self.z = self.dz * self.dt + self.z
        self.y = self.dy * self.dt + self.y

        # update v
        self.run_vsystem()
        # update g
        self.run_gsystem()

        return (self.y, self.dy, self.ddy)
    def stif_run(self, current):
        psi = np.exp(-0.5 * pow((self.x - self.centers), 2) * self.d)
        amp = self.s
        c=0.3
        # calculate nonlinear function
        f = np.sum(self.v * self.weights * psi) / np.sum(psi + 1.0E-10) * amp
        # print "The value of nonlinear function is: %s" % f

        # update z
        self.dz = (self.alpha_z * (self.beta_z * (self.g - self.y) - self.z) + f) * self.tau
        self.dy = self.z * self.tau+c * current * self.tau
        self.ddy = self.dz * self.tau
        self.z = self.dz * self.dt + self.z
        self.y = self.dy * self.dt + self.y

        # update v
        self.run_vsystem()
        # update g
        self.run_gsystem()

        return self.y

class MyLimb(object):
    def __init__(self, limb):
        self.limb = limb
        self.limb_interface = baxter_interface.Limb(self.limb)

        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

    # get the pose  用
    def get_pose(self):
        quaternion_pose = self.limb_interface.endpoint_pose()
        position = quaternion_pose['position']
        quaternion = quaternion_pose['orientation']
        # euler = tf.transformations.euler_from_quaternion(quaternion)
        # rpy_pose = (position[0],position[1],position[2], euler[0], euler[1], euler[2])
        # return rpy_pose
        return (position[0], position[1], position[2], quaternion[0], quaternion[1], quaternion[2], quaternion[3])

    # move a limb
    def baxter_ik_move(self, quaternion_pose, is_first_time):
        # quaternion_pose = conversions.list_to_pose_stamped(rpy_pose, "base")

        node = "ExternalTools/" + self.limb + "/PositionKinematicsNode/IKService"
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id="base")

        pose_stamp = PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=quaternion_pose[0],
                    y=quaternion_pose[1],
                    z=quaternion_pose[2],
                ),
                orientation=Quaternion(
                    x=quaternion_pose[3],
                    y=quaternion_pose[4],
                    z=quaternion_pose[5],
                    w=quaternion_pose[6],
                ),
            ),
        )
        ik_request.pose_stamp.append(pose_stamp)
        # ik_request.pose_stamp.append(quaternion_pose)
        try:
            rospy.wait_for_service(node, 5.0)
            ik_response = ik_service(ik_request)
        except (rospy.ServiceException, rospy.ROSException), error_message:
            rospy.logerr("Service request failed: %r" % (error_message,))
            # sys.exit("ERROR - baxter_ik_move - Failed to append pose")
            print("ERROR - baxter_ik_move - Failed to append pose")
            return

        if ik_response.isValid[0]:
            print("PASS: Valid joint configuration found")
            # convert response to joint position control dictionary
            limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
            # move limb
            if is_first_time:
                self.limb_interface.move_to_joint_positions(limb_joints)
            else:
                self.limb_interface.set_joint_positions(limb_joints)

        else:
            # little point in continuing so exit with error message
            print ("requested move =", rpy_pose)
            print("ERROR - baxter_ik_move - Failed to append pose")
            return

    def clean_shutdown(self): # 通过将头部移动到中立位置，干净地退出示例维护启动状态
        """
        Exits example cleanly by moving head to neutral position and
        maintaining start state
        """
        print("\nExiting example...")
        if not self._init_state and self._rs.state().enabled:
            print("Disabling robot...")
            self._rs.disable()

    def gripperclose(self):  # 夹子关闭
        ###################INIT#########################################
        # rospy.init_node('Hello_Baxter')
        limbr = baxter_interface.Limb('right')
        limbl = baxter_interface.Limb('left')
        gripr = baxter_interface.Gripper('right')
        gripl = baxter_interface.Gripper('left')
        # gripr.calibrate()
        # gripl.calibrate()

        # limbl.set_joint_position_speed(0.1)

        # limbl.move_to_neutral()
        re = gripl.close()
        print re

    def gripperopen(self):
        ###################INIT#########################################

        limbr = baxter_interface.Limb('right')
        limbl = baxter_interface.Limb('left')
        gripr = baxter_interface.Gripper('right')
        gripl = baxter_interface.Gripper('left')
        # gripr.calibrate()
        # gripl.calibrate()

        # limbl.set_joint_position_speed(0.1)

        # limbl.move_to_neutral()
        re = gripl.open()
        print re






