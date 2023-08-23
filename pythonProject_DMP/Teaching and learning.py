#!/usr/bin/env python

import rospy
import time
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
import argparse
import struct
import socket
from robot_baxter import *






def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("control_head")
    dict_path = '/home/yang/ros_ws/src/baxter_examples/scripts/djl/shiyan/'
    fn_x = 'x.txt'
    fn_y = 'y.txt'
    fn_z = 'z.txt'
    fn_q1 = 'q1.txt'
    fn_q2 = 'q2.txt'
    fn_q3 = 'q3.txt'
    fn_q4 = 'q4.txt'
    list_fn = [fn_x, fn_y, fn_z, fn_q1, fn_q2, fn_q3, fn_q4]
    for fn in list_fn:
        with open(dict_path + fn, 'a') as f:
            f.truncate(0)
    with open('/home/yang/ros_ws/src/baxter_examples/scripts/djl/shiyan/f_record.txt', 'a') as f:
        f.truncate(0)
    mylimb = MyLimb("left")
    rospy.on_shutdown(mylimb.clean_shutdown)
    idx = 0
    force_z = []
    for fn in list_fn:
        with open(dict_path + fn, 'w+') as f:
            f.truncate()
    while not rospy.is_shutdown():
        time.sleep(0.01)
        pose = mylimb.get_pose()
        print(pose, '\n', f)
        # with open('/home/yang/ros_ws/src/baxter_examples/scripts/djl/shiyan/f_record.txt','a') as f:
        #     f.write(str(force) + '\n')
        for fn in list_fn:
            with open(dict_path + fn, 'a') as f:
                f.write(str(pose[list_fn.index(fn)]) + '\n')
        # rospy.sleep(1)
        # print(pose)
        # rospy.sleep(1)
        idx += 1
    # print(force_z)
    for i in range(0, 4):
        fname3 = '/home/yang/ros_ws/src/baxter_examples/scripts/djl/shiyan/q%d.txt' % (fname2[i])
        sfn1 = '/home/yang/ros_ws/src/baxter_examples/scripts/djl/shiyan/dmp_ort_%d.txt' % (fname2[i])
        demo = np.loadtxt(fname3)
        size_reg = 1000
        size_demo = np.size(demo)
        index = np.linspace(1, size_reg, size_demo)
        f_demo = interpolate.interp1d(index, demo, kind='quadratic')
        index = np.linspace(1, size_reg, size_reg)
        ort = f_demo(index)
        np.savetxt(sfn1, ort, delimiter='\n')
    for j in range(0, 3):
        mydmp = MyDmp()
        fname = '/home/yang/ros_ws/src/baxter_examples/scripts/djl/shiyan/%s.txt' % (fname1[i])

        mydmp.learn_weights_from_file(fname)
        mydmp.reset_states()

        num_iter = 1000
        pos = np.zeros(num_iter)
        vec = np.zeros(num_iter)
        acc = np.zeros(num_iter)
        # run
        for k in range(0, num_iter):
            [y, dy, ddy] = mydmp.run(mydmp.y)  # if input is mydmp.y, it means that the trajectory tracking is ideal.
            pos[k] = y
            vec[k] = dy
            acc[k] = ddy

        np.savetxt(sfn, pos, delimiter='\n')




if __name__ == '__main__':
    main()
