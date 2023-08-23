#!/usr/bin/env python


_prefix = "Path"

import rospy
from move_base_msgs.msg import *

from bt_tree_lib.bt_tree_ros import *
from GenGoal import *
from bt_tree_node import *
import threading

Path_SubTree_Dict = {}


class Path_SubTreeParamStatus(object):
    PREPARING = 0
    RUNNING = 1
    SUCCESS = 2
    FAILURE = 3


class Path_SubTreeParam(object):
    def __init__(self, *args, **kwargs):
        super(Path_SubTreeParam, self).__init__(*args, **kwargs)
        self.status = Path_SubTreeParamStatus.PREPARING
        return

    def get_status(self):
        return self.status

    def set_status(self, status):
        self.status = status
        return

    def reset(self):
        self.status = Path_SubTreeParamStatus.PREPARING
        return


def Path_cancel():
    for key, value in Path_SubTree_Dict.items():
        value['SubTreeParam'].reset()
        value['SubTree'].reset()
    return


def Path_get_SubTree(name):
    # Input check.
    if not isinstance(name, str):
        return False, None, None, "Parameter passed is not a string."
    if Path_SubTree_Dict.has_key(name):
        return False, None, None, "The Path_SubTree %s has already existed." % name

    SubTreeParam = Path_SubTreeParam()
    SubTree = _Path_get_SubTree(SubTreeParam)

    # Save the SubTree, and return result.
    Path_SubTree_Dict[name] = {'SubTreeParam': SubTreeParam, 'SubTree': SubTree}
    return SubTreeParam, SubTree





def _Path_get_SubTree(param):
    position_check = POSITION_CHECK("position_check", param)
    get_gear = GET_GEAR("get_gear", param)
    PATH_LOOP_HEADER = Selector("PATH_LOOP_HEADER", [position_check, get_gear])
  
    put_gear = PUT_GEAR("put_gear", param)

    root = Sequence_withMemory("root", [PATH_LOOP_HEADER, put_gear]) 
    return root


class POSITION_CHECK(LeafNode):
    def __init__(self, name, param, *args, **kwargs):
        super(POSITION_CHECK, self).__init__(name, *args, **kwargs)
        self.param = param
        return

    def run(self):
        threshold = 0.1
        # 读取齿轮的位置
        x = #np.loadtxt('')
        y = #np.loadtxt('')
        z = #np.loadtxt('')
        ort_0 = #np.loadtxt('')
        ort_1 = #np.loadtxt('')
        ort_2 = #np.loadtxt('')
        ort_3 = #np.loadtxt('')
        #for i in range(0, 1000):
        #    x = file_x[i]
        #    y = file_y[i]
        #    z = file_z[i]
        #    ort_0 = file_0[i]
        #    ort_1 = file_1[i]
        #    ort_2 = file_2[i]
        #    ort_3 = file_3[i]
        pose_a = [x, y, z, ort_0, ort_1, ort_2, ort_3]
        
        # pose = mylimb.get_pose()
        self.param.set_status(1)
        
        for i in range(0,3):
            if pose[i] >= pose_a[i] + threshold and pose[i] <= pose_a[i] - threshold:
                pass
            else:
                print('the robot is in gear_postion')
                return NodeStatus.FAILURE
        return NodeStatus.SUCCESS


class GET_GEAR(LeafNode):
    def __init__(self, name, path):
        super(GET_GEAR, self).__init__(name, *args, **kwargs)
        self.filepath = path
        self.inner_state = 0
        self.name = name

    def run(self):
        if self.inner_state == 0:
            rospy.loginfo("%s begin" % self.name)
            self.inner_state = 1
            thread = threading.Thread(target = self.robot_pick)
            thread.start()
            return NodeStatus.RUNNING
        if self.inner_state == 1:
            rospy.loginfo("running")
            return NodeStatus.RUNNING
        if self.inner_state == 2:
            self.reset()
            threshold = 0.01
            # pose = mylimb.get_pose()
            for i in range(0, 3):
                if pose[i] <= pose_a[i] + threshold and pose[i] >= pose_a[i] - threshold:
                    pass
                else:
                    print('The robot failed to pick up the object')
                    return NodeStatus.FAILURE
            return NodeStatus.SUCCESS


    def reset(self):
        self.inner_state = 0

    def robot_pick(self):
        # mylimb.baxter_ik_move(pose_a, False)
        # mylimb.gripperclose()
        self.inner_state = 2
        return



class PUT_GEAR(LeafNode):
    def __init__(self, name, path):
        super(PUT_GEAR, self).__init__(name, *args, **kwargs)
        self.filepath = path
        self.inner_state = 0
        self.name = name

    def run(self):
        if self.inner_state == 0:
            rospy.loginfo("%s begin" % self.name)
            self.inner_state = 1
            thread = threading.Thread(target = self.robot_put)
            thread.start()
            return NodeStatus.RUNNING
        if self.inner_state == 1:
            rospy.loginfo("running")
            return NodeStatus.RUNNING
        if self.inner_state == 2:
            self.reset()
            threshold = 0.01
            # pose = mylimb.get_pose()
            for i in range(0, 3):
                if pose[i] <= pose_b[i] + threshold and pose[i] >= pose_b[i] - threshold:
                    # mylimb.gripperopen()
                    pass
                else:
                    print('The robot failed to reach the pole')
                    return NodeStatus.FAILURE
            return NodeStatus.SUCCESS


    def reset(self):
        self.inner_state = 0

    def robot_put(self):
        
        # 读取杆的位置
        x = #np.loadtxt('')
        y = #np.loadtxt('')
        z = #np.loadtxt('')
        ort_0 = #np.loadtxt('')
        ort_1 = #np.loadtxt('')
        ort_2 = #np.loadtxt('')
        ort_3 = #np.loadtxt('')
        #for i in range(0, 1000):
        #    x = file_x[i]
        #    y = file_y[i]
        #    z = file_z[i]
        #    ort_0 = file_0[i]
        #    ort_1 = file_1[i]
        #    ort_2 = file_2[i]
        #    ort_3 = file_3[i]
        pose_b = [x, y, z, ort_0, ort_1, ort_2, ort_3]
        
        # mylimb.baxter_ik_move(pose_b, False)
        
        self.inner_state = 2
        return



"""

class MOVE(LeafNode):
    def __init__(self, name, param, *args, **kwargs):
        super(MOVE, self).__init__(name, *args, **kwargs)
        self.param = param
        return

   def run(self):
        status = self.param.get_status()
        
        pose = [x, y, z, ort_0, ort_1, ort_2, ort_3]
        mylimb.baxter_ik_move(pose, False)

        return NodeStatus.SUCCESS


class FALL_TO_B(LeafNode):
    def __init__(self, name, param, *args, **kwargs):
        super(FALL_TO_B, self).__init__(name, *args, **kwargs)
        self.param = param
        return

   def run(self):
        status = self.param.get_status()
        
        pose = [x, y, z, ort_0, ort_1, ort_2, ort_3]
        mylimb.baxter_ik_move(pose, False)

        return NodeStatus.SUCCESS


class LOOSEN(LeafNode):

    def __init__(self, name, param, *args, **kwargs):
        super(LOOSEN, self).__init__(name, *args, **kwargs)
        self.param = param
        return

   def run(self):
        status = self.param.get_status()
        
        mylimb.gripperopen()

        return NodeStatus.SUCCESS


"""