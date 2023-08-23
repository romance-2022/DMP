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
    INITIAL_POSITION = robot_in_initial_postion("INITIAL_POSITION", param)
    PICK_UP_OBJECT = robot_pick_object("PICK_UP_OBJECT", param)
    PICK_UP_STAGE = Sequence("PICK_UP_STAGE", [INITIAL_POSITION, PICK_UP_OBJECT])
    WITH_OBS = WITH_OBSTACLE("WITH_OBSTACLE", param)

    ACTION_WITH_OBS = ACTION_WITH_OBSTACLE("ACTION_WITH_OBSTACLE")
    WITH_OBSTACLE_ACTION = Sequence("WITH_OBSTACLE_ACTION", WITH_OBS, ACTION_WITH_OBS)

    NO_OBSTACLE = NONE_OBSTACLE("NONE_OBSTACLE", param)
    ACTION_NO_OBS = ACTION_NO_OBSTACLE("ACTION_NONE_OBSTACLE")
    NONE_OBSTACLE_ACTION = Sequence("NO_OBSTACLE_ACTION", NO_OBSTACLE, ACTION_NO_OBS)

    ACTION_NODE = Selector("ACTION_STAGE", WITH_OBSTACLE_ACTION, NONE_OBSTACLE_ACTION)

    PICK_UP = PICK_UP_RESULT("PICK_UP_RESULT")

    ACTION_STAGE = Sequence("ACTION_STAGE", PICK_UP, ACTION_NODE)

    ROOT = Sequence("ROOT", PICK_UP_STAGE, ACTION_STAGE)

    return ROOT


class robot_in_initial_postion(LeafNode):
    def __init__(self, name, param, *args, **kwargs):
        super(robot_in_initial_postion, self).__init__(name, *args, **kwargs)
        self.param = param
        return

    def run(self):
        threshold = 0.1
        pose = mylimb.get_pose()
        self.param.set_status(1)

        for i in range(0,3):
            if pose[i] <= init_pos[i] + threshold and pose[i] >= init_pos[i] - threshold:
                pass
            else:
                print('the robot is not in the initial postion')
                return NodeStatus.FAILURE
        return NodeStatus.SUCCESS

class robot_pick_object(LeafNode):
    def __init__(self, name, path):
        super(robot_pick_object, self).__init__(name, *args, **kwargs)
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
            pose = mylimb.get_pose()
            for i in range(0, 3):
                if pose[i] <= pick_pos[i] + threshold and pose[i] >= pick_pos[i] - threshold:
                    pass
                else:
                    print('The robot failed to pick up the object')
                    return NodeStatus.FAILURE
            return NodeStatus.SUCCESS


    def reset(self):
        self.inner_state = 0

    def robot_pick(self):
        file_x = np.loadtxt('')
        file_y = np.loadtxt('')
        file_z = np.loadtxt('')
        file_0 = np.loadtxt('')
        file_1 = np.loadtxt('')
        file_2 = np.loadtxt('')
        file_3 = np.loadtxt('')
        for i in range(0, 1000):
            x = file_x[i]
            y = file_y[i]
            z = file_z[i]
            ort_0 = file_0[i]
            ort_1 = file_1[i]
            ort_2 = file_2[i]
            ort_3 = file_3[i]
            pose = [x, y, z, ort_0, ort_1, ort_2, ort_3]
            mylimb.baxter_ik_move(pose, False)
        mylimb.gripperclose()
        self.inner_state = 2
        return


class WITH_OBSTACLE(LeafNode):
    def __init__(self, name, param, *args, **kwargs):
        super(WITH_OBSTACLE, self).__init__(name, *args, **kwargs)
        self.param = param
        self.name = name
        return

    def run(self):
        global  obs
        obs = input("Whether there are obstacles? 1:YES 0:NO")
        obs = int(obs)
        if obs == 1:
            return NodeStatus.SUCCESS
        else:
            return NodeStatus.FAILURE
            

class PICK_UP_RESULT(LeafNode):
    def __init__(self,name):
        super(PICK_UP_RESULT,self).__init__(name)
        self.name = name
        return
    def run(self):
        W = input("Whether to lift the object successfully? 1:YES 0:NO")
        W = int(W)
        if W == 1:
            return NodeStatus.SUCCESS
        else:
            return NodeStatus.FAILURE



class NONE_OBSTACLE(LeafNode):
    def __init__(self, name, param, *args, **kwargs):
        super(NONE_OBSTACLE, self).__init__(name, *args, **kwargs)
        self.param = param
        self.name = name
        return

    def run(self):
        if obs == 0:
            return NodeStatus.SUCCESS
        else:
            return NodeStatus.FAILURE

class ACTION_NO_OBSTACLE(LeafNode):
    def __init__(self, name):
        super(ACTION_NO_OBSTACLE, self).__init__(name, *args, **kwargs)
        self.inner_state = 0
        self.name = name

    def run(self):
        if self.inner_state == 0:
            rospy.loginfo("%s begin" % self.name)
            self.inner_state = 1
            thread1 = threading.Thread(target=self.action_1())
            thread1.start()
            return NodeStatus.RUNNING
        if self.inner_state == 1:
            rospy.loginfo()
            return NodeStatus.RUNNING
        if self.inner_state == 2:
            self.reset()
            threshold = 0.01
            pose = mylimb.get_pose()
            for i in range(0, 3):
                if pose[i] <= final_pos[i] + threshold and pose[i] >= final_pos[i] - threshold:
                    pass
                else:
                    print('The robot failed to reach the target location')
                    return NodeStatus.FAILURE
            return NodeStatus.SUCCESS
    def reset(self):
        self.inner_state = 0

    def action_1(self):
        file_x = np.loadtxt('')
        file_y = np.loadtxt('')
        file_z = np.loadtxt('')
        file_0 = np.loadtxt('')
        file_1 = np.loadtxt('')
        file_2 = np.loadtxt('')
        file_3 = np.loadtxt('')
        for i in range(0, 1000):
            x = file_x[i]
            y = file_y[i]
            z = file_z[i]
            ort_0 = file_0[i]
            ort_1 = file_1[i]
            ort_2 = file_2[i]
            ort_3 = file_3[i]
            pose = [x, y, z, ort_0, ort_1, ort_2, ort_3]
            mylimb.baxter_ik_move(pose, False)
        mylimb.gripperclose()
        self.inner_state = 2
        return


class ACTION_WITH_OBSTACLE(LeafNode):
    def __init__(self, name):
        super(ACTION_WITH_OBSTACLE, self).__init__(name, *args, **kwargs)
        self.inner_state = 0
        self.name = name

    def run(self):
        if self.inner_state == 0:
            rospy.loginfo("%s begin" % self.name)
            self.inner_state = 1
            thread2 = threading.Thread(target=self.action_1())
            thread2.start()
            self.action_1()
            return NodeStatus.RUNNING
        if self.inner_state == 1:
            rospy.loginfo()
            return NodeStatus.RUNNING
        if self.inner_state == 2:
            self.reset()
            threshold = 0.01
            pose = mylimb.get_pose()
            for i in range(0, 3):
                if pose[i] <= final_pos[i] + threshold and pose[i] >= final_pos[i] - threshold:
                    pass
                else:
                    print('The robot failed to reach the target location')
                    return NodeStatus.FAILURE
            return NodeStatus.SUCCESS
    def reset(self):
        self.inner_state = 0

    def action_1(self):
        file_x = np.loadtxt('')
        file_y = np.loadtxt('')
        file_z = np.loadtxt('')
        file_0 = np.loadtxt('')
        file_1 = np.loadtxt('')
        file_2 = np.loadtxt('')
        file_3 = np.loadtxt('')
        for i in range(0, 1000):
            x = file_x[i]
            y = file_y[i]
            z = file_z[i]
            ort_0 = file_0[i]
            ort_1 = file_1[i]
            ort_2 = file_2[i]
            ort_3 = file_3[i]
            pose = [x, y, z, ort_0, ort_1, ort_2, ort_3]
            mylimb.baxter_ik_move(pose, False)
        mylimb.gripperclose()
        self.inner_state = 2
        return








