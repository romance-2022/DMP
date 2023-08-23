#!/usr/bin/env python


import threading

import rospy

from bt_tree_generate import *
from robot_baxter import *

NodeStatusMaps = {
    NodeStatus.SUCCESS: "SUCCESS",
    NodeStatus.RUNNING: "RUNNING",
    NodeStatus.FAILURE: "FAILURE",
}


ActionNode, timer = None, None
def ActionNodeCycleRun():
    ActionNode.status = ActionNode.run()
    rospy.loginfo(NodeStatusMaps[ActionNode.status])

    if ActionNode.status == 1:
        global timer
        timer = threading.Timer(1, ActionNodeCycleRun)
        timer.setDaemon(True)
        timer.start()



if __name__ == "__main__":
    rospy.init_node("subtree_test")
    global mylimb = MyLimb(left)

    ActionNode = Path_get_SubTree("test")
    ActionNodeCycleRun()

    rospy.spin()

