#!/usr/bin/python
import rospy
import os
import sys
from learn_objects_action.machine import LearnObjectActionMachineRAL16, LearnObjectActionMachineInfoGain
from smach_ros import ActionServerWrapper
from learn_objects_action.msg import LearnObjectAction

rospy.init_node("learn_dynamic_object_action_server")

model_path = rospy.get_param("~model_path","~/models")
model_path = os.path.expanduser(model_path)
rospy.loginfo("Model path set to: "+model_path)
if not os.path.isdir(model_path) and not os.path.isdir(os.path.join(model_path,"reconstruct")):
    rospy.logerr("The chosen model path does not exist. Make sure that " + model_path + 
                 " and " + os.path.join(model_path,"reconstruct") +" exist,"
                 " or set ~model_path properly.")
    sys.exit(1)

rois_file = os.path.expanduser(rospy.get_param("~soma_rois", "NONE"))
if rois_file=="NONE":
    rospy.loginfo("Not using any SOMA ROIs")
else:
    if not os.path.isfile(os.path.expanduer(rois_file)):
        rospy.logerr("SOMA Rois file " + rois_file + "does not exist!")
        sys.exit(1)

debug_mode = rospy.get_param("~debug_mode", False)
if debug_mode:
    rospy.logwarn("Object learning server is in debug mode, topics /object_learning/debug_status and /object_learning/proceed are required for interactive operation.")


# Construct state machine

planning_method = rospy.get_param("~view_planner", "ral16")
if planning_method == "ral16":
    rospy.loginfo("Using the RAL-16 method for learning the object.")
    sm = LearnObjectActionMachineRAL16(model_path, rois_file, debug_mode)
    pass
elif planning_method == "infogain":
    sm = LearnObjectActionMachineInfoGain(model_path, debug_mode)
else:
    rospyt.logerr("The chosen planning method is not available.")
    sys.exit(1)


# Construct action server wrapper
asw = ActionServerWrapper(
    'learn_object',
    LearnObjectAction,
    wrapped_container = sm,
    succeeded_outcomes = ['succeded'], 
    aborted_outcomes = ['failed'],
    preempted_outcomes = ['preempted'], 
    goal_key = 'action_goal',
    result_key='action_result' )

# Run the server in a background thread
asw.run_server()

# Wait for control-c
rospy.spin()
