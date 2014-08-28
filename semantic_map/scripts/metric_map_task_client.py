#!/usr/bin/env python

import rospy
import mongodb_store_msgs.srv as dc_srv
from mongodb_store_msgs.msg import StringPair
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion
import StringIO

from strands_executive_msgs import task_utils
from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import AddTask, SetExecutionStatus
# import strands_executive_msgs
import sys

if __name__ == '__main__':
    rospy.init_node("metric_map_task_client")

    # need message store to pass objects around
    msg_store = MessageStoreProxy() 

    try:

	task = Task(start_node_id=sys.argv[1], action='ptu_pan_tilt_metric_map')
        task_utils.add_int_argument(task, '-150')
        task_utils.add_int_argument(task, '60')
        task_utils.add_int_argument(task, '160')
        task_utils.add_int_argument(task, '-30')
        task_utils.add_int_argument(task, '20')
        task_utils.add_int_argument(task, '30')

        print task

        # now register this with the executor
        if  len(sys.argv)>2 and sys.argv[2]=="demand":
            print "Demanding task be run NOW."
            add_task_srv_name = '/task_executor/demand_task'
        else:
            add_task_srv_name = '/task_executor/add_task'
        set_exe_stat_srv_name = '/task_executor/set_execution_status'
        rospy.loginfo("Waiting for task_executor service...")
        rospy.wait_for_service(add_task_srv_name)
        rospy.wait_for_service(set_exe_stat_srv_name)
        rospy.loginfo("Done")        
        
        add_task_srv = rospy.ServiceProxy(add_task_srv_name, AddTask)
        set_execution_status = rospy.ServiceProxy(set_exe_stat_srv_name, SetExecutionStatus)
        print add_task_srv(task)

        # Make sure the task executor is running
#        set_execution_status(True)


    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


        


