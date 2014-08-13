#!/usr/bin/env python

import rospy
import mongodb_store_msgs.srv as dc_srv
from mongodb_store_msgs.msg import StringPair
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
import StringIO

from strands_executive_msgs import task_utils
from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import AddTask, SetExecutionStatus
import sys
from os.path import expanduser

if __name__ == '__main__':
    rospy.init_node("ftp_upload_task_client")

    # need message store to pass objects around
    msg_store = MessageStoreProxy()

    try:

        task = Task(start_node_id=sys.argv[1], action='ftp_upload')
        ftp_server = '130.237.218.61'
        username = 'guest_data'
        password = 'guest'
        remote_path = 'pools/A/A0/Guest'
        remote_folder = 'G4S'
        cachePath = expanduser("~") + '/.semanticMap/cache'

        task_utils.add_string_argument(task, ftp_server) # FTP server address
        task_utils.add_string_argument(task, username) # username
        task_utils.add_string_argument(task, password) # password
        task_utils.add_string_argument(task, remote_path) # remote path
        task_utils.add_string_argument(task, remote_folder) # remote folder
        task_utils.add_string_argument(task, cachePath) # local path

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

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e





