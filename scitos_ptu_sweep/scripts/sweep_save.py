#!/usr/bin/env python

import rospy
from time import sleep
from datetime import datetime
import actionlib
#from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Pose

from ros_datacentre.message_store import MessageStoreProxy


class saveSweep():
    
    def __init__(self, name):
        rospy.loginfo("Starting %s", name)

        self.msg_store = MessageStoreProxy(collection='patrol_data')
        
        current_time = datetime.now()
        self.dt_text= current_time.strftime('%A, %B %d, at %H:%M hours')

        self.node_sub = rospy.Subscriber('/ptu_sweep/current_node', String, self.nodeCallback, None, 1)
        self.img_sub = rospy.Subscriber('ptu_sweep/rgb/image_color', Image, self.Callback, None, 1)
        self.pub_reg = rospy.Subscriber('/transform_pc2/depth/points', PointCloud2, self.Callback, None, 1)

        


    def nodeCallback(self, msg):
        self.waypoint = msg.data
        current_time = datetime.now()
        self.dt_text= current_time.strftime('%A, %B %d, at %H:%M hours')
        meta = {}
        meta["action"] = 'patrol_snapshot'
        meta["waypoint"] = self.waypoint
        meta["time"] = self.dt_text
        self.msg_store.insert(msg,meta)
        self.pose_sub = rospy.Subscriber('/robot_pose', Pose, self.PoseCallback, None, 1)
        self.received = True


    def PoseCallback(self, msg):
        meta = {}
        meta["action"] = 'patrol_snapshot'
        meta["waypoint"] = self.waypoint
        meta["time"] = self.dt_text
        self.msg_store.insert(msg,meta)
        self.pose_sub.unregister()
        self.received = True
        

    def Callback(self, msg):
        if not self.received :
            print "saving"
            meta = {}
            meta["action"] = 'patrol_snapshot'
            meta["waypoint"] = self.waypoint
            meta["time"] = self.dt_text
            self.msg_store.insert(msg,meta)
            self.received = True


if __name__ == '__main__':
    rospy.init_node("save_sweep")
    ps = saveSweep(rospy.get_name())
    rospy.spin()
