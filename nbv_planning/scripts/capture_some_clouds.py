#!/usr/bin/python

import rospy
import python_pcd.io as pcdio
import tf
from sensor_msgs.msg import PointCloud2

IN_FRAME = "/odom"

rospy.init_node("capture_some")
more=True
i=0
tfs = tf.TransformListener(400)
#rospy.sleep(10)
while more:
    inp=raw_input("Capture? 'q' to stop.")
    if inp=="q":
        more=False
    else:
        cloud = rospy.wait_for_message("/head_xtion/depth_registered/points",
                                       PointCloud2,
                                       10)
        try:
            tfs.waitForTransform(IN_FRAME, cloud.header.frame_id, cloud.header.stamp, rospy.Duration(1))
            frame = tfs.lookupTransform(IN_FRAME, cloud.header.frame_id, cloud.header.stamp)

            print "Capured a cloud at "
            print frame
            pcdio.write_pcd("view%d.pcd"%i,cloud,overwrite=True,viewpoint=frame[0]+(frame[1][3],)+frame[1][0:3])
            i+=1
        except Exception, e:
            print "TF time error! No capture."    
            print e

