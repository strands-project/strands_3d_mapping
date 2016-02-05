#!/usr/bin/python

import rospy
from geometry_msgs.msg import PointStamped, Pose
from object_view_generator.srv import GetTrajectoryPoints, GetTrajectoryPointsRequest

class GeneratorTestClient(object):
    def __init__(self):
        self.min_dist = 0.5
        self.max_dist = 4.0
        self.number_views = 16
        self.inflation_radius = 0.4
        self.return_as_trajectory = False
        self._click_sub = rospy.Subscriber("/clicked_point", PointStamped, self.point_clicked)

        print "Waiting for generator service /generate_object_views..."
        rospy.wait_for_service("/generate_object_views")
        self._generator = rospy.ServiceProxy("/generate_object_views", GetTrajectoryPoints)
        print "ok."
        print ("Subscribed to /clicked_point. Use the tool in RViz "
               "to test the view generator. Subscribe to /object_view_goals to see the result.")
        
    def point_clicked(self, point):
        """ A point was clicked in RViz.... """
        pose = Pose()
        pose.position = point.point        
        self._generator(self.min_dist, self.max_dist, self.number_views, self.inflation_radius,
                        pose, self.return_as_trajectory, "")
    pass

rospy.init_node("object_view_generator_test_client")
gen = GeneratorTestClient()
while not rospy.is_shutdown():
    print "min_dist=",gen.min_dist
    print "max_dist=",gen.max_dist
    print "number_views=",gen.number_views
    print "inflation_radius=",gen.inflation_radius
    print "return_as_trajectory=",gen.return_as_trajectory
    print
    print "Change? For example typle min_dist=5 to change. 'q' to quit"
    inp=raw_input("> ")
    try:
        lhs,rhs = inp.split("=")
        print "Setting ",lhs," to ",rhs
        setattr(gen,lhs,float(rhs))
    except:
        print "oops. try again."
    if inp=="quit":
        break

