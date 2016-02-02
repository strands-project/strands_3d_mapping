#!/usr/bin/env python
import rospy
from object_view_generator.srv import GetTrajectoryPoints, GetTrajectoryPointsResponse
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point32
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan, GetPlanRequest
import tf
from soma_roi_manager.soma_roi import SOMAROIQuery
import math
import numpy
import sys

################################################################
# Ray-casting algorithm
#
# adapted from http://rosettacode.org/wiki/Ray-casting_algorithm
################################################################
_eps = 0.00001

def ray_intersect_seg(p, a, b):
    ''' takes a point p and an edge of two endpoints a,b of a line segment returns boolean
    '''
    if a.y > b.y:
        a,b = b,a
    if p.y == a.y or p.y == b.y:
        p = Point32(p.x, p.y + _eps, 0)

    intersect = False

    if (p.y > b.y or p.y < a.y) or (
        p.x > max(a.x, b.x)):
        return False

    if p.x < min(a.x, b.x):
        intersect = True
    else:
        if abs(a.x - b.x) > sys.float_info.min:
            m_red = (b.y - a.y) / float(b.x - a.x)
        else:
            m_red = sys.float_info.max
        if abs(a.x - p.x) > sys.float_info.min:
            m_blue = (p.y - a.y) / float(p.x - a.x)
        else:
            m_blue = sys.float_info.max
        intersect = m_blue >= m_red
    return intersect

def is_odd(x):
    return x%2 == 1

def is_inside(p, poly):
    ln = len(poly)
    num_of_intersections = 0
    for i in range(0,ln):
        num_of_intersections += ray_intersect_seg(p, poly[i], poly[(i + 1) % ln])

    return is_odd(num_of_intersections)


class TrajectoryGenerator(object):
    def __init__(self):
        rospy.init_node('view_trajectory_generator')

        # subscribing to a map
        self.map_frame = rospy.get_param('~map_topic', '/waypoint_map')
        self.is_costmap = rospy.get_param('~is_costmap', False)
        rospy.loginfo("Sampling goals in %s", self.map_frame)

        # setting up the service
        self.ser = rospy.Service('/test_nav_goal', GetTrajectoryPoints,
                                 self.generate_trajectory_points)
        self.pub = rospy.Publisher("/nav_goals", PoseArray, queue_size=1)

        rospy.spin()
        rospy.loginfo("Stopped nav_goals_generator service")

    def process_map(self,data):

        # get map data
        self.resolution = data.info.resolution
        self.width = data.info.width
        self.height = data.info.height
        self.origin = data.info.origin
        self.data = data.data

        self.map_min_x = self.origin.position.x
        self.map_max_x = self.origin.position.x + self.width * self.resolution
        self.map_min_y = self.origin.position.y
        self.map_max_y = self.origin.position.y + self.height * self.resolution


    def generate_trajectory_points(self,req):
        rospy.loginfo('Incoming service request: %s', req)

        # get arguments
        self.inflation_radius = req.inflation_radius
        min_dist = req.min_dist
        max_dist = req.max_dist
        angle_jump = 2 * math.pi / req.number_views

        res = GetTrajectoryPointsResponse()

        try:
            rospy.loginfo("Getting a map...")
            msg = rospy.wait_for_message(self.map_frame, OccupancyGrid ,
                                         timeout=10.0)
            rospy.loginfo("got map.")
            self.process_map(msg)
        except rospy.ROSException, e:
            rospy.logwarn("Failed to get %s" % self.map_frame)
            return res

        # inflation radius must be positive
        if self.inflation_radius < 0:
            self.inflation_radius = 0.5
    
        self.inflated_footprint_size = int(self.inflation_radius / self.resolution) + 1

        # If req.region is filled then get the SOMA region from it
        self._poly = None
        if req.SOMA_region != "":
            try:
                soma_map, soma_conf, roi_id = req.SOMA_region.split("/")
                rospy.loginfo("Aiming for: %s, %s, %s"%(soma_map, soma_conf, roi_id))
                soma = SOMAROIQuery(soma_map, soma_conf)
                self._poly = soma.get_polygon(roi_id)
            except:
                rospy.logwarn("A soma region specified but can't decipher it, ignoring.")
                rospy.logwarn("Region: %s"% req.SOMA_region)

        # generate response
        pose = req.target_pose
        for i in range(req.number_views):
            angle = i * angle_jump
            p =  Pose()
            for j in range(100):
                rad = j / 100.0 * (max_dist - min_dist) + min_dist
                x = math.sin(angle) * rad + pose.position.x
                y = math.cos(angle) * rad + pose.position.y
                p.orientation.w=1 # default
                p.position.x = x
                p.position.y = y
                if self._poly is not None and not is_inside(p.position, self._poly.points):
                    continue
                if self.test_pose(p):
                    break
            else:
                #rospy.loginfo("Angle %f had no hopper." % angle)
                continue
            res.goals.poses.append(p)
        res.goals.header.frame_id = "/map"
        if len(res.goals.poses) < 1 :
            return res
        # Order the goals in visiting sequence
        # Already in clockwise order, find the longest between two adjacents
        # and cut it there.
        get_plan =  rospy.ServiceProxy("/move_base/make_plan", GetPlan)
        request = GetPlanRequest()
        request.start.header.frame_id = "/map"
        request.goal.header.frame_id = "/map"
        sub_plans = []
        for i in range(0, len(res.goals.poses)):
            print "Testing between %d and %d" % (i - 1, i)
            request.start.pose.position = res.goals.poses[i-1].position
            request.goal.pose.position = res.goals.poses[i].position
            plan = get_plan(request)
            sub_plans.append(self.plan_length(plan.plan))
            print "sub-plan length:", self.plan_length(plan.plan)
        start = numpy.argmax(sub_plans)
#        start=max(start-1,0)
        poses=res.goals.poses[start:]
        poses.extend(res.goals.poses[:start])

        # which is closest to current position; choose as start and strim the
        # shorter direction off. this is not so nice as it wastes views
        robot_pose = rospy.wait_for_message("/robot_pose", Pose)
        print "Robot at ", robot_pose
        dists = map(lambda x: (x.position.x-robot_pose.position.x)**2 +
                    (x.position.y-robot_pose.position.y)**2, poses)
        start = numpy.argmin(dists)
        if start > len(poses)/2:
            poses = [a for a in reversed(poses[:start+1])]
        else:
            poses = poses[start:]

        for i in range(len(poses)-1):
            angle = math.atan2(poses[i+1].position.y - poses[i].position.y,
                               poses[i+1].position.x - poses[i].position.x)
            print "Angle = ",angle
            quaternion = tf.transformations.quaternion_from_euler(0,0, angle)
            print " - - > quaternion=",quaternion
            poses[i].orientation.x = quaternion[0]
            poses[i].orientation.y = quaternion[1]
            poses[i].orientation.z = quaternion[2]
            poses[i].orientation.w = quaternion[3]
        if len(poses)>1:
            poses[-1].orientation.x = poses[-2].orientation.x
            poses[-1].orientation.y = poses[-2].orientation.y
            poses[-1].orientation.z = poses[-2].orientation.z
            poses[-1].orientation.w = poses[-2].orientation.w
        else:
            poses=[]
                               
            
        res.goals.poses=poses
        self.pub.publish(res.goals)
        
        return res
    
    def plan_length(self, plan):
        tot = 0
        for i in range(1, len(plan.poses)):
            tot += math.sqrt(pow(plan.poses[i].pose.position.x -
                                 plan.poses[i-1].pose.position.x, 2)+
                             pow(plan.poses[i].pose.position.y -
                                 plan.poses[i-1].pose.position.y, 2))
        return tot

    def test_pose(self, pose):
        cell_x = int((pose.position.x - self.origin.position.x) / self.resolution)
        cell_y = int((pose.position.y - self.origin.position.y) / self.resolution)
        if not self.in_collision(cell_x, cell_y):
            return True
    
        return False

    def cell(self, x,y):
        if x < 0 or y <0 or x >= self.width or y >= self.height:
            # return 'unknown' if out of bounds
            return -1

        return self.data[x +  self.width * y]


    def in_collision(self,x,y):
        x_min = x - self.inflated_footprint_size
        x_max = x + self.inflated_footprint_size
        y_min = y - self.inflated_footprint_size
        y_max = y + self.inflated_footprint_size

        for i in range(x_min,x_max):
            for j in range(y_min,y_max):
                if self.is_costmap:
                    if (self.cell(i,j) > 97 or self.cell(i,j)==-1):
                        return True
                else:
                    if (self.cell(i,j) != 0):
                        return True
        return False


if __name__ == '__main__':
    TrajectoryGenerator()
