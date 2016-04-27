import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import math
import json

from state import World, Object

# Import opencv
import cv
import cv2
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
import numpy as np
import geometry
from geometry_msgs.msg import Pose

from observation import MessageStoreObject, Observation, TransformationStore

from mongodb_store.message_store import MessageStoreProxy
from robblog.msg import RobblogEntry
import robblog.utils
from robblog import utils as rb_utils
import datetime

import geometry

def generate_report(t, parent_object=None):
    """
    parent_object: string, parent id, only children of this are included
    t: time, float
    """
    # what objects existed at time t 
    w = World()
    if parent_object is None:
        objects = w.get_root_objects()
    else:
        objects = [w.get_object(parent_object)]
        
    for o in objects:
        print "-> ", o.name, " ==> ", o.identification.class_type
        for o2 in [w.get_object(ob) for ob in o.get_children_names()]:
            tm1 = datetime.datetime.fromtimestamp(o2._life_start).strftime('%m-%d %H:%M:%S.%f')
            if o2._life_end is None:
                tm2 = "0"
            else:
                tm2 = datetime.datetime.fromtimestamp(o2._life_end).strftime('%m-%d %H:%M:%S.%f')
            print "-> -> ", o2.name, "[", tm1, "->", tm2, "]", " ==> ", o2.identification.class_type
    
def epoch_to_str_time(epoch):
    return datetime.datetime.fromtimestamp(epoch).strftime('%Y-%m-%d %H:%M:%S')
    
def generate_table_list(parent_object=None):
    w = World()
    objects = w.get_objects_of_type("Table")
    ob_count = 0
    bugger = 0
    oblongs = []
    for o in objects:
        print "-> ", o.name, " ==> ", o.identification.class_type
        for ob in o._observations:
            assert isinstance(ob,  Observation)
            print "< -- > Observation :", epoch_to_str_time(ob.stamp)
            children = w.get_children(o.name, {'_observations': {'$elemMatch': {'stamp': ob.stamp}}})
            ob_count += 1
            for o2 in children:
                print "-> -> ", o2.name, " ==> ", o2.identification.class_type
            print "-" * 20
            bridge = CvBridge()
            ob_ok = []
            for m in ob._messages.keys():
                msg =  ob.get_message(m)
                if msg is None:
                    ob_ok.append(False)
                else:
                    ob_ok.append(True)
            message =  ob.get_message("/head_xtion/rgb/image_color")
            if message is None:
                bugger += 1
                rospy.logerr("table image lost")
                rospy.logwarn("msg store object: "+str(ob._messages["/head_xtion/rgb/image_color"]))
                oblongs.append((ob.stamp, ob_ok))
                continue
            oblongs.append((ob.stamp, ob_ok))
            rgb_image = bridge.imgmsg_to_cv2(message)
            cv2.imshow('image', rgb_image)
            k = cv2.waitKey(10)
            print k
            if k == 98:
                break
        

    print "Total table observations:", ob_count
    print "Buggered: ", bugger
    
    oblongs.sort(key=lambda x: x[0])
    for o in oblongs:
        print epoch_to_str_time(o[0]), o[1]

def get_tables_list(ignore_non_observed=True):
    w =  World()
    objects = w.get_objects_of_type("Table")
    if ignore_non_observed:
        return [o.name for o in objects if len(o.get_children_names())>0]
    else:
        return [o.name for o in objects]

def get_table_observations(table_name):
    w =  World()
    table =  w.get_object(table_name)
    observations =  []
    obby = []
    lst = 0
    for ob in table._observations:
        assert isinstance(ob,  Observation)
        if ob.stamp - lst > 5 * 60 and len(obby) > 0:
            observations.append(obby)
            obby = []
        lst = ob.stamp
        obby.append((ob.stamp, epoch_to_str_time(ob.stamp)))
    observations.append(obby)
    return observations

def get_objects_observed(table_name, observation_stamp):
    w = World()
    children = w.get_children(table_name,
                              {'_observations': {'$elemMatch': {'stamp': observation_stamp}}})
    return children


def get_table_observation_rgb(table_name, observation_timestamp):
    # what objects existed on the table at this timestamp..
    w = World()
    table =  w.get_object(table_name)
    # children = w.get_children(table_name, {'$and': [
                                   #{'_life_start': {'$lt': timestamp} },
                                    #{'$or': [{'_life_end': {'$gt': timestamp}},
                                            #{'_life_end': None } ]}
                                    #] })
    # Which table observation is closest to timestamp
    if len(table._observations) < 1:
        raise Exception("Table has no observations")
    closest =  min([(ob, math.fabs(ob.stamp - observation_timestamp)) for ob in table._observations],
                   key=lambda x: x[1])
    rospy.loginfo("Closest observation of table '%s' to time %d was %d"%(table_name,
                                                                        observation_timestamp,
                                                                        closest[1]))
    observation = closest[0]
    tf =  TransformationStore.msg_to_transformer(observation.get_message("/tf"))
    camera_info =  observation.get_message("/head_xtion/rgb/camera_info")
    bridge = CvBridge()
    rgb_image = bridge.imgmsg_to_cv2(observation.get_message("/head_xtion/rgb/image_color"))
    return rgb_image

def create_table_observation_image(table_name, observation_timestamp):
    """
    returns numpy/opencv image
    """
    # what objects existed on the table at this timestamp..
    w = World()
    table =  w.get_object(table_name)
    # children = w.get_children(table_name, {'$and': [
                                   #{'_life_start': {'$lt': timestamp} },
                                    #{'$or': [{'_life_end': {'$gt': timestamp}},
                                            #{'_life_end': None } ]}
                                    #] })
    # Which table observation is closest to timestamp
    if len(table._observations) < 1:
        raise Exception("Table has no observations")
    closest =  min([(ob, math.fabs(ob.stamp - observation_timestamp)) for ob in table._observations],
                   key=lambda x: x[1])
    rospy.loginfo("Clost observation of table '%s' to time %d was %d"%(table_name,
                                                                        observation_timestamp,
                                                                        closest[1]))
    observation = closest[0]
    tf =  TransformationStore.msg_to_transformer(observation.get_message("/tf"))
    camera_info =  observation.get_message("/head_xtion/rgb/camera_info")
    bridge = CvBridge()
    rgb_image = bridge.imgmsg_to_cv2(observation.get_message("/head_xtion/rgb/image_color"))
          
    children = w.get_children(table_name, {'_observations': {'$elemMatch': {'stamp': observation.stamp}}})
    colours = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 0, 255),
               (0, 255, 255), (255, 255, 0)   ]
    for j, c in enumerate(children):
        print c.name, ' ==> ', c.identification.class_type
        pointcloud = c._point_cloud.retrieve()
        print c._point_cloud.obj_id
        world_to_rgb = tf.lookupTransform(camera_info.header.frame_id, "/map", 
                                                pointcloud.header.stamp)
        world_to_rgb = geometry.Pose(geometry.Point(*(world_to_rgb[0])),
                                   geometry.Quaternion(*(world_to_rgb[1])))
        transform = np.dot(world_to_rgb.as_homog_matrix(),
                           table.pose.as_homog_matrix() )
        transform = np.dot(transform, c.pose.as_homog_matrix())
        rospy.loginfo("Transforming object pointcloud to map")
        pointcloud_transformed = geometry.transform_PointCloud2(pointcloud,
                                                                transform,
                                                                '/map')
        contour =  get_image_contour(camera_info, pointcloud_transformed)
        cv2.drawContours(rgb_image, [contour], 0, colours[j%len(colours)], 2)
        classification = "%s (%f)" % (c.identification.class_type[0], c.identification.class_type[1])
        print contour[0]
        cv2.putText(rgb_image,  classification, (contour[0][0][0], contour[0][0][1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, colours[j%len(colours)], 2)
        
#    cv2.imshow('image', rgb_image)
#    cv2.waitKey(30)
    return rgb_image

def get_next_stamp(table_name, observation_timestamp):
    """
    return the timestamp for the observation of the table after the given one.
    """
    w = World()
    table =  w.get_object(table_name)
    # Which table observation is closest to timestamp going forward
    if len(table._observations) < 1:
        raise Exception("Table has no observations")
    forwards =  [ (ob, (ob.stamp - observation_timestamp))
                    for ob in table._observations
                    if ob.stamp > observation_timestamp ] 

    if len(forwards) < 1:
        return None
    if len(forwards) == 1:
        return forwards[0]
    return min(*forwards,  key=lambda x: x[1])

def get_prev_stamp(table_name, observation_timestamp):
    """
    return the timestamp for the observation of the table before the given one.
    """

    w = World()
    table =  w.get_object(table_name)
    # Which table observation is closest to timestamp going forward
    if len(table._observations) < 1:
        raise Exception("Table has no observations")
    backwards =  [(ob, ( observation_timestamp - ob.stamp))
                    for ob in table._observations
                    if ob.stamp < observation_timestamp ]
    if len(backwards) < 1:
        return None
    if len(backwards) == 1:
        return backwards[0]

    return min(*backwards,  key=lambda x: x[1])
        
def get_tabletop_object_clouds(table_name, observation_timestamp):
    """
    returns list of sensor_msgs/PointCloud2, in /map frame.
    """
    # what objects existed on the table at this timestamp..
    w = World()
    table =  w.get_object(table_name)

    # Which table observation is closest to timestamp
    if len(table._observations) < 1:
        raise Exception("Table has no observations")
    closest =  min([(ob, math.fabs(ob.stamp - observation_timestamp)) for ob in table._observations],
                   key=lambda x: x[1])
    rospy.loginfo("Clost observation of table '%s' to time %d was %d"%(table_name,
                                                                        observation_timestamp,
                                                                        closest[1]))
    observation = closest[0]
    tf =  TransformationStore.msg_to_transformer(observation.get_message("/tf"))
    camera_info =  observation.get_message("/head_xtion/depth/camera_info")

    children = w.get_children(table_name, {'_observations': {'$elemMatch': {'stamp': observation.stamp}}})

    clouds = []
    for j, c in enumerate(children):

        pointcloud = c._point_cloud.retrieve()

        transform = np.dot(table.pose.as_homog_matrix(), c.pose.as_homog_matrix())
        rospy.loginfo("Transforming object pointcloud to map")
        pointcloud_transformed = geometry.transform_PointCloud2(pointcloud,
                                                                transform,
                                                                '/map')
        clouds.append(pointcloud_transformed)
        
    return clouds
        
    
def get_image_contour(camera_info, pointcloud, pt_meld=1):
    pinhole = PinholeCameraModel()
    pinhole.fromCameraInfo(camera_info)
    img =  np.zeros((camera_info.height, camera_info.width, 1), np.uint8)
    for pt in pc2.read_points(pointcloud): # assume x,y,z
        u, v = pinhole.project3dToPixel(pt)
        cv2.circle(img, (int(u), int(v)), pt_meld, 255, -1)
        img[v, u] = 255
    contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    return contours[0]

def create_robblog(table_name,  timestamp):
    """ creates a robblog entry for observation at timestamp (or closest)"""
    msg_store_blog = MessageStoreProxy(collection='robblog')
    entry =  "# Table Observation\ntable name: %s\n"
    e = RobblogEntry(title=datetime.datetime.now().strftime("%H:%M:%S") + ' - Table Observation' )
    e.body = 'I looked at ' + table_name + ' and this is what I found:\n\n'
    img =  create_table_observation_image(table_name, timestamp)
    bridge = CvBridge()
    ros_img = bridge.cv2_to_imgmsg(img)
    img_id = msg_store_blog.insert(ros_img)
    e.body += '![Image of the door](ObjectID(%s))' % img_id
    msg_store_blog.insert(e)
        
        
def calculate_qsrs(table_name, timestamp):
    """
    calculates qsrs, returns them but also triggers the qsr prolog stuff.
    """
    try:
        import strands_qsr_learned_qualitators.qualitators as qualitators
        import strands_qsr_learned_qualitators.geometric_state as gs
    except:
        rospy.logerr("strands_qsr repo not on catkin workspace?")
        return None
    try:
        import qsr_kb.srv
    except:
        rospy.logerr("qsr_kb package not available?")
        return None
    
    
    msg_store = MessageStoreProxy(collection='frozen_tables')
    viewpoints = msg_store.query(Pose._type)
    for v, m in viewpoints:
        if m["name"][:-10] == table_name:
            print "Matched"
            break
    else:
        rospy.logerr("Oh crap, viewpoint does not exist!")
        return []
    viewpoint = geometry.Pose.from_ros_msg(v)
    
    w = World()
    table = w.get_object(table_name)
    
    QUAL_FILE = "/home/chris/review.qsrs"
    qsrs = qualitators.Qualitators.load_from_disk(QUAL_FILE)
    print "Loaded ", len(qsrs._qualitators), "qualitators from disk"
    
    print timestamp
    objs = get_objects_observed(table_name, timestamp)
    print objs
    geo =  gs.GeometricState("scene")
    rels = []
    pretty_rels = []
    for o in objs:
        # Before adding the object, transform it from table to viewer frame.
        pose = geometry.Pose.from_homog(np.dot(np.linalg.inv(viewpoint.as_homog_matrix()), o.pose.as_homog_matrix()))
        
        geo.add_object(o.name, o.identification.class_type[0],
                       pose.position, pose.quaternion,
                       o._bounding_box.points)
        print "Adding object ", o.name
        
    
    for ob1 in geo._objects.keys():
        # Get camera position in ob1 frame
        #cam_pos = [geo._objects[ob1].position.x - viewpoint.position.x,
                   #geo._objects[ob1].position.y - viewpoint.position.y,
                   #geo._objects[ob1].position.z - viewpoint.position.z]
        #cam_pos = [viewpoint.position.x,
                   #viewpoint.position.y,
                   #viewpoint.position.z]
        cam_pos = [0, 0, 0] # is now zeros since objects are in its frame.
        ##None
        for ob2 in geo._objects.keys():
            if ob1 == ob2:
                continue
            print ob1, " - ", ob2
            delta = [[[geo._objects[ob1].position.x, geo._objects[ob1].position.y, geo._objects[ob1].position.z], ["quaternion?"]],
                     [[geo._objects[ob2].position.x, geo._objects[ob2].position.y, geo._objects[ob2].position.z], ["quaternion?"]],
                     ["UNUSED ELEMENT !"]] #geo._objects[ob2].position.x - geo._objects[ob1].position.x,
                     #geo._objects[ob2].position.y - geo._objects[ob1].position.y,
                     #geo._objects[ob2].position.z - geo._objects[ob1].position.z
                     #]
            s1 = geo._objects[ob1].bbox.get_size()
            s2 = geo._objects[ob2].bbox.get_size()
            for q in qsrs._qualitators:
                if q.dimensions == 2:
                    print q.name, "? vp=", cam_pos, "; o1=", delta[0][0], "; o2=", delta[1][0]
                    if q(delta+list(s1)+list(s2), cam_pos):
                        rels.append([str(q.name), str(ob1), str(ob2)])
                        pretty_rels.append([str(q.name),
                                            str(geo._objects[ob1].obj_type)+"["+str(ob1)+"]",
                                            str(geo._objects[ob2].obj_type)+"["+str(ob2)+"]"])
        #for q in qual._qualitators:
            #if q.dimensions == 1:
                #s1 = geo._objects[ob1].bbox.get_size()
                #rel_probs.update_probs_new_example(q.name,
                                                   #(geo._objects[ob1].obj_type, ),
                                                   #q(geo._objects[ob1].position+list(s1)))
                
    
    #rels = [['left-of', 'keyboard', 'cup'], ['left-of', 'monitor', 'cup'],
                            #['behind', 'keyboard', 'cup'], ['in-front-of', 'monitor', 'cup'],
                            #['in-front-of', 'keyboard', 'monitor'], ['right-of', 'cup', 'monitor']]

    loc = table_name

    cls =  []
    pose = []
    for o in objs:
        oo = geometry.Pose.from_homog(np.dot(table.pose.as_homog_matrix(), o.pose.as_homog_matrix()))
        cls.append([str(o.name), str(o.identification.class_type[0]),str(o.identification.class_type[1]) ])
        pose.append([str(o.name), [[oo.position.x, oo.position.y, oo.position.z],
                              [oo.quaternion.x, oo.quaternion.y, oo.quaternion.z, oo.quaternion.w]]])

    cls =  [['BU', cls]]
    #cls = [['BU', [['keyboard', 'Keyboard', 0.8], ['keyboard', 'Monitor', 0.2], 
                                    #['cup', 'Cup', 0.4], ['cup', 'Mouse', 0.5], ['cup', 'Keyboard', 0.1], 
                                    #['monitor', 'Keyboard', 0.1], ['monitor', 'Monitor', 0.9], 
                                    #['mouse', 'Cup', 0.9], ['mouse', 'Mouse', 0.1]]], 
           #['TD', [['keyboard', 'Keyboard', 0.9], ['keyboard', 'Monitor', 0.1], 
                                    #['cup', 'Cup', 0.6], ['cup', 'Mouse', 0.2], ['cup', 'Keyboard', 0.2], 
                                    #['monitor', 'Keyboard', 0.1], ['monitor', 'Monitor', 0.9], 
                                    #['mouse', 'Cup', 0.1], ['mouse', 'Mouse', 0.9]]]]

    ## pose [[x,y,z], [w, x, y, z]], ... ]
    #pose = [ ['monitor', [[1.0,0.0,0.0],[1,0,0,0]]], 
             #['cup', [[0.5,1.0,0.0],[1,0,0,0]]], 
             #['mouse', [[0.5,-0.5,0.0],[1,0,0,0]]],
             #['keyboard', [[0.0,0.0,0.0],[1,0,0,0]]] ]

    def make_query(query):
        try:
            rospy.wait_for_service('qsrvis', 10)
        except:
            rospy.logerr("Can't get qsr kb services.")
            return rels
        try:
            swipl = rospy.ServiceProxy('qsrvis', qsr_kb.srv.PrologQuery)
            resp = swipl(query)
            return json.loads(resp.solution)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    def create_event(relations, location, classifications, pose):
        return "create_event(EVT," + str(relations) + "," + str(location) + "," + str(classifications) + "," + str(pose) + "), new_vis." 


    query = create_event(rels, loc, cls, pose)
    make_query(query)
    
    return pretty_rels

class PointCloudVisualiser(object):
    def __init__(self, topic="pointcloud_visualise"):
        self._pointclouds = []
        self._pub = rospy.Publisher(topic, PointCloud2)
    
    def add_cloud(self, cld):
        assert isinstance(cld, PointCloud2)
        if len(self._pointclouds) > 0:
            assert cld.point_step == self._pointclouds[0].point_step
            # TODO: dirty assumption of the fields being the same
            # TODO: dirty assumption of header being the same
        self._pointclouds.append(cld)
    
    def clear(self):
        self._pointclouds = []
        
    def publish(self):
        if len(self._pointclouds) < 1:
            return
        pts = []
        colours = [0xFF0000, 0x00FF00, 0x0000FF, 
                   0xFFFF00, 0xFF00FF, 0x00FFFF]
        for i, cloud in enumerate(self._pointclouds):
            rgb =  colours[i%len(colours)]
            pts.extend([ (p[0], p[1], p[2], rgb) for p in pc2.read_points(cloud,
                                                                 ['x', 'y', 'z']) ])
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1), 
                  PointField('rgb', 12, PointField.UINT32, 1)]
        pointcloud = pc2.create_cloud(self._pointclouds[0].header,
                                      fields, pts)
        self._pub.publish(pointcloud)
