import rospy
from mongo import MongoTransformable
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovariance, TransformStamped
import copy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import math
from operator import itemgetter

     
def mat_to_quat(m):
    """ http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    takes a numpy matrix and gives ROS quaternion"""
    tr = m[0, 0] + m[1, 1] + m[2, 2]

    if (tr > 0): 
        S = math.sqrt(tr+1.0) * 2
        qw = 0.25 * S
        qx = (m[2, 1] - m[1, 2]) / S
        qy = (m[0, 2] - m[2, 0]) / S 
        qz = (m[1, 0] - m[0, 1]) / S 
    elif ((m[0,0] > m[1,1])&(m[0,0] > m[2,2])):
        S = math.sqrt(1.0 + m[0,0] - m[1,1] - m[2,2]) * 2
        qw = (m[2,1] - m[1,2]) / S
        qx = 0.25 * S
        qy = (m[0,1] + m[1,0]) / S
        qz = (m[0,2] + m[2,0]) / S
    elif (m[1,1] > m[2,2]):
        S = math.sqrt(1.0 + m[1,1] - m[0,0] - m[2,2]) * 2; 
        qw = (m[0,2] - m[2,0]) / S;
        qx = (m[0,1] + m[1,0]) / S; 
        qy = 0.25 * S;
        qz = (m[1,2] + m[2,1]) / S; 
    else:
        S = math.sqrt(1.0 + m[2,2] - m[0,0] - m[1,1]) * 2; 
        qw = (m[1,0] - m[0,1]) / S;
        qx = (m[0,2] + m[2,0]) / S;
        qy = (m[1,2] + m[2,1]) / S;
        qz = 0.25 * S;
        
    quat =  Quaternion()
    quat.x = qx
    quat.y = qy
    quat.z = qz
    quat.w = qw
    return quat

class Point(MongoTransformable):
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def as_numpy(self):
        return np.array([self.x, self.y, self.z])
    
    @classmethod
    def from_ros_point32(cls, point32):
        return cls(point32.x, point32.y, point32.z)
    
    
    def transform(self, transform):
        if isinstance(transform, Pose):
            transform = transform.as_homog_matrix()
        p = np.array([self.x, self.y, self.z, 1])
        self.x, self.y, self.z = np.dot(transform, p)[0:3]
        
    def __str__(self):
        return "(%f, %f, %f)" % (self.x, self.y, self.z)
    
class Quaternion(MongoTransformable):
    def __init__(self, x=0, y=0, z=0, w=1):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
    
    @property
    def as_numpy(self):
        return np.array([self.x, self.y, self.z, self.w])
        
class Pose(MongoTransformable):
    def __init__(self, position=None, quaternion=None):
        self.stamp = rospy.Time.now().to_time()
        self.ros_frame_id = ""
        if position is not None:
            self.position = copy.deepcopy(position)
        else:
            self.position = Point()
            
        if quaternion is not None:
            self.quaternion = copy.deepcopy(quaternion)
        else:
            self.quaternion = Quaternion()
        
    @classmethod
    def create_zero(cls, timestamp=None):
        p = cls() #Point(0, 0, 0), Quaternion(0, 0, 0, 1))
        if timestamp is not None:
            p.stamp = timestamp
        return p
    
    @classmethod
    def from_ros_msg(cls, ros_msg):
        #assert (isinstance(ros_msg, Pose) or isinstance(ros_msg, PoseStamped)
                #or isinstance(ros_msg, PoseWithCovariance))
        pose = ros_msg
        p = cls()
        if hasattr(pose, "pose"):
            p.stamp = pose.header.stamp.to_time()
            p.ros_frame_id = pose.header.frame_id
            pose = pose.pose
        if hasattr(pose, "transform"):
            p.stamp = pose.header.stamp.to_time()
            p.ros_frame_id = pose.header.frame_id
            pose = pose.transform
            
        if hasattr(pose, "position"):
            pos = pose.position
        else:
            pos = pose.translation
        if hasattr(pose, "orientation"):
            rot = pose.orientation
        else:
            rot = pose.rotation
        p.position.x = pos.x
        p.position.y = pos.y
        p.position.z = pos.z
        p.quaternion.x = rot.x
        p.quaternion.y = rot.y
        p.quaternion.z = rot.z
        p.quaternion.w = rot.w
        return p

    @classmethod
    def from_homog(cls, homo):
        quat = mat_to_quat(homo)
        pos = Point(homo[0, 3], homo[1, 3], homo[2, 3])
        return cls(pos, quat)
    
    
    def __str__(self):
        return "{0:d}".format(int(self.stamp*1000))
    
    def as_homog_matrix(self):
        """Return homogeneous transformation matrix for this pose.
        """
        # Borrowed from rospy tf.transformations code
        q = self.quaternion.as_numpy
        nq = np.dot(q, q)
        if nq < np.spacing(0):
            return np.identity(4)
        q *= math.sqrt(2.0 / nq)
        q = np.outer(q, q)
        return np.array((
            (1.0-q[1, 1]-q[2, 2], q[0, 1]-q[2, 3], q[0, 2]+q[1, 3],  self.position.x),
            ( q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2], q[1, 2]-q[0, 3], self.position.y),
            ( q[0, 2]-q[1, 3], q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], self.position.z),
            ( 0.0, 0.0, 0.0, 1.0)
            ), dtype=np.float64)
    
    def to_ros_tf(self):
        t = TransformStamped()
        t.header.stamp = rospy.Time.from_sec(self.stamp)
        t.header.frame_id = self.ros_frame_id
        t.child_frame_id = ""
        tran = t.transform.translation
        tran.x, tran.y, tran.z = self.position.as_numpy()
        rot = t.transform.rotation
        rot.x, rot.y, rot.z, rot.w = self.quaternion.as_numpy
        return t
    
    def to_ros_pose(self):
        p = PoseStamped()
        p.header.stamp = rospy.Time.from_sec(self.stamp)
        p.header.frame_id = self.ros_frame_id
        p.child_frame_id = ""
        tran = p.pose.position
        tran.x, tran.y, tran.z = self.position.as_numpy
        rot = p.pose.orientation
        rot.x, rot.y, rot.z, rot.w = self.quaternion.as_numpy

        return p


class BBoxArray(MongoTransformable):
    """ Bounding box of an object
    """
    def __init__(self, bbox=None):
        if bbox is None:
            return # TODO
        self.points = bbox
        # Calc x_min and x_max for obj1
        x_sorted = sorted(bbox, key=itemgetter(0))
        self._x_min = x_sorted[0][0]
        self._x_max = x_sorted[7][0]

        # Calc y_min and y_max for obj
        y_sorted = sorted(bbox, key=itemgetter(1))
        self._y_min = y_sorted[0][1]
        self._y_max = y_sorted[7][1]

        # Calc z_min and z_max for obj
        z_sorted = sorted(bbox, key=itemgetter(2))
        self._z_min = z_sorted[0][2]
        self._z_max = z_sorted[7][2]
        
    @property
    def x_min(self):
        return self.x_min

    @property
    def x_max(self):
        return self.x_max

    @property
    def y_min(self):
        return self.y_min

    @property
    def y_max(self):
        return self.y_max
    
    @property
    def z_min(self):
        return self.z_min

    @property
    def z_max(self):
        return self.z_max
    
    @property
    def size(self):
        return (self.x_max - self.x_min,
                self.y_max - self.y_min,
                self.z_max - self.z_min )
    
    @property
    def volume(self):
        return reduce(lambda x, y: x*y, self.size)
                

def transform_PointCloud2(cloud, transform, new_frame):
    """ transforms all the points, returns new pc2.
    transform: Pose object
    """
    if isinstance(transform, Pose):
        transform = transform.as_homog_matrix()
    print transform
    def transform_point(pt):
        return np.dot(transform, np.array([pt[0], pt[1], pt[2], 1]))[0:3]
    
    pts = [tuple(transform_point(p[:3])) + p[3:] for p in pc2.read_points(cloud,
                                                                   ['x', 'y', 'z', 'rgb'])]

    header = copy.deepcopy(cloud.header)
    header.frame_id = new_frame
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1), 
              PointField('rgb', 12, PointField.FLOAT32, 1)]
    newcloud = pc2.create_cloud(header, fields, pts)
    #newcloud = pc2.create_cloud_xyz32(header, pts)
    
    return newcloud
