import mongo
import rospy
import tf, tf2_msgs.msg
import cPickle as pickle
import zlib
from collections import deque

from sensor_msgs.msg import Image, PointCloud2, CameraInfo, JointState
from geometry_msgs.msg import PoseWithCovarianceStamped
from mongodb_store.message_store import MessageStoreProxy
from mongodb_store_msgs.msg import SerialisedMessage
from exceptions import StateException

DEFAULT_TOPICS = [("/amcl_pose", PoseWithCovarianceStamped),
                  ("/head_xtion/rgb/image_color", Image), 
                  ("/head_xtion/rgb/camera_info", CameraInfo), 
                  ("/head_xtion/depth/points", PointCloud2),
                  ("/head_xtion/depth/camera_info", CameraInfo),
                  ("/ptu/state", JointState)]

class TransformationStore(object):
    """
    Subscribes to /TF, stores transforms, pickleable for datacentre, turns into
    transformer when needed
    """
    def __init__(self):
        self._transformations = deque([])
        self._lively = False
        self._max_buffer = 10

    def cb(self, transforms):
        time_window = rospy.Duration(self._max_buffer)
        for transform in transforms.transforms:
            #rospy.loginfo("Got transform: %s - > %s"% ( transform.header.frame_id, transform.child_frame_id))
            if self._max_buffer > 0 and len(self._transformations) > 2:
                l =  self._transformations.popleft()
                if (transform.header.stamp -  l.header.stamp) < time_window:
                    self._transformations.appendleft(l)
            self._transformations.append(transform)
            
    @classmethod
    def create_from_transforms(cls, transforms):
        """
        Create a store from a given set of transforms
        transforms: Must be a list of TransformStamped messages
        """
        slf = cls()
        for t in transforms:
            slf._transformations.append(t)
        return slf
        

    @classmethod
    def create_live(cls, max_buffer=10.0):
        # subscribe to tf and store transforms
        slf = cls()
        slf._max_buffer = max_buffer
        slf._sub =  rospy.Subscriber("/tf", tf2_msgs.msg.TFMessage, slf.cb)
        slf._lively = True
        return slf
    
    def pickle(self):
        if self._lively:
            self._lively = False
            self._sub.unregister()
            del self._sub
        return zlib.compress(pickle.dumps(self, protocol=pickle.HIGHEST_PROTOCOL))
    
    def pickle_to_msg(self):
        s = SerialisedMessage()
        s.msg =  self.pickle()
        s.type = "zlibed_pickled_tf"
        return s
    
    @staticmethod
    def msg_to_transformer(msg):
        t = tf.TransformerROS()
        transforms = TransformationStore.unpickle(msg.msg)
        for transform in transforms._transformations:
            t.setTransform(transform)
        return t
    
    @classmethod
    def unpickle(cls, pickle_string):
        return pickle.loads(zlib.decompress(pickle_string))
        
        

class MessageStoreObject(mongo.MongoTransformable):
    def __init__(self,  database="message_store", collection="message_store",
                 obj_id=None, typ=None):
        self.database = database
        self.collection = collection
        self.obj_id = obj_id
        self.typ = typ
        
    def retrieve(self):
        proxy = MessageStoreProxy(database=self.database,
                                  collection=self.collection)
        return proxy.query_id(self.obj_id, self.typ)[0]
    
    @classmethod
    def create(cls, ros_msg, database="message_store", collection="ws_observations"):
        proxy = MessageStoreProxy(database=database, collection=collection)
        o_id = proxy.insert(ros_msg)
        print o_id
        return cls(database, collection, o_id, ros_msg._type)
    
    def __str__(self):
        return "MessageStoreObject(collection=%s, db=%s, obj_id=%s)" % (self.collection, self.database, self.obj_id)

class Observation(mongo.MongoTransformable):
    def __init__(self):
        self.stamp = rospy.Time.now().to_time()
        self._messages = {}
    
    @classmethod    
    def make_observation(cls, topics=DEFAULT_TOPICS):
        """
        topics: list of tuples (topic_name,topic_type)
        """
        observation = cls()
        message_proxy = MessageStoreProxy(collection="ws_observations")
        transforms = TransformationStore.create_live()
        rospy.sleep(0.5)
        for topic_name, topic_type in topics:
            rospy.loginfo("Aquiring message on %s [%s]"%(topic_name, topic_type._type))
            try:
                msg = rospy.wait_for_message(topic_name, topic_type, timeout=10.0)
            except rospy.ROSException, e:
                rospy.logwarn("Failed to get %s" % topic_name)
                continue
            msg_id = message_proxy.insert(msg)
            observation._messages[topic_name] = MessageStoreObject(
                database=message_proxy.database,
                collection=message_proxy.collection,
                obj_id=msg_id,
                typ=msg._type)
            
        rospy.sleep(0.5)
        tf_data =  transforms.pickle_to_msg()
        msg_id = message_proxy.insert(tf_data)
        observation._messages["/tf"]  = MessageStoreObject(
            database=message_proxy.database,
            collection=message_proxy.collection,
            obj_id=msg_id,
            typ=tf_data._type)
        rospy.loginfo("TF size: %dK" % (len(tf_data.msg)/1024) )
        return observation

    @classmethod
    def make_observation_from_messages(cls, messages):
        """
        Creates a new observation form a list of ros messages.
        message: list of tuples (topic,message)
        """
        observation = cls()
        message_proxy = MessageStoreProxy(collection="ws_observations")
        for topic, message in messages:
            msg_id = message_proxy.insert(message)
            observation._messages[topic]  = MessageStoreObject(
                database=message_proxy.database,
                collection=message_proxy.collection,
                obj_id=msg_id,
                typ=message._type)
        return observation

    @classmethod
    def copy(cls, instance):
        """
        Create a copy of the instance. The MessageStoreObjects will not be
        dupicated.
        """
        o = cls()
        o.stamp = instance.stamp
        o._messages.update(instance._messages)
        return o
        
        
    def add_message(self, message, topic):
        """
        Add the message to this observation under the given name.
        """
        message_proxy = MessageStoreProxy(collection="ws_observations")
        msg_id = message_proxy.insert(message)
        self._messages[topic]  = MessageStoreObject(
            database=message_proxy.database,
            collection=message_proxy.collection,
            obj_id=msg_id,
            typ=message._type)
        
    def get_message(self, topic):
        if not self._messages.has_key(topic):
            raise StateException("NO_OBSERVATION")
        return self._messages[topic].retrieve()