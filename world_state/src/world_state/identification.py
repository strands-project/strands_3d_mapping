import rospy
from mongo import MongoTransformable

class ObjectIdentification(MongoTransformable):
    def __init__(self,  class_conf={}, instance_conf={}):
        # TODO: check that objects specified actually exist in object master.
        # TODO: check the conf sum == 1.0
        self.class_distribution = class_conf
        self.instance_distribution = instance_conf
        self._time_stamp = rospy.Time.now().to_time()
        
        if len(class_conf.keys()) == 0:
            self.class_type = ("unknown", 1.0)
        else:
            self.class_type = max(self.class_distribution.items(),
                                   key=lambda f: f[1])

        if len(instance_conf.keys()) == 0:
            self.instance_type = ("unknown", 1.0)
        else:
            self.class_type = max(self.instance_distribution.items(),
                                   key=lambda f: f[1])
    
    def set_time_stamp(self, time):
        self._time_stamp = time
        