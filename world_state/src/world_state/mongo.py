import json
import pymongo
import pymongo.son_manipulator
import numpy as np
import importlib
import rospy
import copy

import rospy

def load_class(full_class_string):
    """
    dynamically load a class from a string
    shamelessly ripped from: http://thomassileo.com/blog/2012/12/21/dynamically-load-python-modules-or-classes/
    """
    # todo: cache classes (if this is an overhead)
    class_data = full_class_string.split(".")
    module_path = ".".join(class_data[:-1])
    class_str = class_data[-1]
    module = importlib.import_module(module_path)
    # Finally, we retrieve the Class
    return getattr(module, class_str)

class MongoConnection(object):
    def __init__(self, database_name="world_state", server=None, port=None):
        if server is None:
            server = rospy.get_param("mongodb_host")
        if port is None:
            port = rospy.get_param("mongodb_port")
            
        rospy.loginfo("Connecting to mongo: %s,%s"%(server, port))
        self.client = pymongo.MongoClient(server, port)
        self.database = self.client[database_name]
        
        self.database.add_son_manipulator(NumpyTransformer())
        self.database.add_son_manipulator(MongoTransformer())
        
class MongoTransformer(pymongo.son_manipulator.SONManipulator):
    def __init__(self):
        pass
    
    def transform_incoming(self, son, collection):
        if isinstance(son, list):
            return self.transform_incoming_list(son, collection)
        elif isinstance(son, dict):
            for (key, value) in son.items():
                #if isinstance(value, dict) or isinstance(value, list): 
                son[key] = self.transform_incoming(value, collection)
        elif hasattr(son, "_mongo_encode"):
            son = self.transform_incoming(son._mongo_encode(son), collection)
            
            
        return son
    
    def transform_incoming_list(self, lst, collection):
        new_lst = map(lambda x: self.transform_incoming(x, collection),
                  lst)
        return new_lst
            
    def transform_outgoing(self, son, collection):
        if isinstance(son, list):
            #print "Trany list: ", son
            return self.transform_outgoing_list(son, collection)
        elif isinstance(son, dict):
            for (key, value) in son.items():
                son[key] = self.transform_outgoing(value, collection)
    
            if "__pyobject_class_type" in son:
                #print "Decoding a ", son["__pyobject_class_type"]
                cls = load_class(son["__pyobject_class_type"])
                return cls._mongo_decode(son)
            else:
                return son
        return son
    
    def transform_outgoing_list(self, lst, collection):
        new_lst = map(lambda x: self.transform_outgoing(x, collection),
                  lst)
        #print "->", new_lst
        return new_lst
    
    
class Keyed(object):
    def __init__(self):
        self.key = None
        self.__create_unique_name()
        
    def __rebase(self, val, syms):
        base = len (syms)
        n = ""
        while val > 0:
            rem = val % base
            val -= rem
            val /= base
            n = syms[rem] + n
        return n
    
    def __create_unique_name(self):
        t = rospy.Time.now()
        self.key = self.__rebase(int(t.secs * 1000000 + t.nsecs),
                      'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ'
                      '1234567890')

    def __hash__(self):
        return hash(self.key)
    
    def __eq__(self, other):
        return self.key == other.key

    
class MongoTransformable(object):
    @classmethod
    def _mongo_encode(cls, class_object):
        #doc = {}
        #doc.update(class_object.__dict__)
        doc = copy.deepcopy (class_object.__dict__)
        doc["__pyobject_class_type"] = (class_object.__module__+
                                        "." + class_object.__class__.__name__)
        return doc
        
    
    @classmethod
    def _mongo_decode(cls, mongo_document):
        if isinstance(mongo_document, dict):
            c = cls()
            c.__dict__.update(mongo_document)
            return c
        return mongo_document
    
    @classmethod
    def get_pyoboject_class_string(cls):
        return cls.__module__ + "." + cls.__name__
    
class MongoDocument(Keyed, MongoTransformable):
    def __init__(self):
        self.__connected = False
        self.__mongo = None
        super(MongoDocument, self).__init__()
        
    
    def _connect(self, mongo):
        #TODO: think
        object.__setattr__(self, "_MongoDocument__mongo", mongo)
        object.__setattr__(self, "_MongoDocument__connected", True)
    
    def force_refresh_from_datacentre(self):
        pass
    
    def __setattr__(self, atr, v):
        # Set the mongo document...
        object.__setattr__(self, atr, v)
        if self.__connected:
            self.__mongo.database.Objects.save(self._mongo_encode(self))
    
class NumpyTransformer(pymongo.son_manipulator.SONManipulator):
    def transform_incoming(self, son, collection):
        for (key, value) in son.items():
            if isinstance(value, np.ndarray):
                son[key] = self._mongo_encode(value)
                #son[key]["__object_class_type"] = str(self.__class__)
            elif isinstance(value, dict): # Make sure we recurse into sub-docs
                son[key] = self.transform_incoming(value, collection)
            elif isinstance(value, list):
                lst = map(lambda x: self._mongo_encode(x)
                          if isinstance(x,  np.ndarray)
                         else x,
                         value)
                son[key] = lst
                #[self._mongo_encode(v) for v in value if isinstance(value, self.__class__)
                #else v]

                
        #print son
        return son
            
    def transform_outgoing(self, son, collection):
        for (key, value) in son.items():
            if isinstance(value, dict):
                if ("__pyobject_class_type" in value and
                    value["__pyobject_class_type"] == "numpy_array"):
                    son[key] = self._mongo_decode(value)
                else: # Again, make sure to recurse into sub-docs
                    son[key] = self.transform_outgoing(value, collection)
        return son
    
    @staticmethod
    def _mongo_encode(numpy_array):
        doc = {}
        doc["array"] = numpy_array.tolist()
        doc["__pyobject_class_type"] = "numpy_array"
        return doc
    
    @staticmethod
    def _mongo_decode(mongo_document):
        return np.array(mongo_document["array"])
    
