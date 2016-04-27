from mongo import MongoConnection, MongoTransformable
from exceptions import ObjectMasterException

class ObjectCategory(MongoTransformable):
    def __init__(self, name=None):
        super(ObjectCategory, self).__init__()
        self._id = "None"
        if name is not None:
            self.name = name
        #self._id = "ObjectCategory_None"
        #self._name = None
        
    #def __update_id(self):
        #self._id = "ObjectCategory_{}".format(self.name)
        
    #@property
    #def category_id(self):
        #return self._id
    
    @property
    def name(self):
        return self._id
    
    @name.setter
    def name(self, value):
        #TODO: check duplication..
        self._id = value
        #self.__update_id()
        


class ObjectInstance(MongoTransformable):
    """
    Provides the CAD model or the like of known objects
    """
    def __init__(self):
        super(ObjectInstance, self).__init__()
        self._id = "ObjectInstance_None"
        self._category = None
        self._name = None
       
    def __update_id(self):
        self._id = "ObjectInstance_{}_{}".format(self._category, self._name)
    
    @property
    def name(self):
        return self._name
    
    @name.setter
    def name(self, value):
        self._name = value
        self.__update_id()
    
    @property
    def category(self):
        return self._category
    
    @category.setter
    def category(self, value):
        self._category = value
        self.__update_id()

        
        
class ObjectMaster(object):
    
    def __init__(self, mongo=None):
        if mongo is None:
            self._mongo = MongoConnection()
        else:
            self._mongo = mongo

    def add_category(self, category):
        self._mongo.database.ObjectMaster.save(ObjectCategory._mongo_encode(category))

    def remove_category(self, category):
        if category.name not in self.get_categories():
            raise ObjectMasterException("UNKNOWN_CAT")
        new_id = self._mongo.database.Wasteland.save(ObjectCategory._mongo_encode(category))
        self._mongo.database.ObjectMaster.remove(ObjectCategory._mongo_encode(category))
        
    def add_instance(self, instance):
        # check that category exists
        if instance.category not in self.get_categories():
            raise ObjectMasterException("UNKNOWN_CAT")
        self._mongo.database.ObjectMaster.save(ObjectInstance._mongo_encode(instance))
    
    def remove_instance(self, instance):
        new_id = self._mongo.database.Wasteland.save(ObjectInstance._mongo_encode(instance))
        self._mongo.database.ObjectMaster.remove(ObjectInstance._mongo_encode(instance))
        
    def get_categories(self):
        classes = self._mongo.database.ObjectMaster.find(
            #{"__pyobject_class_type": ObjectCategory().__class__.__name__},
            {"__pyobject_class_type": ObjectCategory.get_pyoboject_class_string()}, 
            {"_id": 1,})
        return [c['_id'] for c in classes]
    
    def check_category_exists(self, category_name):
        return category_name in self.get_categories()
    
    def get_instances(self, category):
        instances = self._mongo.database.ObjectMaster.find(
            {"__pyobject_class_type": ObjectInstance.get_pyoboject_class_string(),
             "_category": category.name,},
            {"_name": 1,})
        return [i["_name"] for i in instances]
    
    