#!/usr/bin/env python

import sys
import unittest
from world_state.state import World, Object
from world_state.identification import ObjectIdentification
import rospy


class TestWorld(unittest.TestCase):
    def setUp(self):
        try:
            rospy.init_node("testing_world", anonymous=True)
        except rospy.ROSException, e:
            pass # Already a node? 

    def test_world_add_object(self):
        w = World("world_state")
        obj = w.create_object()

        name = obj.name
        obj = w.get_object(obj.name)
        obj.add_identification("TableDetection",
                               ObjectIdentification({'Table': 0.2,
                                                    'Football': 0.3}))
        
        
        obj = w.get_object(obj.name)

        obj.alpha = 45

        self.assertEqual(name, obj.name)

        self.assertEqual(obj.get_identification("TableDetection").class_type,
                         ["Football", 0.3])
        
        w.remove_object(obj)
         
        with self.assertRaises(Exception) as ex:
            w.remove_object(obj.name)
                
        the_exception = ex.exception
        self.assertEqual(str(the_exception),
                         "get_object failed to find object '%s' in database."%obj.name)
            
if __name__ == '__main__':
    import rosunit
    PKG='world_state'
    rosunit.unitrun(PKG, 'test_objects', TestObjects)
    