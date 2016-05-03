#!/usr/bin/env python

import sys
import unittest
import world_state.objectmaster as om
import rospy

class TestObjectMaster(unittest.TestCase):
    def setUp(self):
        try:
            rospy.init_node("testint_om", anonymous=True)
        except rospy.ROSException, e:
            pass # Already a node? 

    def test_object_master_categories(self):
        o = om.ObjectMaster()
            
        categories_to_add =  ["Table", "Book", "Bottle"  ]

        for t in categories_to_add:
            tt = om.ObjectCategory()
            tt.name = t
            o.add_category(tt)
            
        added_categories = o.get_categories()
        
        print added_categories
        for c in categories_to_add:
            self.assertIn(c, added_categories)
 
        for t in categories_to_add:
            tt = om.ObjectCategory()
            tt.name = t
            o.remove_category(tt)
            
        added_categories = o.get_categories()
        
        print added_categories
        for c in categories_to_add:
            self.assertNotIn(c, added_categories)
            
    def test_object_master_instances(self):
        o = om.ObjectMaster()
            
        to_add =  ["afsd", "wr", "xcvsjd"  ]

        cat = om.ObjectCategory()
        cat.name = "test__cat"
        o.add_category(cat)
        
        for t in to_add:
            tt = om.ObjectInstance()
            tt.name = t
            tt.category = cat.name
            
            o.add_instance(tt)
            
        added = o.get_instances(cat)
        
        print added
        for i in to_add:
            self.assertIn(i, added)
 
        for t in to_add:
            tt = om.ObjectInstance()
            tt.name = t
            tt.category = cat.name
            o.remove_instance(tt)
            
        added = o.get_instances(cat)
        o.remove_category(cat)
        print added
        for c in to_add:
            self.assertNotIn(c, added)
            
            
if __name__ == '__main__':
    import rosunit
    PKG='world_state'
    rosunit.unitrun(PKG, 'test_objects', TestObjects)
    
    
    