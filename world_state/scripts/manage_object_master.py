#!/usr/bin/python
import rospy
from world_state.state import World, Object
from world_state.identification import ObjectIdentification
from world_state.observation import MessageStoreObject

import world_state.objectmaster as objectmaster

from mongodb_store.message_store import MessageStoreProxy


if __name__ == '__main__':
    ''' Main Program '''
    rospy.init_node("objectmaster_check")
    
    om =  objectmaster.ObjectMaster()

            
    quit = False
    while not quit:
        cmd = raw_input("> ")
        if cmd == "help":
            print "Help. No Help."
        elif cmd == "quit":
            quit = True
        elif cmd == "list":
            rospy.loginfo("Object categories and instances known to ObjectMaster:")
            for cat in om.get_categories():
                rospy.loginfo("->%s"%cat)
                for inst in om.get_instances(objectmaster.ObjectCategory(cat)):
                    rospy.loginfo("--%s-->%s"%(len(cat)*"-", inst))
        elif cmd.startswith("add class"):
            cls_name = cmd[10:]
            rospy.loginfo("Adding class '%s'"%cls_name)
            c = objectmaster.ObjectCategory()
            c .name = cls_name
            om.add_category(c)
        elif cmd.startswith("add instance"):
            p = cmd.split()
            rospy.loginfo("Adding instance of class '%s', called '%s'"%(p[2], p[3]))
            tt = objectmaster.ObjectInstance()
            tt.name = p[3]
            tt.category = p[2]
            
            om.add_instance(tt)
        else:
            print "Invalid command"
