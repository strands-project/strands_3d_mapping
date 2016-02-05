import rospy 
from std_msgs.msg import String

def get_ros_service(srv_name, srv_type):
    try:
        rospy.loginfo("Initialising LeanObjects SM: "
                      "Getting '%s' service.."%srv_name)
        rospy.wait_for_service(srv_name, 5)
        rospy.loginfo("-> ok.")
    except:
        rospy.logerr( "Service '%s' not available. Aborting"%srv_name)
        raise Exception("LeranObject SM can't initialise; missing service.")
    return rospy.ServiceProxy(srv_name, srv_type)


class DebugModeServices():
    def __init__(self, preempt_requested_fn):
        self._status_publisher = rospy.Publisher("/object_learning/debug_status", String, queue_size=1, latch=True)
        self.preempt_requested = preempt_requested_fn
        
    def _wait_for_debug_proceed_message(self):
        message = String("PREEMPT")
        while not self.preempt_requested():
            try:
                message = rospy.wait_for_message("/object_learning/debug_proceed", String, 5)
                rospy.loginfo("Got debug proceed message:"+str(message))
                break
            except rospy.ROSException, e:
                if str(e).startswith("timeout exceeded"):
                    rospy.logwarn("Object learning in debug mode, waiting for proceed message...")
                    continue
                else:
                    break
        return message.data

    def get_proceed(self, valids):
        proceed = self._wait_for_debug_proceed_message()
        
        while proceed not in valids+("PREEMPT",):
            rospy.logwarn("Object learning debug mode got unexpected proceed string.")
            proceed = self._wait_for_debug_proceed_message()
        return proceed

    def set_status(self, status):
        self._status_publisher.publish(String(status))

