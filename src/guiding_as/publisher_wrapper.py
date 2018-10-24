#! /usr/bin/env python

import rospy
import genpy
from rospy.core import *
from rospy.exceptions import ROSSerializationException
from rospy.msg import args_kwds_to_message
_logger = logging.getLogger('rospy.topics')

# from visualization_msgs.msg import *
# from std_msgs.msg import ColorRGBA


class PublisherWrapper(rospy.Publisher):

    def __init__(self, name, data_class, subscriber_listener=None, tcp_nodelay=False, latch=False, headers=None,
                 queue_size=None):
        rospy.Publisher.__init__(self, name, data_class, subscriber_listener, tcp_nodelay, latch, headers, queue_size)

    def publish(self, *args, **kwds):
        """
        Publish message data object to this topic.
        Publish can either be called with the message instance to
        publish or with the constructor args for a new Message
        instance, i.e.::
          pub.publish(message_instance)
          pub.publish(message_field_1, message_field_2...)
          pub.publish(message_field_1='foo', message_field_2='bar')

        @param args : L{Message} instance, message arguments, or no args if keyword arguments are used
        @param kwds : Message keyword arguments. If kwds are used, args must be unset
        @raise ROSException: If rospy node has not been initialized
        @raise ROSSerializationException: If unable to serialize
        message. This is usually a type error with one of the fields.
        """
        if self.impl is None:
            raise ROSException("publish() to an unregistered() handle")
        if not is_initialized():
            raise ROSException("ROS node has not been initialized yet. Please call init_node() first")
        data = args_kwds_to_message(self.data_class, args, kwds)
        rospy.logdebug("Published message on :" + self.resolved_name)
        rospy.logdebug(data)
        try:
            self.impl.acquire()
            self.impl.publish(data)
        except genpy.SerializationError as e:
            # can't go to rospy.logerr(), b/c this could potentially recurse
            _logger.error(traceback.format_exc())
            raise ROSSerializationException(str(e))
        finally:
            self.impl.release()

#
# def test_publisher():
#     publi = PublisherWrapper("/visualization_marker", Marker, queue_size=10)
#     rospy.sleep(2)
#     marker = Marker()
#     marker.ns = "supervisor"
#     marker.type = Marker.SPHERE
#     marker.action = Marker.ADD
#     marker.id = 200
#     marker.pose.position.x = 2.25
#     marker.pose.position.y = 16.4
#     marker.pose.position.z = 1
#     marker.scale.x = 0.3
#     marker.scale.y = 0.3
#     marker.scale.z = 0.3
#     marker.color = ColorRGBA(0, 0.6, 0.9, 1.0)
#     marker.header.frame_id = "map"
#     publi.publish(marker)
#     # rospy.logwarn(result)
#
#
# if __name__ == "__main__":
#     rospy.init_node("test", log_level=rospy.DEBUG)
#     test_publisher()
