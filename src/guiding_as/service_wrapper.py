#! /usr/bin/env python

import rospy
# from perspectives_msgs.srv import *


class ServiceWrapper(rospy.ServiceProxy):

    def __init__(self, name, service_class):

        rospy.ServiceProxy.__init__(self, name, service_class)

    def __call__(self, *args, **kwds):
        """
        Callable-style version of the service API. This accepts either a request message instance,
        or you can call directly with arguments to create a new request instance. e.g.::

          add_two_ints(AddTwoIntsRequest(1, 2))
          add_two_ints(1, 2)
          add_two_ints(a=1, b=2)

        @param args: arguments to remote service
        @param kwds: message keyword arguments
        @raise ROSSerializationException: If unable to serialize
        message. This is usually a type error with one of the fields.
        """
        saved_args = locals()
        rospy.logdebug(self.resolved_name + " called with the following arguments : " + str(saved_args["args"]))
        return self.call(*args, **kwds)


# def test_service():
#     rospy.wait_for_service('/uwds_ros_bridge/has_mesh')
#     has_mesh_srv = ServiceWrapper('/uwds_ros_bridge/has_mesh', HasMesh)
#     result = has_mesh_srv("szads", "dezdez")
#     # rospy.logwarn(result)
#
#
# if __name__ == "__main__":
#     rospy.init_node("test", log_level=rospy.DEBUG)
#     test_service()

