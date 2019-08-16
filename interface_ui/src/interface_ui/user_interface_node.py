#! /usr/bin/env python
"""Module conatians the ROS node for the User Interface"""

from __future__ import print_function

import rospy
import rosservice

from user_interface import UserInterface, BehaviorInterface, ServiceInterface
from architecture_msgs.srv import BehaviorStatus  # pylint: disable=import-error
from interface_msgs.srv import ServiceList, ServiceListResponse  # pylint: disable=import-error


class UserInterfaceNode(object):
    """The ROS node class for the user interface"""
    def __init__(self):
        rospy.init_node('user_interface')

    @staticmethod
    def call(service_name, **service_args):
        """Calls the service_name srv, with service_args"""
        if service_name[0] != '/':
            service_name = '/' + service_name
        srv_class = rosservice.get_service_class_by_name(service_name)
        func = rospy.ServiceProxy(service_name, srv_class)
        return func(**service_args)

    def get_behaviors(self):
        """Gets the list of behaivors, constructs them, and gets their srvs"""
        behaviors = []

        beh_status_srvs = rosservice.rosservice_find(BehaviorStatus._type)  # pylint: disable=protected-access
        for srv_name in beh_status_srvs:
            func = rospy.ServiceProxy(srv_name, BehaviorStatus)
            behavior = BehaviorInterface(func())
            beh_node = rosservice.get_service_node(srv_name)
            behavior.set_services(self.get_services(beh_node))
            behaviors.append(behavior)

        return behaviors

    def get_services(self, behavior_node):  # pylint: disable=no-self-use
        """Gets a list of services of a behavior and returns them"""
        node_srvs = rosservice.get_service_list(node=behavior_node)
        srv_list_name = None
        for srv in node_srvs:
            srv_type = rosservice.get_service_type(srv)
            if srv_type == ServiceList._type:  # pylint: disable=protected-access
                srv_list_name = srv
                break
        if srv_list_name:
            func = rospy.ServiceProxy(srv_list_name, ServiceList)
            return func()
        return ServiceListResponse(behavior_name="", services=[])

    def run(self):
        """The main method of the node"""
        user_interface = UserInterface(self)
        user_interface.set_behaviors(self.get_behaviors())
        user_interface.run()


if __name__ == "__main__":
    UserInterfaceNode().run()
