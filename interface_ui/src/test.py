#! /usr/bin/env python
"""Test ROS node for the UI node"""

from __future__ import print_function

import json

import rospy

from interface_msgs.srv import Complete, SendSignal, ServiceList  # pylint: disable=import-error
from interface_msgs.msg import Service as ServiceMessage  # pylint: disable=import-error
from architecture_msgs.srv import BehaviorStatus  # pylint: disable=import-error


def srv_callback(request):
    """dummy func to do callback"""
    print(request)
    return True

def beh_1_callback(request):  # pylint: disable=unused-argument
    """dummy func to do behavior_status callback"""
    return 'beh_1', 'beh_1_desc', 3, 'beh_1_status'

def beh_2_callback(request):  # pylint: disable=unused-argument
    """dummy func to do behavior_status callback"""
    return 'beh_2', 'beh_2_desc', 3, 'beh_2_status'

def service_list_callback(request):  # pylint: disable=unused-argument
    """dummy func to do service list callback"""
    variables = {'variables': {'var_1': {'value': 'default', 'type': 'string'}}}
    srv_1 = ServiceMessage("srv_1", "Complete", 7, json.dumps(variables))
    srv_2 = ServiceMessage("/srv_2", "SendASignal", 3, json.dumps(variables))

    return {'behavior_name':'', 'services': [srv_1, srv_2]}


rospy.init_node('test_node')
rospy.Service('beh_1/status', BehaviorStatus, beh_1_callback)
rospy.Service('beh_2/status', BehaviorStatus, beh_2_callback)
rospy.Service('srv_list', ServiceList, service_list_callback)
rospy.Service('srv_1', Complete, srv_callback)
rospy.Service('srv_2', SendSignal, srv_callback)

rospy.spin()
