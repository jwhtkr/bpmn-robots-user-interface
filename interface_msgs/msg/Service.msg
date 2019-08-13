##
# @File: Service.msg
# @Date: 12 August 2019
# @Author: Justin Whitaker
#
# @Brief
# This is a message that represents a service that is offered by a behavior to the UI
#
# @name
# The fully qualified name of the service (including full namespacing)
#
# @type
# The type of service, as one of the values of the enum defined in the message
#
# @json_template
# A JSON template object for what variables need to be provided by the user. Empty string denotes no variables needed
##

## The possible types of services that can be used ##
# Each corresponds to the similarly named message in the interface_msgs package
# NULL corresponds to the "generic" service
uint8 COMPLETE = 7
uint8 GET_MESSAGE = 6
uint8 GET_VARIABLE = 5
uint8 SEND_MESSAGE = 4
uint8 SEND_SIGNAL = 3
uint8 SET_VARIABLE = 2
uint8 THROW_ERROR = 1
uint8 NULL = 0

string name
uint8 type
string json_template
