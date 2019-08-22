#! /usr/bin/env python
"""Module that holds the underlying classes for the user interface"""

from __future__ import print_function

from os import system

import json
# import datetime

from interface_msgs.msg import Service as ServiceMessage  # pylint: disable=import-error
from interface_msgs.srv import Complete, GetVariable  # pylint: disable=import-error
from interface_msgs.srv import SendMessage, SendSignal, SetVariable, ThrowError  # pylint: disable=import-error

# pylint: disable=bad-continuation
JSON_TYPE_DICT = {'boolean': bool,
                  'string':  str,
                  'integer': int,
                #   'long':    int,
                #   'float':   float,
                  'double':  float
                #   'null':    None,
                #   'bytes':   bytes,
                #   'date':    datetime.datetime
                 }
# pylint: enable=bad-continuation

# def message_name_from_template(template):
#     """Gives the user choices from the template of message names to get"""
#     pass

def variable_name_from_template(template):
    """Gives the user choices from the template of variable names to get"""
    prompt = []

    template = template.get(u'variables', template)
    for var_name in template:
        prompt.append(var_name)

    prompt = ",\n".join(prompt)
    prompt += "\nEnter one of the above variable names: "

    value = ""
    while value not in template:
        value = UserInterface.prompt(prompt)

    return value

def variable_from_template(template):
    """Gives the user a choice of templates to fill, or create a new one"""
    update = None
    while update is None:
        val = UserInterface.prompt("Create new variable/update existing(n/u): ")
        if val == 'n':
            update = False
        elif val == 'u':
            update = True

    if update:
        var_name = variable_name_from_template(template)
        var = template[var_name]
        var_type = var[u'type']
        prompt = "Enter a new value for the variable (Old value: {curr_val}): "
        var_value = UserInterface.prompt(prompt.format(curr_val=var[u'value']))
    else:
        var_name = UserInterface.prompt("Enter variable name: ")
        var_type = None
        var_value = UserInterface.prompt("Enter variable value: ")

    wrong_type = True
    while wrong_type:
        try:
            var_py_type = JSON_TYPE_DICT[var_type]
            if var_py_type is None:
                wrong_type = False if var_value is None else True
            else:
                var_value = var_py_type(var_value)
                wrong_type = False
        except (ValueError, KeyError):
            var_type = UserInterface.prompt("Enter the type for the variable: ")

    return {'name': var_name, 'value': var_value, 'type': var_type}


def fill_template(template):
    """Allows the user to fill values, or use defaults, based on template"""
    prompt = "Enter a value for {name}({type}) or "\
        + "press enter for the default value({curr_val}): "

    template = template.get(u'variables', template)
    try:
        for var_name in template:
            item = template[var_name]
            value = UserInterface.prompt(prompt.format(name=var_name,
                                                       type=item[u'type'],
                                                       curr_val=item[u'value']))

            if value != '':
                item[u'value'] = unicode(value, 'utf-8')

        return str(json.dumps({u'variables': template}))
    except ValueError:
        return ""


class ServiceInterface(object):
    """Represents a service being offered by a behavior"""
    srv_type_dict = {ServiceMessage.COMPLETE:       Complete,
                     #  ServiceMessage.GET_MESSAGE:    GetMessage,
                     ServiceMessage.GET_VARIABLE:   GetVariable,
                     ServiceMessage.SEND_MESSAGE:   SendMessage,
                     ServiceMessage.SEND_SIGNAL:    SendSignal,
                     ServiceMessage.SET_VARIABLE:   SetVariable,
                     ServiceMessage.THROW_ERROR:    ThrowError}

    def __init__(self, service):
        self.service_name = service.service_name
        self.name = service.name
        self.type = ServiceInterface.srv_type_dict[service.type]
        self.template = service.json_template

    def build_request(self):
        """Builds the data for the service request, including any user input"""
        UserInterface.print_next_screen_prefix()
        UserInterface.clear_screen(UserInterface.CLEAR_SCREEN)
        print("Building {} request.".format(self.service_name))
        print("Press Ctrl-C any time during request process to abort")
        UserInterface.print_screen_infix()

        user_input = self.input_from_template()

        if isinstance(user_input, dict):
            request = user_input
        elif self.type is GetVariable:
            request = {'name': user_input}
        else:
            request = {"variables": user_input}

        if self.type is SendMessage or self.type is SendSignal:
            request['name'] = self.name

        if self.type is ThrowError:
            request['message'] = UserInterface.prompt('Enter Error Message: ')

        return request

    def input_from_template(self):
        """Create a JSON string filled with user values based on the template"""
        try:
            template = json.loads(self.template)
        except ValueError:
            return ""
        if not isinstance(template, dict):
            raise TypeError("A non-object template was used in user_input()")

        # if self.type is GetMessage:
        #     return message_name_from_template(template)

        if self.type is GetVariable:
            return variable_name_from_template(template)

        if self.type is SetVariable:
            return variable_from_template(template)

        return fill_template(template)


class BehaviorInterface(object):
    """Represents a Behavior that is active"""
    def __init__(self, behavior):
        self.name = behavior.name
        self.description = behavior.description
        self.manager_status = behavior.managerStatus
        self.behavior_status = behavior.behaviorStatus
        self.services = []

    def __repr__(self):
        return "BehaviorInterface({}: {})".format(self.name, self.services)

    def __str__(self):
        return "{}:\n{}".format(self.name, self.description)

    def set_services(self, service_list_msg):
        """Sets self.services to the list of services passed in as ROS msgs"""
        if service_list_msg.behavior_name != self.name \
            and service_list_msg.behavior_name != '':
            raise ValueError("The behavior names didn't match")

        self.services = []
        services = service_list_msg.services
        for service in services:
            self.services.append(ServiceInterface(service))


class UserInterface(object):
    """Main class for the user interface"""
    CLEAR_SCREEN = False
    def __init__(self, context):
        self.context = context
        self.behaviors = []  # list of BehaviorInterfaces
        self.clear_screen(True)

    def set_behaviors(self, behavior_list):
        """Sets the behaviors to the given list"""
        self.behaviors = behavior_list

    @staticmethod
    def prompt(prompt="", clear_screen=False):
        """Prompts the user for input and the option to clear the screen"""
        UserInterface.clear_screen(clear_screen)
        return raw_input(prompt)

    @staticmethod
    def clear_screen(clear_screen):
        """Clears the screen if clear_screen is true"""
        if clear_screen:
            system('clear')

    @staticmethod
    def print_next_screen_prefix():
        """Prints symbols to help visually separate screens if not clearing"""
        print()
        print("*****************************")

    @staticmethod
    def print_screen_infix():
        """Prints symbols to visually separate header info from other info"""
        print("*****************************")

    def show_behaviors(self, clear_screen=CLEAR_SCREEN):
        """Displays the behaviors that a user can interact with"""
        beh_str = "{num}) {opt_name}"

        self.print_next_screen_prefix()
        self.clear_screen(clear_screen)

        for i, behavior in enumerate(self.behaviors):
            print(beh_str.format(num=i, opt_name=behavior.name))

        print(beh_str.format(num=len(self.behaviors),
                             opt_name="Refresh Behavior List"))
        print(beh_str.format(num=len(self.behaviors) + 1, opt_name="Exit UI"))

    def show_services(self, behavior, clear_screen=CLEAR_SCREEN):
        """Displays the services that a behavior has available"""
        self.print_next_screen_prefix()
        self.clear_screen(clear_screen)
        print(str(behavior))
        self.print_screen_infix()

        for i, service in enumerate(behavior.services):
            print("{num}) {srv_name}".format(num=i,
                                             srv_name=(service.name
                                                       + ' '
                                                       + service.service_name)))

        print("{}) Behavior List (prev. screen)".format(len(behavior.services)))

    def get_behavior_selection(self):
        """Retrieve a behavior based on user input"""
        self.show_behaviors()
        beh_idx = int(self.prompt("Enter Selection Number: "))
        try:
            return self.behaviors[beh_idx], False
        except IndexError:
            if beh_idx == len(self.behaviors):
                self.set_behaviors(self.context.get_behaviors())
                return None, False
        return None, True

    def get_service_selection(self, behavior):
        """Retreive a service based on user input"""
        # TODO: Refresh service list that also goes back if behavior is gone
        self.show_services(behavior)
        srv_idx = int(self.prompt("Enter Selection Number: "))
        try:
            return behavior.services[srv_idx], False
        except IndexError:
            pass
        return None, True

    def run(self):
        """The main logic of the User Interface"""
        done = False
        while not done:
            behavior, done = self.get_behavior_selection()

            if behavior:
                back = False
                while not back:
                    service, back = self.get_service_selection(behavior)

                    if service:
                        try:
                            request = service.build_request()
                            resp = self.context.call(service.service_name,
                                                     **request)
                            print("Service Success: {}".format(resp.success))
                        except KeyboardInterrupt:
                            pass
                        except AttributeError:
                            print("The UI was called from a context that has no"
                                  + " call(srv_name, **request_args) function")

if __name__ == "__main__":
    from architecture_msgs.srv import BehaviorStatusResponse  # pylint: disable=import-error
    # pylint: disable=invalid-name
    beh_1 = BehaviorStatusResponse("beh_1", "beh_1_descr", 3, "beh_1_status")
    beh_2 = BehaviorStatusResponse("beh_2", "beh_2_descr", 3, "beh_2_status")

    beh_1 = BehaviorInterface(beh_1)
    beh_2 = BehaviorInterface(beh_2)

    variables = {'variables': {'var_1': {'value': 'default', 'type': 'string'}}}
    srv_1 = ServiceMessage("srv_1", "Complete", 7, json.dumps(variables))
    srv_2 = ServiceMessage("srv_2", "SendASignal", 3, json.dumps(variables))

    beh_1.set_services([srv_1, srv_2])
    beh_2.set_services([srv_1, srv_2])

    ui = UserInterface(None)
    ui.set_behaviors([beh_1, beh_2])
    ui.run()
    # pylint: enable=invalid-name
