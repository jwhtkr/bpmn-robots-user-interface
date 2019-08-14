#! /usr/bin/env python
"""Module that holds the underlying class(es) for the user interface"""

import json
import datetime

from interface_msgs.msg import Service as ServiceMessage  # pylint: disable=import-error
from interface_msgs.srv import Complete, GetMessage, GetVariable  # pylint: disable=import-error
from interface_msgs.srv import SendMessage, SendSignal, SetVariable, ThrowError  # pylint: disable=import-error


JSON_TYPE_DICT = {'boolean': bool,
                  'string':  str,
                  'integer': int,
                  'long':    int,
                  'float':   float,
                  'double':  float,
                  'null':    None,
                  'bytes':   bytes,
                  'date':    datetime.datetime}

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
        value = raw_input(prompt)

    return value

def variable_from_template(template):
    """Gives the user a choice of templates to fill, or create a new one"""
    update = None
    while update is None:
        uinput = raw_input("Create new variable/update existing (n/u): ")
        if uinput == 'n':
            update = False
        elif uinput == 'u':
            update = True

    if update:
        var_name = variable_name_from_template(template)
        var = template[var_name]
        var_type = var[u'type']
        prompt = "Enter a new value for the variable (Old value: {curr_val}): "
        var_value = raw_input(prompt.format(curr_val=var[u'value']))
    else:
        var_name = raw_input("Enter variable name: ")
        var_type = None
        var_value = raw_input("Enter variable value: ")

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
            var_type = raw_input("Enter the type for the variable: ")
    
    


def fill_template(template):
    """Allows the user to fill values, or use defaults, based on template"""
    prompt = "Enter a value for {name}({type}) or "\
        + "press enter for the default value({curr_val}): "

    template = template.get(u'variables', d=template)
    try:
        for var_name in template:
            item = template[var_name]
            value = raw_input(prompt.format(name=var_name,
                                            type=item[u'type'],
                                            curr_val=item[u'value']))

            if value != '':
                template[u'value'] = value

        return json.dumps(template)
    except ValueError:
        return ""


class Service(object):
    """Represents a service being offered by a behavior"""
    srv_type_dict = {ServiceMessage.COMPLETE:       Complete,
                     ServiceMessage.GET_MESSAGE:    GetMessage,
                     ServiceMessage.GET_VARIABLE:   GetVariable,
                     ServiceMessage.SEND_MESSAGE:   SendMessage,
                     ServiceMessage.SENG_SIGNAL:    SendSignal,
                     ServiceMessage.SET_VARIABLE:   SetVariable,
                     ServiceMessage.THROW_ERROR:    ThrowError}

    def __init__(self, service):
        self.name = service.name
        self.type = Service.srv_type_dict[service.type]
        self.template = service.json_template

    def user_input(self):
        """Create a JSON string filled with user values based on the template"""
        try:
            template = json.loads(self.template)
        except ValueError:
            return ""
        if not isinstance(template, dict):
            raise TypeError("A non-object template was used in user_input()")

        # if isinstance(self.type, GetMessage):
        #     return message_name_from_template(template)

        if isinstance(self.type, GetVariable):
            return variable_name_from_template(template)

        if isinstance(self.type, SetVariable):
            return variable_from_template(template)

        return fill_template(template)


class UserInterface(object):
    """Main class for the user interface"""
    def __init__(self, context):
        self.context = context
        self.options = []  # list of active behaviors, each with a list of srvs
