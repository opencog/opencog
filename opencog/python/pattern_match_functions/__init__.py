from opencog.atomspace import Handle

__author__ = 'keyvan'
import rules


def execute_user_defined_function(func, handle_uuid):
    handle = Handle(handle_uuid)
    args_list_link = ATOMSPACE[handle]
    
