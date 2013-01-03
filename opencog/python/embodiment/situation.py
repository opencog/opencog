__author__ = 'keyvan'

from opencog.atomspace import AtomSpace, types

_counter_by_type = {}
def generate_unique_name(object_type):

    if object_type in _counter_by_type:
        _counter_by_type[object_type] += 1
    else:
        _counter_by_type[object_type] = 0

    return object_type + '_' +  str(_counter_by_type[object_type])


class CombinationDescriptor(object):

    def __init__(self, weight, object_type, containment=None):

        self.weight = weight
        self.object_type = object_type
        self.containment = containment

    def append_to_atomspace(self, atomspace):
        entity = atomspace.add_node(types.ConceptNode, generate_unique_name(self.object_type))
#        if self.containment is not None:
#            for string in self.containment:
#                object =
#                atomspace.add_link()


    def __repr__(self):
        pass
