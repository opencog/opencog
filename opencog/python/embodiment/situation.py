__author__ = 'keyvan'

from opencog.atomspace import AtomSpace, types

_counter_by_type = {}
def generate_unique_name(type):

    if type in _counter_by_type:
        _counter_by_type[type] += 1
    else:
        _counter_by_type[type] = 0

    return type + '_' +  str(_counter_by_type[type])


class CombinationDescriptor(object):

    def __init__(self, weight, type, containment=None):

        self.weight = weight
        self.type = type
        self.containment = containment

    def append_to_atomspace(self, atomspace):
        atomspace = AtomSpace()
        entity = atomspace.add_node(types.ConceptNode, generate_unique_name(self.type))


    def __repr__(self):
        pass
