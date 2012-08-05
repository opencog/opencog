##
# @file m_adaptors.py
# @brief adaptor to opencog
# @author Dingjie.Wang
# @version 1.0
# @date 2012-07-31
from opencog.atomspace import Atom
from types_inheritance import type_to_name

class Viz_OpenCog_Tree_Adaptor(object):
    """ adaptor to tree type used in pln"""
    def __init__(self, tree_opencog):
        self._op = tree_opencog.op.name if isinstance(tree_opencog.op, Atom) else str(tree_opencog.op)
        self._children = []
        for child in tree_opencog.args:
            self._children.append(Viz_OpenCog_Tree_Adaptor(child))

    def get_op(self):
        return self._op
    def set_op(self, value):
        '''docstring for set_op''' 
        self._op = value

    op = property(get_op, set_op)

    def get_children(self):
        return self._children

    children = property(get_children)

class FakeAtom(object):
    """docstring for FakeAtom"""
    def __init__(self, type, name, tv = None, av = None):
        self.type = type
        self.name = name
        self.tv = tv
        self.av = av
        # @@ could just use attribute method
        try:
            self.type_name = type_to_name[type]         # a string
        except Exception, e:
            print "Unknown Atom type '%s' in line %s"%str(type)
            raise e
