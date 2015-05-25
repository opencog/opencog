__author__ = 'DongMin Kim'

from abc import ABCMeta, abstractmethod

from opencog.type_constructors import *

from util import blending_util


class BaseTestCase(object):
    """
    :type a: opencog.atomspace_details.AtomSpace
    """
    __metaclass__ = ABCMeta

    def __init__(self, atomspace):
        self.a = atomspace
        self.atom_list_for_debug = []
        self.link_list_for_debug = []
        self.default_atom_tv = TruthValue(0.9, 0.8)
        self.default_link_tv = TruthValue(0.7, 0.6)
        self.a_blend_target = \
            self.a.get_atoms_by_name(
                types.Atom,
                blending_util.BLEND_TARGET_NODE_NAME
            )[0]

    def __str__(self):
        return 'BaseTestCase'

    @abstractmethod
    def make(self):
        pass