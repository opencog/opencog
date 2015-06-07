from opencog.type_constructors import *
from abc import ABCMeta, abstractmethod

__author__ = 'DongMin Kim'


class BaseTestCase(object):
    """
    :type a: opencog.atomspace_details.AtomSpace
    """
    __metaclass__ = ABCMeta

    def __init__(self, a):
        self.a = a

        self.default_atom_tv = TruthValue(0.9, 0.8)
        self.default_link_tv = TruthValue(0.7, 0.6)

    def __str__(self):
        return self.__class__.__name__

    @abstractmethod
    def make(self):
        raise NotImplementedError("Please implement this method.")
