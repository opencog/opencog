from abc import ABCMeta, abstractmethod

from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class BaseMaker(object):
    """Base class of new blend atom maker.

    Attributes:
        a: An instance of atomspace.
        last_status: A last status of class.
        ret: A merged atom.
        :type a: opencog.atomspace.AtomSpace
        :type last_status: int
        :type ret: Atom
    """

    __metaclass__ = ABCMeta

    def __init__(self, a):
        self.a = a
        self.last_status = blending_status.UNKNOWN_ERROR
        self.ret = None

        self.make_default_config()

    def make_default_config(self):
        BlendConfig().update(self.a, "make-atom-prefix", "")
        BlendConfig().update(self.a, "make-atom-separator", "-")
        BlendConfig().update(self.a, "make-atom-postfix", "")

    @abstractmethod
    def new_blend_make_impl(self, decided_atoms, config_base):
        raise NotImplementedError("Please implement this method.")

    def new_blend_make(self, decided_atoms, config_base):
        self.last_status = blending_status.IN_PROCESS

        self.new_blend_make_impl(decided_atoms, config_base)

        if self.last_status == blending_status.IN_PROCESS:
            self.last_status = blending_status.SUCCESS
        else:
            self.ret = None
            raise UserWarning('ERROR_IN_BLEND_ATOM_MAKER')

        return self.ret
