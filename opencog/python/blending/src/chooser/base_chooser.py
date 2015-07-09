from abc import ABCMeta, abstractmethod

from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class BaseChooser(object):
    __metaclass__ = ABCMeta

    def __init__(self, a):
        self.a = a
        self.last_status = blending_status.UNKNOWN_ERROR
        self.make_default_config()
        self.ret = []

    def make_default_config(self):
        BlendConfig().update(self.a, "choose-atom-type", "Node")
        BlendConfig().update(self.a, "choose-least-count", "2")

    @abstractmethod
    def atom_choose_impl(self, focus_atoms, config_base):
        raise NotImplementedError("Please implement this method.")

    def atom_choose(self, focus_atoms, config_base):
        self.last_status = blending_status.IN_PROCESS

        self.atom_choose_impl(focus_atoms, config_base)

        if self.last_status == blending_status.IN_PROCESS:
            self.last_status = blending_status.SUCCESS
        else:
            self.ret = []
            raise UserWarning('ERROR_IN_ATOMS_CHOOSER')

        return self.ret
