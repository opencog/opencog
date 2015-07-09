from abc import ABCMeta, abstractmethod

from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class BaseConnector(object):
    __metaclass__ = ABCMeta

    def __init__(self, a):
        self.a = a
        self.last_status = blending_status.UNKNOWN_ERROR
        self.make_default_config()
        self.ret = []

    def make_default_config(self):
        pass

    @abstractmethod
    def link_connect_impl(self, decided_atoms, merged_atom, config_base):
        raise NotImplementedError("Please implement this method.")

    def link_connect(self, decided_atoms, merged_atom, config_base):
        self.last_status = blending_status.IN_PROCESS

        self.link_connect_impl(decided_atoms, merged_atom, config_base)

        if self.last_status == blending_status.IN_PROCESS:
            self.last_status = blending_status.SUCCESS
        else:
            self.ret = []
            raise UserWarning('ERROR_IN_LINK_CONNECTOR')

        return self.ret
