from abc import ABCMeta, abstractmethod

from opencog.logger import log

from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status, get_status_str

__author__ = 'DongMin Kim'


class BaseConnector(object):
    __metaclass__ = ABCMeta

    def __init__(self, a):
        self.a = a
        self.last_status = blending_status.UNKNOWN_ERROR
        self.make_default_config()
        self.ret = None

    def make_default_config(self):
        pass

    @abstractmethod
    def link_connect_impl(self, decided_atoms, merged_atom, config_base):
        raise NotImplementedError("Please implement this method.")

    def link_connect(self, decided_atoms, merged_atom, config_base):
        self.last_status = blending_status.IN_PROCESS

        try:
            self.link_connect_impl(decided_atoms, merged_atom, config_base)
        except UserWarning as e:
            log.info("Skipping connect, caused by '" + str(e) + "'")
            log.info(
                "Last status is '" +
                get_status_str(self.last_status) +
                "'"
            )
            raise e

        if self.last_status == blending_status.IN_PROCESS:
            self.last_status = blending_status.SUCCESS

        return self.ret
