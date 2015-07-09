from abc import ABCMeta, abstractmethod

from opencog.logger import log

from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class BaseChooser(object):
    __metaclass__ = ABCMeta

    def __init__(self, a):
        self.a = a
        self.last_status = blending_status.UNKNOWN_ERROR
        self.make_default_config()
        self.ret = None

    def make_default_config(self):
        BlendConfig().update(self.a, "choose-atom-type", "Node")
        BlendConfig().update(self.a, "choose-least-count", "2")

    @abstractmethod
    def atom_choose_impl(self, config_base):
        raise NotImplementedError("Please implement this method.")

    def atom_choose(self, config_base):
        self.last_status = blending_status.IN_PROCESS

        try:
            self.atom_choose_impl(config_base)
        except UserWarning as e:
            log.info("Skipping choose, caused by '" + str(e) + "'")
            log.info(
                "Last status is '" +
                blending_status.reverse_mapping[self.last_status] +
                "'"
            )
            raise e

        if self.last_status == blending_status.IN_PROCESS:
            self.last_status = blending_status.SUCCESS

        return self.ret
