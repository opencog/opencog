from abc import ABCMeta, abstractmethod

from opencog.logger import log

from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status, get_status_str

__author__ = 'DongMin Kim'


class BaseMaker(object):
    __metaclass__ = ABCMeta

    def __init__(self, a):
        self.a = a
        self.last_status = blending_status.UNKNOWN_ERROR
        self.make_default_config()
        self.ret = None

    def make_default_config(self):
        BlendConfig().update(self.a, "make-atom-prefix", "")
        BlendConfig().update(self.a, "make-atom-separator", "-")
        BlendConfig().update(self.a, "make-atom-postfix", "")

    @abstractmethod
    def new_blend_make_impl(self, decided_atoms, config_base):
        raise NotImplementedError("Please implement this method.")

    def new_blend_make(self, decided_atoms, config_base):
        self.last_status = blending_status.IN_PROCESS

        try:
            self.new_blend_make_impl(decided_atoms, config_base)
        except UserWarning as e:
            log.info("Skipping make, caused by '" + str(e) + "'")
            log.info(
                "Last status is '" +
                get_status_str(self.last_status) +
                "'"
            )
            raise e

        if self.last_status == blending_status.IN_PROCESS:
            self.last_status = blending_status.SUCCESS

        return self.ret
