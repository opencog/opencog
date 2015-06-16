from abc import ABCMeta, abstractmethod

from opencog_b.python.conceptual_blending.util.general_util import *

__author__ = 'DongMin Kim'


class BaseMaker(object):
    """
    :type a: opencog.atomspace_details.AtomSpace
    """
    __metaclass__ = ABCMeta

    Status = enum_simulate(
        'SUCCESS_MAKE',
        'IN_PROCESS',
        'UNKNOWN_ERROR',
        'PARAMETER_ERROR'
    )

    def __init__(self, a):
        self.a = a
        self.last_status = self.Status.UNKNOWN_ERROR
        self.make_default_config()
        self.ret = None

    def __str__(self):
        return self.__class__.__name__

    def is_succeeded(self):
        return (lambda x: True
                if x == BaseMaker.Status.SUCCESS_MAKE
                else False
                )(self.last_status)

    def make_default_config(self):
        BlendConfig().update(self.a, "make-atom-prefix", "")
        BlendConfig().update(self.a, "make-atom-separator", "-")
        BlendConfig().update(self.a, "make-atom-postfix", "")

    @abstractmethod
    def new_blend_make_impl(self, decided_atoms, config_base):
        """
        :param decided_atoms: list
        """
        raise NotImplementedError("Please implement this method.")

    def new_blend_make(self, decided_atoms, config_base):
        self.last_status = self.Status.IN_PROCESS

        try:
            self.new_blend_make_impl(decided_atoms, config_base)
        except UserWarning as e:
            BlLogger().log("Skipping make, caused by '" + str(e) + "'")
            BlLogger().log(
                "Last status is '" +
                self.Status.reverse_mapping[self.last_status] +
                "'"
            )
            raise e

        if self.last_status == self.Status.IN_PROCESS:
            self.last_status = self.Status.SUCCESS_MAKE

        return self.ret
