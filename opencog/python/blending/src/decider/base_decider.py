from abc import ABCMeta, abstractmethod

from blending.util.blend_config import BlendConfig
from blending.util.blend_logger import blend_log
from blending.util.general_util import enum_simulate

__author__ = 'DongMin Kim'


class BaseDecider(object):
    """
    :type a: opencog.atomspace_details.AtomSpace
    """
    __metaclass__ = ABCMeta

    Status = enum_simulate(
        'SUCCESS_DECIDE',
        'IN_PROCESS',
        'UNKNOWN_ERROR',
        'PARAMETER_ERROR',
        'NOT_ENOUGH_ATOMS'
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
                if x == BaseDecider.Status.SUCCESS_DECIDE
                else False
                )(self.last_status)

    def make_default_config(self):
        BlendConfig().update(self.a, "decide-result-atoms-count", "2")

    @abstractmethod
    def blending_decide_impl(self, chosen_atoms, config_base):
        """
        :param chosen_atoms: list
        :return: list
        """
        raise NotImplementedError("Please implement this method.")

    def blending_decide(self, chosen_atoms, config_base):
        self.last_status = self.Status.IN_PROCESS

        try:
            self.blending_decide_impl(chosen_atoms, config_base)
        except UserWarning as e:
            blend_log("Skipping decide, caused by '" + str(e) + "'")
            blend_log(
                "Last status is '" +
                self.Status.reverse_mapping[self.last_status] +
                "'"
            )
            raise e

        if self.last_status == self.Status.IN_PROCESS:
            self.last_status = self.Status.SUCCESS_DECIDE

        return self.ret
