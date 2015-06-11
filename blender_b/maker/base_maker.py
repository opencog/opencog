from util_b.general_util import enum_simulate, BlLogger
from abc import ABCMeta, abstractmethod

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
        pass

    @abstractmethod
    def new_blend_make_impl(self, a_decided_atoms_list, config):
        """
        :param a_decided_atoms_list: list
        """
        raise NotImplementedError("Please implement this method.")

    def new_blend_make(self, a_decided_atoms_list, config=None):
        self.last_status = self.Status.IN_PROCESS

        try:
            self.new_blend_make_impl(a_decided_atoms_list, config)
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
