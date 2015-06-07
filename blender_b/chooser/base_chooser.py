from util_b.general_util import enum_simulate

__author__ = 'DongMin Kim'

from abc import ABCMeta, abstractmethod


class BaseChooser(object):
    """
    :type a: opencog.atomspace_details.AtomSpace
    """

    Status = enum_simulate(
        'SUCCESS_CHOOSE',
        'IN_PROCESS',
        'UNKNOWN_ERROR',
        'PARAMETER_ERROR',
        'NOT_ENOUGH_ATOMS'
    )

    __metaclass__ = ABCMeta

    def __init__(self, a):
        self.a = a
        self.last_status = self.Status.UNKNOWN_ERROR
        self.make_default_config()

    def __str__(self):
        return self.__class__.__name__

    def get_last_status(self):
        return self.last_status

    def make_default_config(self):
        pass

    @abstractmethod
    def atom_choose(self, config=None):
        """
        :param config: dict
        :return: list
        """
        raise NotImplementedError("Please implement this method.")
