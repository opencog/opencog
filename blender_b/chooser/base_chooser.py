from util_b.general_util import enum_simulate, BlLogger
from abc import ABCMeta, abstractmethod

__author__ = 'DongMin Kim'


class BaseChooser(object):
    """
    :type a: opencog.atomspace_details.AtomSpace
    """
    __metaclass__ = ABCMeta

    Status = enum_simulate(
        'SUCCESS_CHOOSE',
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
                if x == BaseChooser.Status.SUCCESS_CHOOSE
                else False
                )(self.last_status)

    def make_default_config(self):
        pass

    @abstractmethod
    def atom_choose_impl(self, config):
        """
        :param config: dict
        :return: list
        """
        raise NotImplementedError("Please implement this method.")

    def atom_choose(self, config=None):
        self.last_status = self.Status.IN_PROCESS

        try:
            self.atom_choose_impl(config)
        except UserWarning as e:
            BlLogger().log("Skipping choose, caused by '" + str(e) + "'")
            BlLogger().log(
                "Last status is '" +
                self.Status.reverse_mapping[self.last_status] +
                "'"
            )

        if self.last_status == self.Status.IN_PROCESS:
            self.last_status = self.Status.SUCCESS_CHOOSE

        return self.ret
