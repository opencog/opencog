from util_b.general_util import enum_simulate, BlLogger, BlendConfig
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
        BlendConfig().update(self.a, "choose-atom-type", "Node")
        BlendConfig().update(self.a, "choose-least-count", "2")

    @abstractmethod
    def atom_choose_impl(self, focus_atoms, config_base):
        """
        :param focus_atoms: dict
        :return: list
        """
        raise NotImplementedError("Please implement this method.")

    def atom_choose(self, focus_atoms, config_base):
        self.last_status = self.Status.IN_PROCESS

        try:
            self.atom_choose_impl(focus_atoms, config_base)
        except UserWarning as e:
            BlLogger().log("Skipping choose, caused by '" + str(e) + "'")
            BlLogger().log(
                "Last status is '" +
                self.Status.reverse_mapping[self.last_status] +
                "'"
            )
            raise e

        if self.last_status == self.Status.IN_PROCESS:
            self.last_status = self.Status.SUCCESS_CHOOSE

        return self.ret
