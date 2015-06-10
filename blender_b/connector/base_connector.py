from util_b.general_util import enum_simulate, BlLogger
from abc import ABCMeta, abstractmethod

__author__ = 'DongMin Kim'


class BaseConnector(object):
    """
    :type a: opencog.atomspace_details.AtomSpace
    """
    __metaclass__ = ABCMeta

    Status = enum_simulate(
        'SUCCESS_CONNECT',
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
                if x == BaseConnector.Status.SUCCESS_CONNECT
                else False
                )(self.last_status)

    def make_default_config(self):
        pass

    @abstractmethod
    def link_connect_impl(self, a_decided_atoms, a_new_blended_atom, config):
        """
        :param config: dict
        :return: list
        """
        raise NotImplementedError("Please implement this method.")

    def link_connect(self, a_decided_atoms, a_new_blended_atom, config=None):
        self.last_status = self.Status.IN_PROCESS

        try:
            self.link_connect_impl(a_decided_atoms, a_new_blended_atom, config)
        except UserWarning as e:
            BlLogger().log("Skipping connect, caused by '" + str(e) + "'")
            BlLogger().log(
                "Last status is '" +
                self.Status.reverse_mapping[self.last_status] +
                "'"
            )
            raise e

        if self.last_status == self.Status.IN_PROCESS:
            self.last_status = self.Status.SUCCESS_CONNECT

        return self.ret
