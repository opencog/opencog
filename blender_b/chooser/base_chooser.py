from util_b.blending_util import BlendTargetCtlForDebug
from util_b.general_util import BlConfig

__author__ = 'DongMin Kim'

from abc import ABCMeta, abstractmethod

from opencog.type_constructors import *

from util_b import blending_util


class BaseChooser(object):
    """
    :type a: opencog.atomspace_details.AtomSpace
    """

    SUCCESS_CHOOSE = 0
    IN_PROCESS = 1

    UNKNOWN_ERROR = 10
    PARAMETER_ERROR = 11

    NOT_ENOUGH_ATOMS = 12

    __metaclass__ = ABCMeta

    def __init__(self, a):
        self.a = a
        if BlConfig().is_use_blend_target:
            self.a_blend_target = BlendTargetCtlForDebug().get_blend_target()

        self.last_status = self.UNKNOWN_ERROR

    def __str__(self):
        return 'BaseChooser'

    @abstractmethod
    def atom_choose(self, is_use_config_file=True, option=None):
        """
        :param is_use_config_file: bool
        :param option: dict
        :return: list
        """
        raise NotImplementedError("Please implement this method.")
