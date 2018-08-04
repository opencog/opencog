from abc import ABCMeta, abstractmethod

from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class BaseDecider(object):
    """Abstract class to provide 'blending_decide()' interface.

    The blender will call the method 'blending_decide()', and this method will
    call the method 'blending_decide_impl()' in the derived class.

    Attributes:
        a: An instance of AtomSpace.
        last_status: A last status of class.
        ret: The decided atoms.
        :type a: opencog.atomspace.AtomSpace
        :type last_status: int
        :type ret: list[Atom]
    """

    __metaclass__ = ABCMeta

    def __init__(self, a):
        self.a = a
        self.last_status = blending_status.UNKNOWN_ERROR
        self.ret = None

        self.make_default_config()

    def make_default_config(self):
        """Initialize a default config for this class."""
        BlendConfig().update(self.a, "decide-result-atoms-count", "2")

    @abstractmethod
    def blending_decide_impl(self, chosen_atoms, config_base):
        """Abstract factory method for derived class.

        Args:
            chosen_atoms: The atoms to decide.
            config_base: A Node to save custom config.
            :param chosen_atoms: list[Atom]
            :param config_base: Atom
        Raises:
            NotImplementedError: Someone tried to call the abstract method.
        """
        raise NotImplementedError("Please implement this method.")

    def blending_decide(self, chosen_atoms, config_base):
        """Wrapper method to control exception in derived class.

        Args:
            chosen_atoms: The atoms to decide.
            config_base: A Node to save custom config.
            :param chosen_atoms: list[Atom]
            :param config_base: Atom
        Returns:
            The decided atom(s).
            Example:
            [(ConceptNode "decided-atom-1"),
             (ConceptNode "decided-atom-3"),
             ...]
            If a list is empty, then means blending decider couldn't decided the
            proper atom(s) with given condition.
            :rtype : list[Atom]
        Raises:
            UserWarning: An error occurred in deciding.
        """
        self.last_status = blending_status.IN_PROCESS

        self.blending_decide_impl(chosen_atoms, config_base)

        if self.last_status == blending_status.IN_PROCESS:
            self.last_status = blending_status.SUCCESS
        else:
            self.ret = []
            raise UserWarning('ERROR_IN_BLENDING_DECIDER')

        return self.ret
