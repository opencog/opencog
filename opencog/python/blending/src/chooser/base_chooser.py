from abc import ABCMeta, abstractmethod

from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class BaseChooser(object):
    """Abstract class to provide 'atom_choose()' interface.

    The blender will call the method 'atom_choose()', and this method will call
    the method 'atom_choose_impl()' in the derived class.

    Attributes:
        a: An instance of AtomSpace.
        last_status: A last status of class.
        ret: The chosen atoms.
        :type a: opencog.atomspace.AtomSpace
        :type last_status: int
        :type ret: list[Atom]
    """

    __metaclass__ = ABCMeta

    def __init__(self, a):
        self.a = a
        self.last_status = blending_status.UNKNOWN_ERROR
        self.ret = []

        self.make_default_config()

    def make_default_config(self):
        """Initialize a default config for this class."""
        BlendConfig().update(self.a, "choose-atom-type", "Node")
        BlendConfig().update(self.a, "choose-least-count", "2")

    @abstractmethod
    def atom_choose_impl(self, focus_atoms, config_base):
        """Abstract factory method for derived class.

        Args:
            focus_atoms: The atoms to blend.
            config_base: A Node to save custom config.
            :param focus_atoms: list[Atom]
            :param config_base: Atom
        Raises:
            NotImplementedError: Someone tried to call the abstract method.
        """
        raise NotImplementedError("Please implement this method.")

    def atom_choose(self, focus_atoms, config_base):
        """Wrapper method to control exception in derived class.

        Args:
            focus_atoms: The atoms to blend.
            config_base: A Node to save custom config.
            :param focus_atoms: list[Atom]
            :param config_base: Atom
        Returns:
            The chosen atom(s).
            Example:
            [(ConceptNode "atom-0"),
             (ConceptNode "atom-1"),
             (ConceptNode "atom-2"),
             ...]
            If a list is empty, then means atoms chooser couldn't find the
            proper atom(s) with given condition.
            :rtype : list[Atom]
        Raises:
            UserWarning: An error occurred in choosing the atoms.
        """
        self.last_status = blending_status.IN_PROCESS

        self.atom_choose_impl(focus_atoms, config_base)

        if self.last_status == blending_status.IN_PROCESS:
            self.last_status = blending_status.SUCCESS
        else:
            self.ret = []
            raise UserWarning('ERROR_IN_ATOMS_CHOOSER')

        return self.ret
