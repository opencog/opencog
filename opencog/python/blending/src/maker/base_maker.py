from abc import ABCMeta, abstractmethod

from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class BaseMaker(object):
    """Abstract class to provide 'new_blend_make()' interface.

    The blender will call the method 'new_blend_make()', and this method will
    call the method 'new_blend_make_impl()' in the derived class.

    Attributes:
        a: An instance of AtomSpace.
        last_status: A last status of class.
        ret: Newly made atom.
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
        BlendConfig().update(self.a, "make-atom-prefix", "")
        BlendConfig().update(self.a, "make-atom-separator", "-")
        BlendConfig().update(self.a, "make-atom-postfix", "")

    @abstractmethod
    def new_blend_make_impl(self, decided_atoms, config_base):
        """Abstract factory method for derived class.

        Args:
            decided_atoms: The source atoms to make new atom.
            config_base: A Node to save custom config.
            :param decided_atoms: list[Atom]
            :param config_base: Atom
        Raises:
            NotImplementedError: Someone tried to call the abstract method.
        """
        raise NotImplementedError("Please implement this method.")

    def new_blend_make(self, decided_atoms, config_base):
        """Wrapper method to control exception in derived class.

        Args:
            decided_atoms: The source atoms to make new atom.
            config_base: A Node to save custom config.
            :param decided_atoms: list[Atom]
            :param config_base: Atom
        Returns:
            Newly made atom.
            Example:
            (ConceptNode "atom-1-atom-3")
            :rtype : Atom
        Raises:
            UserWarning: An error occurred in making the new atom.
        """
        self.last_status = blending_status.IN_PROCESS

        self.new_blend_make_impl(decided_atoms, config_base)

        if self.last_status == blending_status.IN_PROCESS:
            self.last_status = blending_status.SUCCESS
        else:
            self.ret = None
            raise UserWarning('ERROR_IN_BLEND_ATOM_MAKER')

        return self.ret
