from abc import ABCMeta, abstractmethod

from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class BaseConnector(object):
    """Abstract class to provide 'link_connect()' interface.

    The blender will call the method 'link_connect()', and this method will
    call the method 'link_connect_impl()' in the derived class.

    Attributes:
        a: An instance of AtomSpace.
        last_status: A last status of class.
        ret: The blended atoms.
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
        pass

    @abstractmethod
    def link_connect_impl(self, decided_atoms, merged_atom, config_base):
        """Abstract factory method for derived class.

        Args:
            decided_atoms: The source atoms to make new atom.
            merged_atom: The atom to connect new links.
            config_base: A Node to save custom config.
            :param decided_atoms: list[Atom]
            :param merged_atom: Atom
            :param config_base: Atom
        Raises:
            NotImplementedError: Someone tried to call the abstract method.
        """
        raise NotImplementedError("Please implement this method.")

    def link_connect(self, decided_atoms, merged_atom, config_base):
        """Wrapper method to control exception in derived class.

        Args:
            decided_atoms: The source atoms to make new atom.
            merged_atom: The atom to connect new links.
            config_base: A Node to save custom config.
            :param decided_atoms: list[Atom]
            :param merged_atom: Atom
            :param config_base: Atom
        Returns:
            The blended atom(s) that connected with new links.
            Example:
            [(ConceptNode "new-atom-connected-1-A"),
             (ConceptNode "new-atom-connected-1-B"),
             (ConceptNode "new-atom-connected-1-C"),
             ...
             (ConceptNode "new-atom-connected-2-A"),
             (ConceptNode "new-atom-connected-2-B"),
             (ConceptNode "new-atom-connected-2-C"),
             ...]
            If a list is empty, then means link connector couldn't found the
            proper links with given condition.
            :rtype : list[Atom]
        Raises:
            UserWarning: An error occurred in connecting.
        """
        self.last_status = blending_status.IN_PROCESS

        self.link_connect_impl(decided_atoms, merged_atom, config_base)

        if self.last_status == blending_status.IN_PROCESS:
            self.last_status = blending_status.SUCCESS
        else:
            self.ret = []
            raise UserWarning('ERROR_IN_LINK_CONNECTOR')

        return self.ret
