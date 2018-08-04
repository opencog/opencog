from blending.src.connector.base_connector import BaseConnector
from blending.src.connector.connect_util import *
import blending.src.connector.equal_link_key as eq_link

__author__ = 'DongMin Kim'


class ConnectSimple(BaseConnector):
    """Link connector that connects every links to new blends.

    1. Find the duplicate links, and the non-duplicate links.
    2. Find the conflict links, and non-conflict links from duplicate links.
    3. Connects every links to new blends.
    """

    def __init__(self, a):
        super(self.__class__, self).__init__(a)

    def make_default_config(self):
        """Initialize a default config for this class."""
        super(self.__class__, self).make_default_config()

    def __prepare_blended_atoms(self, merged_atom):
        """Prepare new blend atoms list if the number of result atom is
        expected to more than one.

        Args:
            merged_atom: The atom to connect new links.
            :param merged_atom: Atom
        """
        self.ret = [merged_atom]

    def __connect_links(self, non_conflict_links, non_duplicate_links):
        """Connect the every links to new blend.

        Args:
            conflict_links: Conflicted link tuples list.
            non_duplicate_links: Non-duplicated links list.
            :param non_conflict_links: list[EqualLinkKey]
            :param non_duplicate_links: list[EqualLinkKey]
        """
        for i, merged_atom in enumerate(self.ret):
            # Just copy.
            for link in non_conflict_links:
                eq_link.key_to_link(self.a, link, merged_atom, link.tv)
            for link in non_duplicate_links:
                eq_link.key_to_link(self.a, link, merged_atom, link.tv)

    def __connect_to_blended_atoms(self, decided_atoms):
        """Connect source atoms to new blend atom.

        Args:
            decided_atoms: The source atoms to make new atom.
            :param decided_atoms: list[Atom]
        """
        # Make the links between source nodes and newly blended node.
        # TODO: Give proper truth value, not average of truthvalue.
        for merged_atom in self.ret:
            weighted_tv = get_weighted_tv(merged_atom.incoming)
            for decided_atom in decided_atoms:
                self.a.add_link(
                    types.AssociativeLink,
                    [decided_atom, merged_atom],
                    weighted_tv
                )

    def __connect_links_simple(self, decided_atoms, merged_atom):
        """Actual algorithm for connecting links.

        Args:
            decided_atoms: The source atoms to make new atom.
            merged_atom: The atom to connect new links.
            :param decided_atoms: list[Atom]
            :param merged_atom: Atom
        """
        # 1. Find the duplicate links, and the non-duplicate links.
        duplicate_links, non_duplicate_links = \
            find_duplicate_links(self.a, decided_atoms)

        # 2. Evaluate the weighted average of truth value between the
        # duplicate links.
        conflict_links, non_conflict_links = \
            find_conflict_links(self.a, duplicate_links, types.NO_TYPE, -1, -1)

        # 3. Connect the every links to new blends.
        self.__prepare_blended_atoms(merged_atom)
        self.__connect_links(
            non_conflict_links, non_duplicate_links
        )
        self.__connect_to_blended_atoms(decided_atoms)

    def link_connect_impl(self, decided_atoms, merged_atom, config_base):
        """Implemented factory method to connecting links.

        Args:
            decided_atoms: The source atoms to make new atom.
            merged_atom: The atom to connect new links.
            config_base: A Node to save custom config.
            :param decided_atoms: list[Atom]
            :param merged_atom: Atom
            :param config_base: Atom
        """
        self.__connect_links_simple(decided_atoms, merged_atom)
