from blending.src.connector.base_connector import BaseConnector
from blending.src.connector.connect_util import *
import blending.src.connector.equal_link_key as eq_link

__author__ = 'DongMin Kim'


class ConnectSimple(BaseConnector):
    """Connect every links.
    """

    def __init__(self, a):
        super(self.__class__, self).__init__(a)

    def make_default_config(self):
        super(self.__class__, self).make_default_config()

    def __prepare_blended_atoms(self, conflict_links, merged_atom):
        self.ret = [merged_atom]

    def __connect_links(self, duplicate_links, non_duplicate_links):
        for i, merged_atom in enumerate(self.ret):
            # Just copy.
            for link in duplicate_links:
                eq_link.key_to_link(self.a, link, merged_atom, link.tv)
            for link in non_duplicate_links:
                eq_link.key_to_link(self.a, link, merged_atom, link.tv)

    def __connect_to_blended_atoms(self, decided_atoms):
        # Make the links between source nodes and newly blended node.
        # TODO: Give proper truth value, not average of truthvalue.
        for merged_atom in self.ret:
            weighted_tv = get_weighted_tv(self.a.get_incoming(merged_atom.h))
            for decided_atom in decided_atoms:
                self.a.add_link(
                    types.AssociativeLink,
                    [decided_atom, merged_atom],
                    weighted_tv
                )

    def __connect_links_simple(self, decided_atoms, merged_atom):
        duplicate_links, non_duplicate_links = \
            find_duplicate_links(self.a, decided_atoms)
        # Evaluate the weighted average of truth value
        # between the duplicate links.
        conflict_links, non_conflict_links = \
            find_conflict_links(self.a, duplicate_links, types.NO_TYPE, -1, -1)
        self.__prepare_blended_atoms(None, merged_atom)
        self.__connect_links(
            non_conflict_links, non_duplicate_links
        )
        self.__connect_to_blended_atoms(decided_atoms)

    def link_connect_impl(self, decided_atoms, merged_atom, config_base):
        self.__connect_links_simple(decided_atoms, merged_atom)
