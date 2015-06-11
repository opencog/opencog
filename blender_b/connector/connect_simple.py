# coding=utf-8
from blender_b.connector.base_connector import BaseConnector
from blender_b.connector.connect_util import *
from util_b.blending_util import get_weighted_tv
from util_b.general_util import BlConfig

__author__ = 'DongMin Kim'


class ConnectSimple(BaseConnector):
    def __init__(self, atomspace):
        super(self.__class__, self).__init__(atomspace)

    def __str__(self):
        return self.__class__.__name__

    def make_default_config(self):
        """
        default_config = {
        }
        BlConfig().make_default_config(str(self), default_config)
        """
        pass

    def __connect_duplicate_links(self, duplicate_links, dst_node):
        """
        Connect duplicate links.

        Make the links between exist nodes, with detect and improve conflict
        links in newly blended node.

        :param dict(Handle, list(EqualLinkKey)) duplicate_links: List of
        duplicated link expressed by EqualLinkKey list.
        :param types.Atom dst_node: node to connect new links.
        """
        # 1. If duplicate links have big conflict, remove one of them.
        # -- Do nothing.

        # 2. Others, evaluate the weighted average of truth value between
        # the duplicate links.
        for links in duplicate_links:
            # array elements are same except original node information.
            link_key_sample = links[0]

            make_link_from_equal_link_key(
                self.a, link_key_sample, dst_node, get_weighted_tv(links)
            )

    def __connect_non_duplicate_links(self, non_duplicate_links, dst_node):
        """
        Connect non-duplicate links.

        Make the links between exist nodes.

        :param dict(Handle, list(EqualLinkKey)) non_duplicate_links: List of
        non-duplicated link expressed by EqualLinkKey list.
        :param types.Atom dst_node: node to connect new links.
        """
        # Just copying.
        for links in non_duplicate_links:
            for link in links:
                make_link_from_equal_link_key(
                    self.a, link, dst_node, link.tv
                )

    def __connect_links_simple(self, a_decided_atoms, a_new_blended_atom):
        """
        Implementation of simple link connector.

        1. Find duplicate, non-duplicate links both.
        2. Try to improve some conflict in duplicate links and connect to new
         blended atom.
        3. Try to connect to new blended atom.

        :param list(types.Atom) a_decided_atoms: List of atoms to search
        links to be connected to new blended atom.
        :param types.Atom a_new_blended_atom: New blended atom.
        """
        duplicate_links, non_duplicate_links = \
            find_duplicate_links(self.a, a_decided_atoms)

        self.__connect_duplicate_links(duplicate_links, a_new_blended_atom)
        self.__connect_non_duplicate_links(non_duplicate_links, a_new_blended_atom)

    def link_connect_impl(self, a_decided_atoms, a_new_blended_atom, config):
        if config is None:
            config = BlConfig().get_section(str(self))

        self.__connect_links_simple(a_decided_atoms, a_new_blended_atom)
