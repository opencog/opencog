# coding=utf-8
from examples.python.conceptual_blending.networks.network_util import \
    make_link_all
from opencog_b.python.blending.connector.base_connector import \
    BaseConnector
from opencog_b.python.blending.connector.connect_util import \
    make_link_from_equal_link_key, find_duplicate_links
from opencog_b.python.blending.util.blend_logger import blend_log
from opencog_b.python.blending.util.blending_util import *

__author__ = 'DongMin Kim'


class ConnectSimple(BaseConnector):
    def __init__(self, a):
        super(self.__class__, self).__init__(a)

    def __str__(self):
        return self.__class__.__name__

    def make_default_config(self):
        super(self.__class__, self).make_default_config()

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

    def __connect_links_simple(self, decided_atoms, new_blended_atoms):
        """
        Implementation of simple link connector.

        1. Find duplicate, non-duplicate links both.
        2. Try to improve some conflict in duplicate links and connect to new
         blended atom.
        3. Try to connect to new blended atom.

        :param list(types.Atom) decided_atoms: List of atoms to search
        links to be connected to new blended atom.
        :param types.Atom new_blended_atoms: New blended atom.
        """
        # In ConnectSimple, number of result atoms is only one.
        if len(new_blended_atoms) < 1:
            self.last_status = self.Status.NOT_ENOUGH_ATOMS
            raise UserWarning(
                'New target atoms to copy links was not provided.'
            )
        elif len(new_blended_atoms) > 1:
            self.last_status = self.Status.TOO_MANY_ATOMS
            raise UserWarning(
                'Too many new target atoms. ' +
                'ConnectSimple can handle one atom in each execute.'
            )

        new_blended_atom = new_blended_atoms[0]

        duplicate_links, non_duplicate_links = \
            find_duplicate_links(self.a, decided_atoms)

        self.__connect_duplicate_links(duplicate_links, new_blended_atom)
        self.__connect_non_duplicate_links(non_duplicate_links, new_blended_atom)

        # Make the links between source nodes and newly blended node.
        # TODO: Give proper truth value, not average of truthvalue.
        # 평균 진릿값 말고 적당한 진릿값을 주어야 한다.
        try:
            weighted_tv = get_weighted_tv(self.a.get_incoming(new_blended_atom.h))
        except UserWarning as e:
            blend_log(e)
            weighted_tv = TruthValue()
        for decided_atom in decided_atoms:
            self.a.add_link(
                types.AssociativeLink,
                [decided_atom, new_blended_atom],
                weighted_tv
            )

    def link_connect_impl(self, decided_atoms, new_blended_atoms, config_base):
        self.__connect_links_simple(decided_atoms, new_blended_atoms)
