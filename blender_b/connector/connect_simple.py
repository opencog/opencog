# coding=utf-8
from opencog.atomspace import TruthValue
from blender_b.connector.base_connector import BaseConnector
from blender_b.connector.connect_util import *
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

    def __get_weighted_tv(self, links):
        """
        Make new TruthValue by evaluate weighted average of exist
        link's TruthValue.

        https://groups.google.com/forum/#!topic/opencog/fa5c4yE8YdU

        :param list(EqualLinkKey) links: List of EqualLinkKey which are
        expected to make weighted average TruthValue from theirs.
        :rtype TruthValue: New truth value.
        """
        if len(links) < 2:
            self.last_status = self.Status.UNKNOWN_ERROR
            raise UserWarning(
                "Weighted TruthValue can't be evaluated with small size."
            )

        weighted_strength_sum = 0
        confidence_sum = 0
        link_count = 0

        for link in links:
            weighted_strength_sum += (link.tv.confidence * link.tv.mean)
            confidence_sum += link.tv.confidence
            link_count += 1

        new_strength = weighted_strength_sum / confidence_sum

        # TODO: Currently, confidence value for new blended node is just
        # average of old value.
        # 충돌값 보정을 단순 평균이 아닌 적절한 이유를 가진 값으로 바꿔야 한다.
        new_confidence = confidence_sum / link_count

        return TruthValue(new_strength, new_confidence)

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
            # array elements are same except original node.
            link_key = links[0]
            weighted_tv = self.__get_weighted_tv(links)
            make_link_from_equal_link_key(
                self.a, link_key, dst_node, weighted_tv
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
                make_link_from_equal_link_key(self.a, link, dst_node, link.tv)

    def link_connect_impl(self, a_decided_atoms, a_new_blended_atom, config):
        """
        Implementation of simple link connector.

        1. Find duplicate, non-duplicate links both.
        2. Try to improve some conflict in duplicate links and connect to new
         blended atom.
        3. Try to connect to new blended atom.

        :param list(types.Atom) a_decided_atoms: List of atoms to search
        links to be connected to new blended atom.
        :param types.Atom a_new_blended_atom: New blended atom.
        :param dict config: config.
        """
        duplicate_links, non_duplicate_links = \
            find_duplicate_links(self.a, a_decided_atoms)

        self.__connect_duplicate_links(duplicate_links, a_new_blended_atom)
        self.__connect_non_duplicate_links(non_duplicate_links, a_new_blended_atom)

    def link_connect(self, a_decided_atoms, a_new_blended_atom, config=None):
        if config is None:
            config = BlConfig().get_section(str(self))

        self.link_connect_impl(a_decided_atoms, a_new_blended_atom, config)


"""
    def __get_atoms_all(self, atom_type, least_count):

        Choose all atoms.
        :param Type atom_type: type of atoms to choose.
        :param int least_count: minimum number of atoms to choose.

        self.ret = self.a.get_atoms_by_type(atom_type, True)

        if len(self.ret) < least_count:
            self.last_status = self.Status.NOT_ENOUGH_ATOMS
            raise UserWarning('Size of atom list is too small.')
"""
