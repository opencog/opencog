# coding=utf-8
import random
from examples.python.conceptual_blending.networks.network_util import \
    make_link_all
from opencog.atomspace import get_type, get_type_name
from opencog_b.python.blending.connector.base_connector import \
    BaseConnector
from opencog_b.python.blending.connector.connect_util import \
    make_link_from_equal_link_key, find_duplicate_links
from opencog_b.python.blending.util.blend_config import BlendConfig
from opencog_b.python.blending.util.blend_logger import blend_log
from opencog_b.python.blending.util.blending_util import *

__author__ = 'DongMin Kim'


class ConnectConflictRandom(BaseConnector):
    # TODO: Currently, this class can handle
    # when the number of decided atom is only 2.
    def __init__(self, a):
        super(self.__class__, self).__init__(a)
        self.check_type = None
        self.strength_diff_limit = None
        self.confidence_above_limit = None

    def __str__(self):
        return self.__class__.__name__

    def make_default_config(self):
        super(self.__class__, self).make_default_config()
        BlendConfig().update(self.a, "connect-check-type", "SimilarityLink")
        BlendConfig().update(self.a, "connect-strength-diff-limit", "0.3")
        BlendConfig().update(self.a, "connect-confidence-above-limit", "0.7")

    def __is_conflict(self, duplicate_links):
        link_0 = duplicate_links[0]
        link_1 = duplicate_links[1]

        if not self.a[link_0.h].is_a(self.check_type):
            return False

        if abs(link_0.tv.mean - link_1.tv.mean) > self.strength_diff_limit and \
           link_0.tv.confidence > self.confidence_above_limit and \
           link_1.tv.confidence > self.confidence_above_limit:
            return True

        """
        # TODO: Currently, this method can handle
        # when the number of decided atom is only 2.
        conflict_links = []
        non_conflict_links = []

        pair_links = itertools.combinations(duplicate_links, 2)
        for pair_link in pair_links:
            link_0 = pair_link[0]
            link_1 = pair_link[1]

            # TODO: mean is strength?
            strength_diff = abs(link_0.tv.mean - link_1.tv.mean)
            if strength_diff > self.strength_diff_limit and \
               link_0.tv.confidence > self.confidence_above_limit and \
               link_1.tv.confidence > self.confidence_above_limit:
                conflict_links.append(pair_link)
            else:
                non_conflict_links.append(pair_link)
        """

        return False

    def __connect_duplicate_links(self, duplicate_links, dst_node):
        """
        Connect duplicate links.

        Make the links between exist nodes, with detect and improve conflict
        links in newly blended node.

        :param dict(Handle, list(EqualLinkKey)) duplicate_links: List of
        duplicated link expressed by EqualLinkKey list.
        :param types.Atom dst_node: node to connect new links.
        """
        conflict_links = []
        non_conflict_links = []

        for links in duplicate_links:
            if self.__is_conflict(links):
                conflict_links.append(links)
            else:
                non_conflict_links.append(links)

        # 1. Choose one link randomly in each conflict link set.
        for links in conflict_links:
            link_key = random.choice(links)
            make_link_from_equal_link_key(
                self.a, link_key, dst_node, link_key.tv
            )

        # 2. Others, evaluate the weighted average of truth value between
        # the duplicate links.
        for links in non_conflict_links:
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

    def __connect_links_simple(self, decided_atoms, new_blended_atom):
        """
        Implementation of simple link connector.

        1. Find duplicate, non-duplicate links both.
        2. Try to improve some conflict in duplicate links and connect to new
         blended atom.
        3. Try to connect to new blended atom.

        :param list(types.Atom) decided_atoms: List of atoms to search
        links to be connected to new blended atom.
        :param Atom new_blended_atom: New blended atom.
        """
        duplicate_links, non_duplicate_links = \
            find_duplicate_links(self.a, decided_atoms)

        self.__connect_duplicate_links(duplicate_links, new_blended_atom)
        self.__connect_non_duplicate_links(non_duplicate_links, new_blended_atom)

        # Make the links between source nodes and newly blended node.
        # TODO: Give proper truth value, not average of truthvalue.
        # 평균 진릿값 말고 적당한 진릿값을 주어야 한다.
        for new_blended_atom in self.ret:
            try:
                weighted_tv = get_weighted_tv(
                    self.a.get_incoming(new_blended_atom.h))
            except UserWarning as e:
                blend_log(e)
                weighted_tv = TruthValue()
            for decided_atom in decided_atoms:
                self.a.add_link(
                    types.AssociativeLink,
                    [decided_atom, new_blended_atom],
                    weighted_tv
                )

        self.ret.append(new_blended_atom)

    def link_connect_impl(self, decided_atoms, new_blended_atom, config_base):
        check_type_str = BlendConfig().get_str(
            self.a, "connect-check-type", config_base
        )
        strength_diff_threshold = BlendConfig().get_str(
            self.a, "connect-strength-diff-limit", config_base
        )
        confidence_above_threshold = BlendConfig().get_str(
            self.a, "connect-confidence-above-limit", config_base
        )

        self.check_type = get_type(check_type_str)

        if check_type_str != get_type_name(self.check_type):
            raise UserWarning("Can't resolve type" + str(check_type_str))

        try:
            self.strength_diff_limit = \
                float(strength_diff_threshold)
            self.confidence_above_limit = \
                float(confidence_above_threshold)
        except (TypeError, ValueError):
            raise UserWarning(
                "Can't parse threshold value:: "
                "{strength: {0}, confidence: {1}}".format(
                    str(strength_diff_threshold),
                    str(confidence_above_threshold))
            )

        self.__connect_links_simple(decided_atoms, new_blended_atom)
