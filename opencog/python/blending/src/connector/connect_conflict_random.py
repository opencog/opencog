import random
from opencog.atomspace import *

from blending.src.connector.base_connector import BaseConnector
from blending.src.connector.connect_util import *
import blending.src.connector.equal_link_key as eq_link
from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class ConnectConflictRandom(BaseConnector):
    """Choose one link in each of conflict links and connect.

    Attributes:
        check_type: A link type to check conflict.
        strength_diff_limit: A limit of difference between links strength value.
        confidence_above_limit: A threshold of both links confidence value.
        :type check_type: opencog.type_constructors.types
        :type strength_diff_limit: float
        :type confidence_above_limit: float
    """

    # TODO: Currently, this class can handle
    # when the number of decided atom is only 2.
    def __init__(self, a):
        super(self.__class__, self).__init__(a)
        self.check_type = None
        self.strength_diff_limit = None
        self.confidence_above_limit = None

    def make_default_config(self):
        super(self.__class__, self).make_default_config()
        BlendConfig().update(self.a, "connect-check-type", "SimilarityLink")
        BlendConfig().update(self.a, "connect-strength-diff-limit", "0.3")
        BlendConfig().update(self.a, "connect-confidence-above-limit", "0.7")

    def __prepare_blended_atoms(self, conflict_links, merged_atom):
        self.ret = [merged_atom]

    def __connect_conflict_links(self, conflict_links):
        # 1. Choose one link randomly in each conflict link set.
        for merged_atom in self.ret:
            for links in conflict_links:
                link_key = random.choice(links)
                eq_link.key_to_link(self.a, link_key, merged_atom, link_key.tv)

    def __connect_non_conflict_links(self, non_conflict_links):
        # 2. Others, evaluate the weighted average of truth value between
        # the duplicate links.
        for merged_atom in self.ret:
            for links in non_conflict_links:
                # array elements are same except original node information.
                link_key_sample = links[0]
                eq_link.key_to_link(
                    self.a, link_key_sample, merged_atom, get_weighted_tv(links)
                )

    def __connect_non_duplicate_links(self, non_duplicate_links):
        # Just copy.
        for merged_atom in self.ret:
            for links in non_duplicate_links:
                for link in links:
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

    def __connect_random_conflict_links(self, decided_atoms, merged_atom):
        duplicate_links, non_duplicate_links = \
            find_duplicate_links(self.a, decided_atoms)
        conflict_links, non_conflict_links = \
            find_conflict_links(
                self.a, duplicate_links,
                self.check_type,
                self.strength_diff_limit,
                self.confidence_above_limit
            )

        self.__prepare_blended_atoms(conflict_links, merged_atom)

        self.__connect_conflict_links(conflict_links)
        self.__connect_non_conflict_links(non_conflict_links)
        self.__connect_non_duplicate_links(non_duplicate_links)

        self.__connect_to_blended_atoms(decided_atoms)

    def link_connect_impl(self, decided_atoms, merged_atom, config_base):
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
            self.last_status = blending_status.UNKNOWN_TYPE
            return

        self.strength_diff_limit = float(strength_diff_threshold)
        self.confidence_above_limit = float(confidence_above_threshold)

        self.__connect_random_conflict_links(decided_atoms, merged_atom)
