import itertools

from opencog.atomspace import *

from blending.src.connector.base_connector import BaseConnector
from blending.src.connector.connect_util import *
import blending.src.connector.equal_link_key as eq_link
from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class ConnectConflictAllViable(BaseConnector):
    """Make 2^k available(viable) new blend atoms if there exists k conflicts.

    Attributes:
        check_type: A link type to check conflict.
        strength_diff_limit: A limit of difference between links strength value.
        confidence_above_limit: A threshold of both links confidence value.
        viable_atoms_count_threshold: A max count limit of new blend atoms.
        :type check_type: opencog.type_constructors.types
        :type strength_diff_limit: float
        :type confidence_above_limit: float
        :type viable_atoms_count_threshold: int
    """

    # TODO: Currently, this class can handle
    # when the number of decided atom is only 2.
    def __init__(self, a):
        super(self.__class__, self).__init__(a)
        self.check_type = None
        self.strength_diff_limit = None
        self.confidence_above_limit = None
        self.viable_atoms_count_threshold = None

    def make_default_config(self):
        super(self.__class__, self).make_default_config()
        BlendConfig().update(self.a, "connect-check-type", "SimilarityLink")
        BlendConfig().update(self.a, "connect-strength-diff-limit", "0.3")
        BlendConfig().update(self.a, "connect-confidence-above-limit", "0.7")
        BlendConfig().update(self.a, "connect-viable-atoms-count-limit", "100")

    def __prepare_blended_atoms(self, conflict_links, merged_atom):
        # 1. Make all available new blended atoms.
        # Number of new atoms is expected to 2^k, if there exists k conflicts.
        if self.viable_atoms_count_threshold is not None:
            if self.viable_atoms_count_threshold < 2 ** len(conflict_links):
                # TODO: Control if expected result atoms count
                # is bigger than threshold
                log.warn(
                    "ConnectConflictAllViable: Too many atoms! ->" +
                    str(2 ** len(conflict_links)) +
                    " atoms will produce."
                )

        # 1-a. Prepare 2^k new blend atoms.
        self.ret = [merged_atom]
        for i in xrange(1, 2 ** len(conflict_links)):
            self.ret.append(
                self.a.add_node(
                    merged_atom.t,
                    merged_atom.name + '-' + str(i), merged_atom.tv
                )
            )

    def __connect_conflict_links(self, conflict_links):
        # 1-b. Prepare cartesian product iterator.
        # if number of conflict_links is 3, this iterator produces:
        # (0, 0, 0), (0, 0, 1), (0, 1, 0), (0, 1, 1), ... (1, 1, 1)
        cartesian_binary_iterator = \
            itertools.product([0, 1], repeat=len(conflict_links))

        # 1-c. Connect to each viable atoms.
        for i, viable_case_binary in enumerate(cartesian_binary_iterator):
            for j, selector in enumerate(viable_case_binary):
                eq_link.key_to_link(
                    self.a,
                    conflict_links[j][selector],
                    self.ret[i],
                    conflict_links[j][selector].tv
                )

    def __connect_non_conflict_links(self, non_conflict_links):
        # 2. Others, evaluate the weighted average of truth value between
        # the duplicate links.
        for merged_atom in self.ret:
            for links in non_conflict_links:
                # array elements are same except original node information.
                link_key_sample = links[0]
                eq_link.key_to_link(
                    self.a, link_key_sample, merged_atom,
                    get_weighted_tv(links)
                )

    def __connect_non_duplicate_links(self, non_duplicate_links):
        # Just copy.
        for links in non_duplicate_links:
            for link in links:
                for merged_atom in self.ret:
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

    def __connect_viable_conflict_links(self, decided_atoms, merged_atom):
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
        viable_atoms_count_threshold = BlendConfig().get_int(
            self.a, "connect-viable-atoms-count-limit", config_base
        )

        self.check_type = get_type(check_type_str)

        if check_type_str != get_type_name(self.check_type):
            self.last_status = blending_status.UNKNOWN_TYPE
            return

        self.strength_diff_limit = float(strength_diff_threshold)
        self.confidence_above_limit = float(confidence_above_threshold)
        self.viable_atoms_count_threshold = viable_atoms_count_threshold

        self.__connect_viable_conflict_links(decided_atoms, merged_atom)
