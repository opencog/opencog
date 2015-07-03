# coding=utf-8
import itertools
from opencog.atomspace import get_type, get_type_name
from blending.src.connector.base_connector import BaseConnector
from blending.src.connector.connect_util import *
from blending.util.blend_config import BlendConfig
from blending.util.blend_logger import blend_log, debug_log
from blending.util.blending_util import *

__author__ = 'DongMin Kim'


class ConnectConflictAllViable(BaseConnector):
    # TODO: Currently, this class can handle
    # when the number of decided atom is only 2.
    def __init__(self, a):
        super(self.__class__, self).__init__(a)
        self.check_type = None
        self.strength_diff_limit = None
        self.confidence_above_limit = None
        self.viable_atoms_count_threshold = None

    def __str__(self):
        return self.__class__.__name__

    def make_default_config(self):
        super(self.__class__, self).make_default_config()
        BlendConfig().update(self.a, "connect-check-type", "SimilarityLink")
        BlendConfig().update(self.a, "connect-strength-diff-limit", "0.3")
        BlendConfig().update(self.a, "connect-confidence-above-limit", "0.7")
        BlendConfig().update(self.a, "connect-viable-atoms-count-limit", "100")

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
        :param Atom dst_node: node to connect new links.
        """
        conflict_links = []
        non_conflict_links = []

        for links in duplicate_links:
            if self.__is_conflict(links):
                conflict_links.append(links)
            else:
                non_conflict_links.append(links)

        # 1. Make all available new blended atoms.
        # Number of new atoms is expected to 2^k, if there exists k conflicts.
        if self.viable_atoms_count_threshold is not None:
            if self.viable_atoms_count_threshold < 2 ** len(conflict_links):
                # TODO: Control if expected result atoms count
                # is bigger than threshold
                debug_log(
                    "ConnectConflictAllViable: Too many atoms! ->" +
                    str(2 ** len(conflict_links)) +
                    " atoms will produce."
                )

        # 1-a. Prepare 2^k new blend atoms.
        new_blended_atoms = [dst_node]
        for i in xrange(1, 2 ** len(conflict_links)):
            new_blended_atoms.append(
                self.a.add_node(
                    dst_node.t, dst_node.name + '-' + str(i), dst_node.tv
                )
            )

        # 1-b. Prepare cartesian product iterator.
        # if number of conflict_links is 3, this iterator produces:
        # (0, 0, 0), (0, 0, 1), (0, 1, 0), (0, 1, 1), ... (1, 1, 1)
        cartesian_binary_iterator = \
            itertools.product([0, 1], repeat=len(conflict_links))

        # 1-c. Connect to each viable atoms.
        for i, viable_case_binary in enumerate(cartesian_binary_iterator):
            for j, selector in enumerate(viable_case_binary):
                make_link_from_equal_link_key(
                    self.a,
                    conflict_links[j][selector],
                    new_blended_atoms[i],
                    conflict_links[j][selector].tv
                )

        # 2. Others, evaluate the weighted average of truth value between
        # the duplicate links.
        for new_blend_atom in new_blended_atoms:
            for links in non_conflict_links:
                # array elements are same except original node information.
                link_key_sample = links[0]

                make_link_from_equal_link_key(
                    self.a,
                    link_key_sample,
                    new_blend_atom,
                    get_weighted_tv(links)
                )

        self.ret = new_blended_atoms

    def __connect_non_duplicate_links(self, non_duplicate_links):
        """
        Connect non-duplicate links.

        Make the links between exist nodes.

        :param dict(Handle, list(EqualLinkKey)) non_duplicate_links: List of
        non-duplicated link expressed by EqualLinkKey list.
        """
        # Just copying.
        for links in non_duplicate_links:
            for link in links:
                for new_blend_atom in self.ret:
                    make_link_from_equal_link_key(
                        self.a, link, new_blend_atom, link.tv
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
        self.__connect_non_duplicate_links(non_duplicate_links)

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
        viable_atoms_count_threshold = BlendConfig().get_str(
            self.a, "connect-viable-atoms-count-limit", config_base
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
                "{strength: {0}, confidence: {1}, count: {2}}".format(
                    str(strength_diff_threshold),
                    str(confidence_above_threshold),
                    str(viable_atoms_count_threshold)
                )
            )

        try:
            self.viable_atoms_count_threshold = \
                int(viable_atoms_count_threshold)
        except ValueError:
            self.viable_atoms_count_threshold = None

        self.__connect_links_simple(decided_atoms, new_blended_atom)
