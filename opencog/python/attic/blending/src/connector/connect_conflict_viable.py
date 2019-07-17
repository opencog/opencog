import itertools

from opencog.atomspace import *

from blending.src.connector.base_connector import BaseConnector
from blending.src.connector.connect_util import *
import blending.src.connector.equal_link_key as eq_link
from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class ConnectConflictAllViable(BaseConnector):
    """Link connector that connects every viable cases of links to new blends.

    If there exists k conflict links pair, then connector will makes the 2^k
    new blends, and connects each viable case to each new blend.

    1. Find the duplicate links, and the non-duplicate links.
    2. Find the conflict links, and non-conflict links from duplicate links.
    3. Make 2^k available conflict link cases if there exists k conflicts.
    4. Connects every viable cases of links to new blends.

    Attributes:
        check_type: A link type to check conflict.
        strength_diff_limit: A limit of difference between links strength value.
        confidence_above_limit: A threshold of both links confidence value.
        viable_atoms_count_threshold: A max limit count of new blend atoms.
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
        """Initialize a default config for this class."""
        super(self.__class__, self).make_default_config()
        BlendConfig().update(self.a, "connect-check-type", "SimilarityLink")
        BlendConfig().update(self.a, "connect-strength-diff-limit", "0.3")
        BlendConfig().update(self.a, "connect-confidence-above-limit", "0.7")
        BlendConfig().update(self.a, "connect-viable-atoms-count-limit", "100")

    def __prepare_blended_atoms(self, conflict_links, merged_atom):
        """Prepare new blend atoms list if the number of result atom is
        expected to more than one.

        Args:
            conflict_links: Conflicted link tuples list.
            merged_atom: The atom to connect new links.
            :param merged_atom: Atom
        """
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

    def __connect_links(self, conflict_link_cases, non_conflict_links,
                        non_duplicate_links):
        """Connect the every viable cases of links to new blends.

        Args:
            conflict_link_cases: conflicted links list.
            non_conflict_link_cases: non-conflicted links list.
            non_duplicate_link_cases: non-duplicated links list.
            :param conflict_link_cases: list[list[EqualLinkKey]]
            :param non_conflict_links: list[EqualLinkKey]
            :param non_conflict_links: list[EqualLinkKey]
        """
        for i, merged_atom in enumerate(self.ret):
            # 1. Connect to each viable atoms.
            for conflict_link_case in conflict_link_cases:
                for link in conflict_link_case:
                    eq_link.key_to_link(self.a, link, merged_atom, link.tv)
            # 2. Others, connect link with weighted average of truth value
            # between the duplicate links.
            for link in non_conflict_links:
                eq_link.key_to_link(self.a, link, merged_atom, link.tv)
            # 3. Just copy.
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

    def __connect_viable_conflict_links(self, decided_atoms, merged_atom):
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

        # 2. Find the conflict links, and non-conflict links from
        # duplicate links.
        conflict_links, non_conflict_links = \
            find_conflict_links(
                self.a, duplicate_links,
                self.check_type,
                self.strength_diff_limit,
                self.confidence_above_limit
            )

        # 3. Make 2^k available conflict link cases if there exists k conflicts.
        conflict_link_cases = make_conflict_link_cases(conflict_links)

        # 4. Connect the every viable cases of links to new blends.
        self.__prepare_blended_atoms(conflict_links, merged_atom)
        self.__connect_links(
            conflict_link_cases, non_conflict_links, non_duplicate_links
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

        # Check if given atom_type is valid or not.
        try:
            self.check_type = types.__dict__[check_type_str]
        except KeyError:
            self.check_type = types.Node

        # Check if given threshold value is valid or not.
        self.strength_diff_limit = float(strength_diff_threshold)
        self.confidence_above_limit = float(confidence_above_threshold)
        self.viable_atoms_count_threshold = viable_atoms_count_threshold

        self.__connect_viable_conflict_links(decided_atoms, merged_atom)
