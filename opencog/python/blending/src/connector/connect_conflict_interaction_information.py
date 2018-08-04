from opencog.atomspace import *
from opencog.logger import log
from opencog.statistics import \
    PyDataProviderAtom, PyProbabilityAtom, \
    PyEntropyAtom, PyInteractionInformationAtom
from blending.src.connector.base_connector import BaseConnector
from blending.src.connector.connect_util import *
import blending.src.connector.equal_link_key as eq_link
from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class ConnectConflictInteractionInformation(BaseConnector):
    """Link connector that connecting to new blend by checking the value of
    interaction information.

    This connector estimates the link set is most valuable, when it has the
    highest interaction information value.

    1. Find the duplicate links, and the non-duplicate links.
    2. Find the conflict links, and non-conflict links from duplicate links.
    3. Find the target links in nodes related with decided nodes(blend source).
       -> Get the related nodes by searching InheritanceLink.
    4. Make 2^k available conflict link cases if there exists k conflicts.
    5. Calculate probabilities with target links.
       -> Create the probabilities from example cases in whole AtomSpace.
    6. Calculate interaction information for each available conflict link cases.
    7. Choose one case which has largest interaction information and connect to
       new blend.

    Attributes:
        check_type: A link type to check conflict.
        strength_diff_limit: A limit of difference between links strength value.
        confidence_above_limit: A threshold of both links confidence value.
        data_n_gram_limit: A max value of n_gram during probability generation.
            -1 means infinite.
        evaluate_n_gram_limit: A max value of n_gram during
            interaction information generation. -1 means infinite.
        inter_info_strength_above_limit: A max value of strength in TruthValue
            during interaction information generation.
        :type check_type: opencog.type_constructors.types
        :type strength_diff_limit: float
        :type confidence_above_limit: float
        :type data_n_gram_limit: int
        :type evaluate_n_gram_limit: int
        :type inter_info_strength_above_limit: float
    """

    # TODO: Currently, this class can handle
    # when the number of decided atom is only 2.
    def __init__(self, a):
        super(self.__class__, self).__init__(a)
        self.check_type = None
        self.strength_diff_limit = None
        self.confidence_above_limit = None
        self.data_n_gram_limit = None
        self.evaluate_n_gram_limit = None
        self.inter_info_strength_above_limit = None

    def make_default_config(self):
        """Initialize a default config for this class."""
        super(self.__class__, self).make_default_config()
        BlendConfig().update(self.a, "connect-check-type", "SimilarityLink")
        BlendConfig().update(self.a, "connect-strength-diff-limit", "0.3")
        BlendConfig().update(self.a, "connect-confidence-above-limit", "0.7")
        BlendConfig().update(self.a, "connect-data-n-gram-limit", "None")
        BlendConfig().update(self.a, "connect-evaluate-n-gram-limit", "None")
        BlendConfig().update(
            self.a, "connect-inter-info-strength-above-limit", "0.5")

    def __prepare_blended_atoms(self, merged_atom):
        """Prepare new blend atoms list if the number of result atom is
        expected to more than one.

        Args:
            merged_atom: The atom to connect new links.
            :param merged_atom: Atom
        """
        self.ret = [merged_atom]

    def __get_max_n_gram(
            self,
            conflict_link_cases,
            non_conflict_link_cases,
            non_duplicate_link_cases,
            related_node_target_links
    ):
        """Decide the max value of n_gram, from every category link set.

        MAX(
           (USER DEFINED LIMIT),
           length of (related_node_target_link),
           length of (conflict_link + non_conflict_link + non_duplicate_link)
        )

        Args:
            conflict_link_cases: Conflicted link tuples list.
            non_conflict_link_cases: Non-conflict links list.
            non_duplicate_link_cases: Non-duplicated links list.
            related_node_target_links: Target link tuples in related node list.
            :param conflict_link_cases: list[list[EqualLinkKey]]
            :param non_conflict_link_cases: list[EqualLinkKey]
            :param non_duplicate_link_cases: list[EqualLinkKey]
            :param related_node_target_links: list[list[EqualLinkKey]]
        Returns:
            The max value of n_gram.
            :rtype : int
        """
        conflict_link_n_gram = 0 \
            if len(conflict_link_cases) == 0 \
            else len(conflict_link_cases[0])

        merged_link_n_gram = \
            conflict_link_n_gram + \
            len(non_conflict_link_cases) + \
            len(non_duplicate_link_cases)

        target_n_gram = list(map(lambda x: len(x), related_node_target_links))
        target_n_gram.append(merged_link_n_gram)

        n_gram = self.data_n_gram_limit \
            if 0 < self.data_n_gram_limit < max(target_n_gram) \
            else max(target_n_gram)

        if n_gram == self.data_n_gram_limit:
            log.info(
                "ConnectConflictInteractionInformation: "
                "n_gram was limited to: " + str(self.data_n_gram_limit) +
                ", original n_gram was: " + str(max(target_n_gram))
            )

        return n_gram

    def __make_statistics_data_provider(
            self,
            conflict_link_cases,
            non_conflict_link_cases,
            non_duplicate_link_cases,
            related_node_target_links
    ):
        """Make the statistics data provider.

        Args:
            conflict_link_cases: Conflicted link tuples list.
            non_conflict_link_cases: Non-conflict links list.
            non_duplicate_link_cases: Non-duplicated links list.
            related_node_target_links: Target link tuples in related node list.
            :param conflict_link_cases: list[list[EqualLinkKey]]
            :param non_conflict_link_cases: list[EqualLinkKey]
            :param non_duplicate_link_cases: list[EqualLinkKey]
            :param related_node_target_links: list[list[EqualLinkKey]]
        """
        self.provider = PyDataProviderAtom(
            self.__get_max_n_gram(
                conflict_link_cases,
                non_conflict_link_cases,
                non_duplicate_link_cases,
                related_node_target_links
            ),
            # Must be non-sorted information.
            False
        )

    def __generate_information_probability(
            self,
            related_node_target_links
    ):
        """Calculate probabilities with target links.

        It creates the entropy and probabilities from related nodes in
        whole AtomSpace.

        Args:
            related_node_target_links: Target link tuples in related node list
            :param related_node_target_links: list[list[EqualLinkKey]]
        Returns:
            The max value of n_gram.
            :rtype : int
        """
        log.debug(
            "ConnectConflictInteractionInformation: Calculating probabilities "
            "(Total: " + str(len(related_node_target_links)) + ")"
        )

        current_ratio = 0
        # Register the every link in related nodes to provider.
        for i, target_equal_link in enumerate(related_node_target_links):
            current_ratio = self.__print_progress(
                "ConnectConflictInteractionInformation:PROB:",
                current_ratio, i, len(related_node_target_links), 30
            )
            # TODO: To prevent freeze during probability generation,
            # user can limit the max value of calculation.
            max_repeat_length = self.provider.n_gram \
                if 0 < self.provider.n_gram < len(target_equal_link) \
                else len(target_equal_link)

            # Make n-gram data in each related node.
            # The provider with high n-gram will provides more correct data,
            # but speed will going slower rapidly.
            # (0, 0, 0), (0, 0, 1), (0, 1, 0), (0, 1, 1), ... (1, 1, 1)
            cartesian_binary_iterator = \
                itertools.product([False, True], repeat=max_repeat_length)

            for viable_case_binary in enumerate(cartesian_binary_iterator):
                gram_data = list()
                for j, selector in enumerate(viable_case_binary):
                    # Make each gram data.
                    if selector:
                        gram_data.append(eq_link.key_to_link(
                            self.a,
                            target_equal_link[j],
                            self.ret[0],
                            target_equal_link[j].tv
                        ))
                # Register the generated gram_data.
                self.provider.add_one_rawdata_count(
                    [data for data in gram_data if data is not None], 1
                )

        # Update provider's statistic data.
        PyProbabilityAtom().calculate_probabilities(self.provider)
        PyEntropyAtom().calculate_entropies(self.provider)

    def __evaluate_interaction_information(
            self,
            decided_atoms,
            conflict_link_cases,
            non_conflict_link_cases,
            non_duplicate_link_cases,
    ):
        """Evaluate interaction information value for each available conflict
        link cases, and returns one link set has maximum information value.

        Args:
            decided_atoms: The source atoms to make new atom.
            conflict_link_cases: Conflicted link tuples list.
            non_conflict_link_cases: Non-conflict links list.
            non_duplicate_link_cases: Non-duplicated links list.
            :param decided_atoms: list[Atom]
            :param conflict_link_cases: list[list[EqualLinkKey]]
            :param non_conflict_link_cases: list[EqualLinkKey]
            :param non_duplicate_link_cases: list[EqualLinkKey]
        Returns:
            A link set has maximum interaction information value.
            :rtype: list[EqualLinkKey]
        """
        result_list = list()

        inheritance_nodes = list()
        for decided_atom in decided_atoms:
            inheritance_nodes += find_inheritance_nodes(self.a, decided_atom)

        # TODO: To prevent freeze during interaction information generation,
        # user can limit the max value of calculation.
        max_repeat_length = self.evaluate_n_gram_limit \
            if self.evaluate_n_gram_limit < self.provider.n_gram \
            else self.provider.n_gram

        log.debug(
            "ConnectConflictInteractionInformation: " +
            "Calculating interaction information " +
            "(Total: " + str(len(conflict_link_cases)) + ")"
        )

        current_ratio = 0
        for i, conflict_link in enumerate(conflict_link_cases):
            current_ratio = self.__print_progress(
                "ConnectConflictInteractionInformation:II:",
                current_ratio, i, len(conflict_link_cases), 25
            )
            merged_links = list()
            merged_links.extend(non_conflict_link_cases)
            merged_links.extend(non_duplicate_link_cases)
            merged_links.extend(conflict_link)

            # Calculate n-gram data in each available link cases.
            # The provider with high n-gram will provides more correct data,
            # but speed will going slower rapidly.
            filtered_merged_links = \
                map(lambda x:
                    eq_link.key_to_link(self.a, x, self.ret[0], x.tv),
                    filter(lambda x:
                           # TODO: Currently connector excludes
                           # an InheritanceLink to get valuable(funny) result.
                           self.a[x.h].out[0] not in inheritance_nodes and
                           # Manually throw away the links have low strength.
                           (x.tv.mean > self.inter_info_strength_above_limit),
                           merged_links
                           )
                    )

            if len(filtered_merged_links) is 0:
                continue

            interaction_information = PyInteractionInformationAtom(). \
                calculate_interaction_information(
                filtered_merged_links,
                self.provider,
                max_repeat_length
            )

            result_list.append({
                "merged_links": merged_links,
                "filtered_merged_links": filtered_merged_links,
                "interaction_information": interaction_information
            })

        if len(result_list) < 1:
            self.last_status = blending_status.EMPTY_RESULT
            return []

        result_list = sorted(
            result_list,
            key=(lambda x: x["interaction_information"]),
            reverse=True
        )

        self.__make_result_log(result_list, reverse=False)
        # self.__make_result_log(result_list, reverse=True)

        return result_list[0]['merged_links']

    def __connect_links(self, best_interaction_information_link):
        """Connect selected links to new blend atom.

        Args:
            best_interaction_information_link: Selected link set.
            :param best_interaction_information_link: list[EqualLinkKey]
        """
        for link in best_interaction_information_link:
            eq_link.key_to_link(self.a, link, self.ret[0], link.tv)

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

    def __connect_best_interaction_information_conflict_links(
            self, decided_atoms, merged_atom
    ):
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

        # 3. Find the target links in nodes related with
        # decided nodes(blend source).
        # -> Get the related nodes by searching InheritanceLink.
        related_node_target_links = find_related_links(
            self.a, decided_atoms, self.inter_info_strength_above_limit
        )

        # 4. Make 2^k available conflict link cases if there exists k conflicts.
        conflict_link_cases = make_conflict_link_cases(conflict_links)

        # 5. Calculate probabilities with target links.
        # -> Create the probabilities from example cases in whole AtomSpace.
        self.__prepare_blended_atoms(merged_atom)
        self.__make_statistics_data_provider(
            conflict_link_cases,
            non_conflict_links,
            non_duplicate_links,
            related_node_target_links
        )
        self.__generate_information_probability(
            related_node_target_links
        )

        # 6. Calculate interaction information for each available
        # conflict link cases.
        best_interaction_information_link = \
            self.__evaluate_interaction_information(
                decided_atoms,
                conflict_link_cases,
                non_conflict_links,
                non_duplicate_links,
            )

        # 7. Choose one case which has largest interaction information
        # and connect to new blend.
        self.__connect_links(best_interaction_information_link)
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
        data_n_gram_limit = BlendConfig().get_str(
            self.a, "connect-data-n-gram-limit", config_base
        )
        evaluate_n_gram_limit = BlendConfig().get_str(
            self.a, "connect-evaluate-n-gram-limit", config_base
        )
        inter_info_strength_above_limit = BlendConfig().get_str(
            self.a, "connect-inter-info-strength-above-limit", config_base
        )

        # Check if given atom_type is valid or not.
        try:
            self.check_type = types.__dict__[check_type_str]
        except KeyError:
            self.check_type = types.Node

        # Check if given threshold value is valid or not.
        self.strength_diff_limit = float(strength_diff_threshold)
        self.confidence_above_limit = float(confidence_above_threshold)
        self.data_n_gram_limit = \
            int(data_n_gram_limit) \
            if data_n_gram_limit.isdigit()\
            else -1
        self.evaluate_n_gram_limit = \
            int(evaluate_n_gram_limit) \
            if evaluate_n_gram_limit.isdigit() \
            else -1
        self.inter_info_strength_above_limit = \
            float(inter_info_strength_above_limit)

        self.__connect_best_interaction_information_conflict_links(
            decided_atoms, merged_atom
        )


    def __print_progress(
            self, msg, current_ratio, current_count, total, step=10
    ):
        # To print(debug) progress of evaluating.
        if current_ratio < current_count:
            current_ratio += total * step * 0.01
            log.debug(
                msg + ": " + str(current_count) + "/" + str(total) +
                " (" + str(100 * current_count / float(total)) + "%)"
            )
        return current_ratio

    def __make_result_log(self, result_list, reverse):
        # To print(debug) interaction information value.
        if reverse:
            result_list = reversed(result_list)

        for i, result in enumerate(result_list):
            name = ""
            # Prints only top 5 results.
            if i < 15:
                for link in result['filtered_merged_links']:
                    for node in link.out:
                        if node.t == types.ConceptNode and node != self.ret[0]:
                            name += node.name + ", "
                log.debug(name + ": " + str(result['interaction_information']))
