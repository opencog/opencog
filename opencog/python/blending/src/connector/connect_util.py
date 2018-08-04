import itertools
from copy import copy

from opencog.atomspace import TruthValue
from opencog.type_constructors import types
from opencog.logger import log

from blending.src.connector.equal_link_key import link_to_keys
from blending.util.py_cog_execute import PyCogExecute
from blending.util.blending_config import BlendConfig

__author__ = 'DongMin Kim'


def is_conflict_link(
        a, link_0, link_1,
        check_type, strength_diff_limit, confidence_above_limit
):
    """Check the links if conflict or not.

    Args:
        a: An instance of AtomSpace.
        link_0: First link to find conflict or not.
        link_1: Second link to find conflict or not.
        check_type: A link type to check conflict.
        strength_diff_limit: A limit of difference between links strength value.
        confidence_above_limit: A threshold of both links confidence value.
        :param a: opencog.atomspace.AtomSpace
        :param link_0: Link
        :param link_1: Link
        :param check_type: int
        :param strength_diff_limit: float
        :param confidence_above_limit: float
    Returns:
        If two link conflicts to each other, then returns True.
        If not, returns False.
        :rtype : bool
    """
    # TODO: Currently, this method can handle
    # when the number of decided atom is only 2.
    if a[link_0.h].is_a(check_type) is False:
        return False
    if abs(link_0.tv.mean - link_1.tv.mean) < strength_diff_limit:
        return False
    if link_0.tv.confidence < confidence_above_limit:
        return False
    if link_1.tv.confidence < confidence_above_limit:
        return False

    return True


def find_conflict_links(
        a, duplicate_links_list,
        check_type, strength_diff_limit, confidence_above_limit
):
    """Find the conflicted, non-conflicted links from
    duplicated link tuples list.

    Args:
        a: An instance of AtomSpace.
        duplicate_links_list: Duplicated links list.
        check_type: A link type to check conflict.
        strength_diff_limit: A limit of difference between links strength value.
        confidence_above_limit: A threshold of both links confidence value.
        :param a: opencog.atomspace.AtomSpace
        :param duplicate_links_list: list[Link]
        :param check_type: int
        :param strength_diff_limit: float
        :param confidence_above_limit: float
    Returns:
        conflict_links: Conflicted link tuples list.
        non_conflict_links: Non-conflicted links list.
        :rtype : (list[list[EqualLinkKey]], list[EqualLinkKey])
    """
    # TODO: Currently, this method can handle
    # when the number of decided atom is only 2.
    conflict_links = []
    non_conflict_links = []

    for duplicate_links in duplicate_links_list:
        link_0 = duplicate_links[0]
        link_1 = duplicate_links[1]

        if is_conflict_link(
                a, link_0, link_1,
                check_type, strength_diff_limit, confidence_above_limit
        ):
            # TODO: Currently, this method checks if only two atom have exactly
            # same link, except TruthValue. Find the unveiled conflicts
            # via PLN would be great.
            # e.g. Conflict:
            # InheritanceLink(car alive)<0>,
            # InheritanceLink(man alive)<1>
            # Unveiled Conflict:
            # InheritanceLink(car non-living)<1>,
            # InheritanceLink(man alive)<1>
            conflict_links.append(duplicate_links)
        else:
            # array elements are same except original node information.
            link_key_sample = copy(link_0)
            link_key_sample.tv = get_weighted_tv(duplicate_links)
            non_conflict_links.append(link_key_sample)

    return conflict_links, non_conflict_links


def find_duplicate_links(a, decided_atoms):
    """Find the duplicated, non-duplicated links from decided atoms list.

    Args:
        a: An instance of AtomSpace.
        decided_atoms: The source atoms to make new atom.
        :param a: opencog.atomspace.AtomSpace
        :param decided_atoms: list[Atom]
    Returns:
        duplicate_links: Duplicated link tuples list.
        non_duplicate_links: Non-duplicated links list.
        :rtype : (list[list[EqualLinkKey]], list[EqualLinkKey])
    """
    # ex) atom_link_pairs =
    # {
    #   <a1>: [<l1, type=a>, <l2, type=b>, <l3, type=c>]
    #   <a2>: [<l4, type=a>, <l5, type=x>, <l6, type=y>]
    # }
    atom_link_pairs = {}

    for atom in decided_atoms:
        original_links = atom.incoming_by_type(types.Link)
        equal_links = link_to_keys(a, original_links, atom)
        atom_link_pairs[atom.handle_uuid()] = equal_links

    # ex) inverted_links_index =
    # {
    #   <type=a>: [<l1, type=a>, <l4, type=a>]
    #   <type=b>: [<l2, type=b>]
    #   <type=c>: [<l3, type=c>]
    #   <type=x>: [<l5, type=x>]
    #   <type=y>: [<l6, type=y>]
    # }
    inverted_links_index = get_inverted_index_value(atom_link_pairs)

    # ex) duplicate_links =
    # [
    #   [<l1, type=a>, <l4, type=a>]
    # ]

    # ex) non_duplicate_links =
    # {
    #   [<l2, type=b>]
    #   [<l3, type=c>]
    #   [<l5, type=x>]
    #   [<l6, type=y>]
    # }
    duplicate_links = []
    non_duplicate_links = []

    for item in inverted_links_index.itervalues():
        duplicate_links.append(item) if len(item) > 1 \
            else non_duplicate_links.extend(item)

    return duplicate_links, non_duplicate_links


def find_related_links(
        a,
        decided_atoms,
        inter_info_strength_above_limit
):
    """Find the all related links from decided atoms list.

    It estimates that related links are all of links in inherited nodes.

    Args:
        a: An instance of AtomSpace.
        decided_atoms: The source atoms to make new atom.
        inter_info_strength_above_limit: A max value of strength in TruthValue
        during interaction information generation.
        :param a: opencog.atomspace.AtomSpace
        :param decided_atoms: list[Atom]
        :param inter_info_strength_above_limit: float
    Returns:
        related_node_target_links: Target link tuples in related node list.
        :rtype : list[list[EqualLinkKey]]
    """
    # Evaluate inheritance relation,
    # and copy all link in each inheritance node.
    related_node_target_links = list()

    for decided_atom in decided_atoms:
        inheritance_node_set = find_inheritance_nodes(a, decided_atom)
        for related_node in inheritance_node_set:
            # Manually throw away the links have low strength.
            target_links = filter(
                lambda link: (link.tv.mean > inter_info_strength_above_limit),
                related_node.incoming_by_type(types.Link)
            )
            related_node_target_links.append(
                link_to_keys(a, target_links, related_node)
            )

    return related_node_target_links


def find_inheritance_nodes(a, decided_atoms):
    """Find the inheritance nodes from decided atoms list.

    Args:
        a: An instance of AtomSpace.
        decided_atoms: The source atoms to make new atom.
        :param a: opencog.atomspace.AtomSpace
        :param decided_atoms: list[Atom]
    Returns:
        ret: Nodes inherited to decided_atoms.
        :rtype : list[Atom]
    """
    free_var = a.add_node(types.VariableNode, "$")

    inheritance_link = a.add_link(
        types.InheritanceLink, [free_var, decided_atoms]
    )
    """
    GetLink
        InheritanceLink
            VariableNode $
            ConceptNode <Decided Atom>
    """
    get_link = a.add_link(types.GetLink, [inheritance_link])
    """
    SetLink
        ConceptNode <Related node 1>
        ConceptNode <Related node 2>
        ...
        ConceptNode <Related node N>
    """
    inheritance_node_set = PyCogExecute().execute(a, get_link)

    ret = inheritance_node_set.out

    # Delete temp links.
    BlendConfig().exe_factory.clean_up(inheritance_node_set)
    BlendConfig().exe_factory.clean_up(get_link)
    BlendConfig().exe_factory.clean_up(inheritance_link)
    BlendConfig().exe_factory.clean_up(free_var)
    return ret


def make_conflict_link_cases(conflict_links):
    """Make the conflict link cases(count:2^k),
    from conflict link tuples list(count:k).

    Args:
        conflict_links: Conflicted link tuples list.
        :param conflict_links: list[list[EqualLinkKey]]
    Returns:
        conflict_link_cases: All available conflicted link tuples list cases.
        :rtype: list[EqualLinkKey]
    """
    # TODO: Convert to return 'iterator'. Currently it returns 'list' that has
    # all case of link, makes algorithm very slow.

    # Prepare cartesian product iterator.
    # if number of conflict_links is 3, this iterator produces:
    # (0, 0, 0), (0, 0, 1), (0, 1, 0), (0, 1, 1), ... (1, 1, 1)
    cartesian_binary_iterator = \
        itertools.product([0, 1], repeat=len(conflict_links))

    # Insert each viable atoms to return list.
    conflict_link_cases = list()
    for i, viable_case_binary in enumerate(cartesian_binary_iterator):
        viable_links = list()
        for j, selector in enumerate(viable_case_binary):
            viable_links.append(conflict_links[j][selector])
        if len(viable_links) > 0:
            conflict_link_cases.append(viable_links)

    return conflict_link_cases


def get_inverted_index_key(key_value_pairs):
    """Get the key of inverted index.

    Input =
    {
      <a1>: [<l1, type=a>, <l2, type=b>, <l3, type=c>]
      <a2>: [<l4, type=a>, <l5, type=x>, <l6, type=y>]
    }
    Output =
    {
      <type=a>: [<a1>, <a2>]
      <type=b>: [<a1>]
      <type=c>: [<a1>]
      <type=x>: [<a2>]
      <type=y>: [<a2>]

    Args:
        key_value_pairs: A list of key-value pairs.
        :param key_value_pairs: dict[Handle, list[EqualLinkKey]
    Returns:
        inverted_index: A dict includes the key of inverted index.
        :rtype: dict[EqualLinkKey, list[Handle]]
    """
    inverted_index = {}

    all_values = []
    for values in key_value_pairs.itervalues():
        all_values.extend(values)

    all_values = set(all_values)
    for value in all_values:
        keys = []
        for pair_key, pair_value in key_value_pairs.iteritems():
            if value in pair_value:
                keys.append(pair_key)
        inverted_index[value] = keys

    return inverted_index


def get_inverted_index_value(key_value_pairs):
    """Get the value of inverted index.

    ex) Input =
    {
      <a1>: [<l1, type=a>, <l2, type=b>, <l3, type=c>]
      <a2>: [<l4, type=a>, <l5, type=x>, <l6, type=y>]
    }
    ex) output =
    {
      <type=a>: [<l1, type=a>, <l4, type=a>]
      <type=b>: [<l2, type=b>]
      <type=c>: [<l3, type=c>]
      <type=x>: [<l5, type=x>]
      <type=y>: [<l6, type=y>]
    }

    Args:
        key_value_pairs: A list of key-value pairs.
        :param key_value_pairs: dict[Handle, list[EqualLinkKey]
    Returns:
        inverted_index: A dict includes the key of inverted index.
        :rtype: dict[EqualLinkKey, list[EqualLinkKey]]
    """
    inverted_index = {}

    all_values = []
    for key, values in key_value_pairs.iteritems():
        all_values.extend(values)

    all_values = set(all_values)
    for value in all_values:
        values = []
        for pair_key, pair_value in key_value_pairs.iteritems():
            if value in pair_value:
                values.append(pair_value[pair_value.index(value)])
        inverted_index[value] = values

    return inverted_index


def get_weighted_tv(atoms):
    """Calculate the weighted average of TruthValue of atoms list.

    T1 ... Tk = TruthValue in source atoms
    A = new TruthValue

    strength of A = sA
    confidence of A = cA

    sA = revision of T1...Tk = (sT1*cT1 + ... + sTk*cTk) / (cT1 + ... + cTk)
    cA = (cT1 + ... + cTk) / k

    See: https://groups.google.com/forum/#!topic/opencog/fa5c4yE8YdU

    Args:
        atoms: A list of atom to calculate the weighted average of TruthValue.
        :param atoms: list[Atom]
    Returns:
        An weighted average of TruthValue.
        :rtype: TruthValue
    """
    if len(atoms) < 1:
        log.info("Weighted TruthValue can't be evaluated with small size.")
        return TruthValue()
    elif len(atoms) == 1:
        return atoms[0].tv

    mean_sum = 0

    weighted_strength_sum = 0
    confidence_sum = 0
    link_count = 0

    for atom in atoms:
        weighted_strength_sum += (atom.tv.confidence * atom.tv.mean)
        confidence_sum += atom.tv.confidence
        link_count += 1

    if confidence_sum != 0:
        new_strength = weighted_strength_sum / confidence_sum
    else:
        # This is arithmetic mean, maybe given atoms doesn't have TruthValue.
        for atom in atoms:
            mean_sum += atom.tv.mean
        new_strength = mean_sum / link_count

    # TODO: Currently, confidence value for new blended node is just
    # average of old value.
    new_confidence = confidence_sum / link_count
    return TruthValue(new_strength, new_confidence)
