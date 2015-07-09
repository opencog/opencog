from opencog.atomspace import TruthValue
from opencog.type_constructors import types
from opencog.logger import log

from blending.src.connector.equal_link_key import link_to_keys

__author__ = 'DongMin Kim'


def find_conflict_links(
        a,
        duplicate_links_list,
        check_type, strength_diff_limit, confidence_above_limit
):
    conflict_links = []
    non_conflict_links = []

    for duplicate_links in duplicate_links_list:
        link_0 = duplicate_links[0]
        link_1 = duplicate_links[1]

        if not a[link_0.h].is_a(check_type):
            non_conflict_links.append(duplicate_links)

        if abs(link_0.tv.mean - link_1.tv.mean) > strength_diff_limit:
            if link_0.tv.confidence > confidence_above_limit:
                if link_1.tv.confidence > confidence_above_limit:
                    conflict_links.append(duplicate_links)

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
        non_conflict_links.append(duplicate_links)

    return conflict_links, non_conflict_links


def find_duplicate_links(a, decided_atoms):
    # ex) atom_link_pairs =
    # {
    #   <a1>: [<l1, type=a>, <l2, type=b>, <l3, type=c>]
    #   <a2>: [<l4, type=a>, <l5, type=x>, <l6, type=y>]
    # }
    atom_link_pairs = {}

    for atom in decided_atoms:
        original_links = a.get_atoms_by_target_atom(types.Link, atom)
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
            else non_duplicate_links.append(item)

    return duplicate_links, non_duplicate_links


def get_inverted_index_key(key_value_pairs):
    # ex) input =
    # {
    #   <a1>: [<l1, type=a>, <l2, type=b>, <l3, type=c>]
    #   <a2>: [<l4, type=a>, <l5, type=x>, <l6, type=y>]
    # }
    # ex) output =
    # {
    #   <type=a>: [<a1>, <a2>]
    #   <type=b>: [<a1>]
    #   <type=c>: [<a1>]
    #   <type=x>: [<a2>]
    #   <type=y>: [<a2>]
    # }
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
    # ex) input =
    # {
    #   <a1>: [<l1, type=a>, <l2, type=b>, <l3, type=c>]
    #   <a2>: [<l4, type=a>, <l5, type=x>, <l6, type=y>]
    # }
    # ex) output =
    # {
    #   <type=a>: [<l1, type=a>, <l4, type=a>]
    #   <type=b>: [<l2, type=b>]
    #   <type=c>: [<l3, type=c>]
    #   <type=x>: [<l5, type=x>]
    #   <type=y>: [<l6, type=y>]
    # }
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
