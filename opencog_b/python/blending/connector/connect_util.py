# coding=utf-8
from opencog.type_constructors import types
from opencog_b.python.blending.connector.equal_link_key import \
    get_equal_link_keys

__author__ = 'DongMin Kim'


def find_duplicate_links(a, decided_atoms):
    # ex) atom_link_pairs =
    # {
    #   <a1>: [<l1, type=a>, <l2, type=b>, <l3, type=c>]
    #   <a2>: [<l4, type=a>, <l5, type=x>, <l6, type=y>]
    # }
    atom_link_pairs = {}

    for atom in decided_atoms:
        original_links = a.get_atoms_by_target_atom(types.Link, atom)
        equal_links = get_equal_link_keys(a, original_links, atom)
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


def make_link_from_equal_link_key(a, link_key, dst_atom, tv):
    """
    Add new link to target node.

    :param AtomSpace a: An atomspace.
    :param EqualLinkKey link_key: EqualLinkKey which has information of new
    link.
    :param types.Atom dst_atom: Destination of connecting new link.
    :param TruthValue tv: New link's truth value.
    """
    # TODO: Change to make with proper reason, not make in every blending.
    # 적절한 이유가 있을 때만 연결시켜야 한다.
    src_list = link_key.get_src_list(a)

    # To avoid self-pointing
    if dst_atom in src_list:
        return

    src_list.insert(link_key.dst_pos_in_outgoing, dst_atom)
    a.add_link(link_key.t, src_list, tv)


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
