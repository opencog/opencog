# coding=utf-8
from opencog.type_constructors import types
from blender_b.connector.equal_link_key import *

__author__ = 'DongMin Kim'


def find_duplicate_links(a, a_decided_atoms):
    atom_link_pair_list = {}

    for atom in a_decided_atoms:
        original_links = a.get_atoms_by_target_atom(types.Link, atom)
        equal_links = get_equal_link_keys(a, original_links, atom)
        atom_link_pair_list[atom.handle_uuid()] = equal_links

    inverted_links_index = get_inverted_index_value(atom_link_pair_list)

    duplicate_links = []
    non_duplicate_links = []
    for key, values in inverted_links_index.items():
        if len(values) > 1:
            duplicate_links.append(values)
        else:
            non_duplicate_links.append(values)

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


def get_inverted_index_key(key_value_pair_list):
    inverted_index = {}

    all_values = []
    for values in key_value_pair_list.values():
        all_values.extend(values)

    all_values = set(all_values)
    for value in all_values:
        key_list = []
        for pair_key, pair_value in key_value_pair_list.items():
            if value in pair_value:
                key_list.append(pair_key)
        inverted_index[value] = key_list

    return inverted_index


def get_inverted_index_value(key_value_pair_list):
    inverted_index = {}

    all_values = []
    for key, values in key_value_pair_list.items():
        all_values.extend(values)

    all_values = set(all_values)
    for value in all_values:
        value_list = []
        for pair_key, pair_value in key_value_pair_list.items():
            if value in pair_value:
                value_list.append(pair_value[pair_value.index(value)])
        inverted_index[value] = value_list

    return inverted_index
