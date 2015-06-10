# coding=utf-8
from opencog.type_constructors import types
from opencog.atomspace import Handle

__author__ = 'DongMin Kim'


class EqualLinkKey:
    """
    :type t: opencog.atomspace.types
    :type src_h_list_str: str
    :type dst_pos_in_outgoing: int
    """

    def __init__(self, link_type=None):
        self.t = link_type
        self.src_h_list_str = ''
        self.dst_pos_in_outgoing = -1

    # It is equal if
    # (type, outgoing list, self position in outgoing list)
    # is same.
    def __key(self):
        return self.t, self.src_h_list_str, self.dst_pos_in_outgoing

    def __eq__(self, other):
        if self.__key() == other.__key():
            return True

        return False

    def __hash__(self):
        return hash(self.__key())

    def add_src_h(self, src_node):
        src_node_h_str = str(src_node.h.value())
        self.src_h_list_str += src_node_h_str + ', '

    def get_src_h_list(self):
        ret = []

        src_node_h_int_list = self.src_h_list_str.split(', ')
        # Delete end of string
        src_node_h_int_list.pop()

        for src_node_h_int in src_node_h_int_list:
            ret.append(Handle(int(src_node_h_int)))

        return ret


# TODO: How to check and merge link which has ingoing atoms?
# 흔한 경우는 아니지만 Link가 ingoing atom들을 갖고 있으면 어떡하지?
def get_equal_link_keys(a, original_links, dst_atom):
    """
    :param a: opencog.atomspace_details.AtomSpace
    :param original_links: types.Link
    :param dst_atom: types.Node
    :rtype : list[EqualLinkKey]
    """
    ret = list()
    for link in original_links:
        equal_link_key = EqualLinkKey(link.t)

        xget_link_src = a.xget_outgoing(link.h)
        for i, link_src in enumerate(xget_link_src):
            if link_src.h == dst_atom.h:
                equal_link_key.dst_pos_in_outgoing = i
            else:
                equal_link_key.add_src_h(link_src)

        ret.append(equal_link_key)

    return ret


def find_duplicate_links(a, a_decided_atoms):
    atom_link_pair_list = {}

    for atom in a_decided_atoms:
        original_links = a.get_atoms_by_target_atom(types.Link, atom)
        equal_links = get_equal_link_keys(a, original_links, atom)
        atom_link_pair_list[atom.handle_uuid()] = equal_links

    inverted_links_index = get_inverted_index_value(atom_link_pair_list)

    duplicate_links = []
    non_duplicate_links = []
    for values in inverted_links_index.values():
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
    src_h_list = link_key.get_src_h_list()

    # To avoid self-pointing
    if dst_atom.h in src_h_list:
        return

    src_h_list.insert(link_key.dst_pos_in_outgoing, dst_atom)
    a.add_link(link_key.t, src_h_list, tv)


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
    for values in key_value_pair_list.values():
        all_values.extend(values)

    all_values = set(all_values)
    for value in all_values:
        value_list = []
        for pair_key, pair_value in key_value_pair_list.items():
            if value in pair_value:
                value_list.append(pair_value)
        inverted_index[value] = value_list

    return inverted_index
