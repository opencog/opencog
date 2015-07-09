# coding=utf-8
from opencog.atomspace import Handle

__author__ = 'DongMin Kim'


class EqualLinkKey:
    def __init__(self, link_h=None, link_type=None, truth_value=None):
        self.h = link_h
        self.t = link_type
        self.tv = truth_value
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

    def get_src_list(self, a):
        ret = []

        src_node_h_int_list = self.src_h_list_str.split(', ')
        # Delete end of string
        src_node_h_int_list.pop()

        for src_node_h_int in src_node_h_int_list:
            ret.append(a[Handle(int(src_node_h_int))])

        return ret


# TODO: How to check and merge link which has ingoing atoms?
def link_to_keys(a, original_links, dst_atom):
    ret = list()
    for link in original_links:
        equal_link_key = EqualLinkKey(link.h, link.t, link.tv)

        xget_link_src = a.xget_outgoing(link.h)
        for i, link_src in enumerate(xget_link_src):
            if link_src.h == dst_atom.h:
                equal_link_key.dst_pos_in_outgoing = i
            else:
                equal_link_key.add_src_h(link_src)

        ret.append(equal_link_key)

    return ret


def key_to_link(a, link_key, dst_atom, tv):
    # TODO: Change to make with proper reason, not make in every blending.
    src_list = link_key.get_src_list(a)

    # To avoid self-pointing
    if dst_atom in src_list:
        return

    src_list.insert(link_key.dst_pos_in_outgoing, dst_atom)
    return a.add_link(link_key.t, src_list, tv)
