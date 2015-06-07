# coding=utf-8
from opencog.type_constructors import types
from opencog.atomspace import Handle
from util_b.blending_util import rand_tv
from util_b.general_util import Singleton

__author__ = 'DongMin Kim'


class EqualLinkContainers:
    """
    :type link_list: list[types.Link]
    :type link_dict: dict[EqualLinkKey]
    :type link_set: set
    """
    def __init__(self, link_list=None, link_dict=None, link_set=None):
        self.link_list = link_list
        self.link_dict = link_dict
        self.link_set = link_set

    @property
    def l(self):
        return self.link_list

    @property
    def d(self):
        return self.link_dict

    @property
    def s(self):
        return self.link_set


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


class ConnectUtil(Singleton):
    def __init__(cls):
        super(ConnectUtil, cls).__init__()

    # TODO: How to check and merge link which has ingoing atoms?
    # 흔한 경우는 아니지만 Link가 ingoing atom들을 갖고 있으면 어떡하지?
    def __get_equal_link_dict(cls, a, link_list, dst_node):
        """
        :param a: opencog.atomspace_details.AtomSpace
        :param link_list: types.Link
        :param dst_node: types.Node
        :rtype : dict[EqualLinkKey]
        """
        ret = dict()
        for link in link_list:
            equal_link_key = EqualLinkKey(link.t)

            xget_link_src = a.xget_outgoing(link.h)
            for i, link_src in enumerate(xget_link_src):
                if link_src.h == dst_node.h:
                    equal_link_key.dst_pos_in_outgoing = i
                else:
                    equal_link_key.add_src_h(link_src)

            ret[equal_link_key] = link.h

        return ret

    def make_equal_link_containers(cls, a, dst_node):
        src_link_list = a.get_atoms_by_target_atom(types.Link, dst_node)
        src_link_dict = cls.__get_equal_link_dict(a, src_link_list, dst_node)
        src_link_set = set(src_link_dict.keys())

        return EqualLinkContainers(src_link_list, src_link_dict, src_link_set)
