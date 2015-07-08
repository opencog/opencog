from opencog.atomspace import Handle, TruthValue
from opencog.type_constructors import types

__author__ = 'DongMin Kim'

class LinkCopier:
    def __init__(self, atomspace):
        self.a = atomspace

    # TODO: Translate to english.
    # If (A) -> (B), and (C) is new blended node,
    # purpose of LinkCopier is link with (A) -> (C).
    # (A) is src_node
    # (C) is dst_node
    # (B) is src_node in LinkCopier context.
    # (B) is dst_node in EqualLinkKey context.

    class EqualLinkContainer:
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
    def __get_equal_link_dict(self, link_list, dst_node):
        ret = dict()
        for link in link_list:
            equal_link_key = self.EqualLinkKey(link.t)

            xget_link_src = self.a.xget_outgoing(link.h)
            for i, link_src in enumerate(xget_link_src):
                if link_src.h == dst_node.h:
                    equal_link_key.dst_pos_in_outgoing = i
                else:
                    equal_link_key.add_src_h(link_src)

            ret[equal_link_key] = link.h

        return ret

    def get_link_src_info_containers(self, dst_node):
        src_link_list = self.a.get_atoms_by_target_atom(types.Link, dst_node)
        src_link_dict = self.__get_equal_link_dict(src_link_list, dst_node)
        src_link_set = set(src_link_dict.keys())

        return self.EqualLinkContainer(src_link_list, src_link_dict,
                                       src_link_set)

    def __add_new_links(self, src_info_cont, link_set, dst_node):
        # Add new link to target node.
        for link_key in link_set:
            src_h_list = link_key.get_src_h_list()

            if dst_node.h in src_h_list:
                continue

            src_h_list.insert(link_key.dst_pos_in_outgoing, dst_node)
            src_link = self.a[src_info_cont.d[link_key]]
            self.a.add_link(
                src_link.t,
                src_h_list,
                src_link.tv,
            )

    def __modify_exist_links(self, src_info_cont, dst_info_cont, link_set):
        # Correct conflict link value in target node.
        for link_key in link_set:
            src_link = self.a[src_info_cont.d[link_key]]
            dst_link = self.a[dst_info_cont.d[link_key]]

            # Correct conflict link value in target node.
            self.__correct_strength_of_links(src_link, dst_link)

    def __correct_strength_of_links(self, src_link, dst_link):
        found_s = src_link.tv.mean
        found_c = src_link.tv.confidence
        exist_s = dst_link.tv.mean
        exist_c = dst_link.tv.confidence

        # sC = (cA sA + cB sB) / (cA + cB)
        # https://groups.google.com/forum/#!topic/opencog/fa5c4yE8YdU
        new_strength = \
            ((found_c * found_s) + (exist_c * exist_s)) \
            / (found_c + exist_c)

        # TODO: Currently, conflicting confidence value for new blended node
        # is just average of old value.
        new_confidence = (found_c + exist_c) / 2

        tv = TruthValue(new_strength, new_confidence)
        self.a.set_tv(dst_link.h, tv)

    def copy_all_link_to_new_node(self, src_node_list, dst_node):
        # TODO: Optimize dst_info_container update period.
        # It should be move to out of src_node_list loop.
        for src_node in src_node_list:
            src_info_cont = self.get_link_src_info_containers(src_node)
            dst_info_cont = self.get_link_src_info_containers(dst_node)

            exclusive_link_set = src_info_cont.s - dst_info_cont.s
            non_exclusive_link_set = src_info_cont.s & dst_info_cont.s

            self.__add_new_links(
                src_info_cont, exclusive_link_set, dst_node
            )
            self.__modify_exist_links(
                src_info_cont, dst_info_cont, non_exclusive_link_set
            )
