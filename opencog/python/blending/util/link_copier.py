from opencog.atomspace import Handle, TruthValue
from opencog.type_constructors import types

__author__ = 'DongMin Kim'

class LinkCopier:
    def __init__(self, atomspace):
        self.a = atomspace

    # TODO: Translate to english.
    # If (A) -> (B) and (C) is new blended node,
    # purpose of LinkCopier is link with (A) -> (C).
    # (A) is src_node
    # (C) is dst_node
    # (B) is src_node in LinkCopier context.
    # (B) is dst_node in LinkSrcInfo context.

    class LinkSrcInfoContainer:
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

    class LinkSrcInfo:
        def __init__(self, link_type=None, link_src_handle=None):
            self.t = link_type
            self.src_h = link_src_handle

    class LinkInfo:
        def __init__(self, link_h_value=None, link_tv=None):
            self.h = link_h_value
            self.tv = link_tv

    def __get_link_src_info_dict(self, link_list, dst_node):
        ret_dict = dict()
        for link in link_list:
            xget_link_src = self.a.xget_outgoing(link.h)
            for link_src in xget_link_src:
                if link_src.h != dst_node.h:
                    link_src_info = self.LinkSrcInfo(
                        link.t, link_src.h.value()
                    )
                    link_info = self.LinkInfo(
                        link.h, link.tv
                    )
                    ret_dict[link_src_info] = link_info

        return ret_dict

    def __get_link_src_info_set(self, link_dict):
        return set(link_dict.keys())

    def get_link_src_info_containers(self, dst_node):
        src_link_list = self.a.get_atoms_by_target_atom(types.Link, dst_node)
        src_link_dict = self.__get_link_src_info_dict(src_link_list, dst_node)
        src_link_set = self.__get_link_src_info_set(src_link_dict)

        return self.LinkSrcInfoContainer(
            src_link_list, src_link_dict, src_link_set
        )

    def __add_new_links(self, src_info_cont, link_set, dst_node):
        # Add new link to target node.
        for link_src_info in link_set:
            src_node = self.a[Handle(link_src_info.src_h)]
            link_info = src_info_cont.d[link_src_info]
            self.a.add_link(
                link_src_info.t,
                [src_node, dst_node],
                link_info.tv
            )

    def __modify_exist_links(self, src_info_cont, dst_info_cont, link_set):
        # Correct conflict link value in target node.
        for link_src_info in link_set:
            src_link_info = src_info_cont.d[link_src_info]
            dst_link_info = dst_info_cont.d[link_src_info]
            src_link = self.a[Handle(src_link_info.h)]
            dst_link = self.a[Handle(dst_link_info.h)]

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

        dst_link.tv = TruthValue(new_strength, new_confidence)

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
