# coding=utf-8
__author__ = 'DongMin Kim'

from opencog.atomspace import *


class ConnectSimple:
    """
    If (A) -> (B), and (C) is new blended node,
    purpose of ConnectSimple is link with (A) -> (C).
    (A) is src_node
    (C) is dst_node
    (B) is src_node in ConnectSimple context.
    (B) is dst_node in EqualLinkKey context.

    :type a: opencog.atomspace_details.AtomSpace
    """

    def __init__(self, atomspace):
        self.a = atomspace

    def __str__(self):
        return self.__class__.__name__

    def make_default_config(self):
        """
        default_config = {
        }
        BlConfig().make_default_config(str(self), default_config)
        """
        pass

    def add_new_links(self, src_info_cont, link_set, dst_node):
        # Add new link to target node.
        # TODO: Change to make with proper reason, not make in every blending.
        # 적절한 이유가 있을 때만 연결시켜야 한다.
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

    def modify_exist_links(self, src_info_cont, dst_info_cont, link_set):
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
        # 충돌값 보정을 단순 평균이 아닌 적절한 이유를 가진 값으로 바꿔야 한다.
        new_confidence = (found_c + exist_c) / 2

        tv = TruthValue(new_strength, new_confidence)
        self.a.set_tv(dst_link.h, tv)