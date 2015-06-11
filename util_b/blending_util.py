# coding=utf-8
__author__ = 'DongMin Kim'

import random
from opencog.type_constructors import *
from util_b.general_util import *

blend_target_link_tv = TruthValue(1.0, 1.0)

# Singleton class
# From now, this class don't check any validity of atomspace, so be careful!
class BlendTargetCtlForDebug(Singleton):
    def __init__(cls):
        super(BlendTargetCtlForDebug, cls).__init__()

    BLEND_TARGET_NAME = 'BlendTarget'

    a = None
    a_blend_target = None
    link_list_backup = []

    def is_exist(self):
        if self.a is None:
            return False
        elif self.a_blend_target is None:
            return False
        else:
            return True

    def get_blend_target(self):
        if self.is_exist() is False:
            self.make_blend_target()
        return self.a_blend_target

    # DEBUG: Make temporary concept - To define which node is target to blend.
    def make_blend_target(self):
        if self.is_exist() is False:
            self.a_blend_target = \
                self.a.add_node(types.ConceptNode, self.BLEND_TARGET_NAME)

    def delete_blend_target(self):
        if self.is_exist() is False:
            self.make_blend_target()

        l_link_list = self.a.get_atoms_by_target_atom(
            types.Link, self.a_blend_target
        )

        for link in l_link_list:
            self.a.remove(link)

        self.a.remove(self.a_blend_target)
        self.a_blend_target = None

    def backup_debug_link_list(self):
        if self.is_exist() is False:
            self.make_blend_target()

        node_name_list = [
            "BlendTarget",
            "InputSpace0",
            "InputSpace1",
            "GenericSpace"
        ]
        self.link_list_backup = []

        for name in node_name_list:
            dst_node = self.a.get_atoms_by_name(types.Node, name)[0]
            get_target_link = \
                self.a.get_atoms_by_target_atom(types.Link, dst_node)

            for link in get_target_link:
                xget_target_link_node = self.a.xget_outgoing(link.h)
                for src_node in xget_target_link_node:
                    if src_node.h != dst_node.h:
                        self.link_list_backup.append(
                            {
                                'type': link.t,
                                'src_node': src_node,
                                'dst_node': dst_node,
                                'tv': link.tv
                            }
                        )
                self.a.remove(link)

    def restore_debug_link_list(self):
        if self.is_exist() is False:
            self.make_blend_target()

        if self.link_list_backup.__len__() > 0:
            for info in self.link_list_backup:
                self.a.add_link(
                    info['type'],
                    [info['src_node'], info['dst_node']],
                    info['tv']
                )


def make_link_all(a, link_type, src_node_list, dst_node, tv=None):
    if tv is None:
        for node in src_node_list:
            a.add_link(link_type, [node, dst_node], rand_tv())
    else:
        for node in src_node_list:
            a.add_link(link_type, [node, dst_node], tv)


# Choose atoms which are connected to specific atom.
def get_incoming_node_list(a, target):
    ret = []

    xget_target_link = a.xget_atoms_by_target_atom(types.Link, target)

    for link in xget_target_link:
        xget_target_link_node = a.xget_outgoing(link.h)
        for node in xget_target_link_node:
            if node.h != target.h:
                ret.append(node)

    return ret


def rand_tv():
    s = random.uniform(0.5, 0.9)
    c = random.uniform(0.5, 0.9)
    return TruthValue(s, c)

# For control STI values.
sti_value_dict = {
    'NONE': None,
    'JUST_TARGET': 16,
    'IMPORTANT': 32,
    'VERY_IMPORTANT': 64
}


def make_sti_all(a, src_node_list, sti):
    for node in src_node_list:
        node.av = {'sti': sti}


def get_weighted_tv(atoms):
    """
    Make new TruthValue by evaluate weighted average of exist
    link's TruthValue.

    This is implement code of this idea written by Ben Goertzel:
    https://groups.google.com/forum/#!topic/opencog/fa5c4yE8YdU

    :param list(EqualLinkKey) atoms: List of EqualLinkKey which are
    expected to make weighted average TruthValue from theirs.
    :rtype TruthValue: New truth value.
    """
    if len(atoms) < 2:
        raise UserWarning(
            "Weighted TruthValue can't be evaluated with small size."
        )

    weighted_strength_sum = 0
    confidence_sum = 0
    link_count = 0

    for atom in atoms:
        weighted_strength_sum += (atom.tv.confidence * atom.tv.mean)
        confidence_sum += atom.tv.confidence
        link_count += 1

    new_strength = weighted_strength_sum / confidence_sum

    # TODO: Currently, confidence value for new blended node is just
    # average of old value.
    # 충돌값 보정을 단순 평균이 아닌 적절한 이유를 가진 값으로 바꿔야 한다.
    new_confidence = confidence_sum / link_count

    return TruthValue(new_strength, new_confidence)
