# coding=utf-8
__author__ = 'DongMin Kim'

import random
from opencog.type_constructors import *
from util_b.general_util import *


def make_link_all(a, link_type, src_nodes, dst_node, tv=None):
    if tv is None:
        for node in src_nodes:
            a.add_link(link_type, [node, dst_node], rand_tv())
    else:
        for node in src_nodes:
            a.add_link(link_type, [node, dst_node], tv)


# Choose atoms which are connected to specific atom.
def get_incoming_nodes(a, target):
    ret = []

    xget_target_link = a.xget_atoms_by_target_atom(types.Link, target)

    for link in xget_target_link:
        xget_target_link_node = a.xget_outgoing(link.h)
        for node in xget_target_link_node:
            if node.h != target.h:
                ret.append(node)

    return ret


def rand_tv():
    strength = random.uniform(0.5, 0.9)
    confidence = random.uniform(0.5, 0.9)
    return TruthValue(strength, confidence)

# For control STI values.
sti_value_dict = {
    'NONE': None,
    'JUST_TARGET': 16,
    'IMPORTANT': 32,
    'VERY_IMPORTANT': 64
}


def make_sti_all(src_nodes, sti):
    for node in src_nodes:
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

    mean_sum = 0

    weighted_strength_sum = 0
    confidence_sum = 0
    link_count = 0

    for atom in atoms:
        weighted_strength_sum += (atom.tv.confidence * atom.tv.mean)
        confidence_sum += atom.tv.confidence
        link_count += 1

    try:
        new_strength = weighted_strength_sum / confidence_sum
    except ZeroDivisionError:
        # This is arithmetic mean, maybe given atoms doesn't have TruthValue.
        for atom in atoms:
            mean_sum += atom.tv.mean
        new_strength = mean_sum / link_count

    # TODO: Currently, confidence value for new blended node is just
    # average of old value.
    # 충돌값 보정을 단순 평균이 아닌 적절한 이유를 가진 값으로 바꿔야 한다.
    new_confidence = confidence_sum / link_count
    return TruthValue(new_strength, new_confidence)
