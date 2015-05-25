__author__ = 'DongMin Kim'

import random
from opencog.atomspace import TruthValue

BLEND_TARGET_NODE_NAME = 'BlendTarget'
blend_target_link_tv = TruthValue(1.0, 1.0)


def rand_tv():
    s = random.uniform(0.5, 0.9)
    c = random.uniform(0.5, 0.9)
    return TruthValue(s, c)


def make_link_all(atomspace, link_type, src_node_list, dst_node, tv=None):
    if tv is None:
        for node in src_node_list:
            atomspace.add_link(link_type, [node, dst_node], rand_tv())
    else:
        for node in src_node_list:
            atomspace.add_link(link_type, [node, dst_node], tv)
