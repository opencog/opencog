import random
from opencog.atomspace import TruthValue

__author__ = 'DongMin Kim'


def make_link_all(a, link_type, src_nodes, dst_node, tv=None):
    if tv is None:
        for node in src_nodes:
            a.add_link(link_type, [node, dst_node], rand_tv())
    else:
        for node in src_nodes:
            a.add_link(link_type, [node, dst_node], tv)


def rand_tv():
    strength = random.uniform(0.5, 0.9)
    confidence = random.uniform(0.5, 0.9)
    return TruthValue(strength, confidence)


def make_sti_all(src_nodes, sti):
    for node in src_nodes:
        node.av = {'sti': sti}
