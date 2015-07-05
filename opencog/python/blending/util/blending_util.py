# coding=utf-8
__author__ = 'DongMin Kim'

from opencog.type_constructors import *


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
    new_confidence = confidence_sum / link_count
    return TruthValue(new_strength, new_confidence)
