import random
from opencog.type_constructors import *

__author__ = 'DongMin Kim'


"""
Blending Utils.
"""

# Empty.


"""
General Utils.
"""


class _Singleton(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            # noinspection PyArgumentList
            cls._instances[cls] = \
                super(_Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class Singleton(
    _Singleton(
        'SingletonMeta',
        (object,),
        {}
    )
):
    pass


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
