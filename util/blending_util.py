__author__ = 'DongMin Kim'

import random
from opencog.atomspace import TruthValue

BLEND_TARGET_NODE_NAME = 'BlendTarget'
blend_target_link_tv = TruthValue(1.0, 1.0)


def rand_tv():
    s = random.uniform(0.5, 0.9)
    c = random.uniform(0.5, 0.9)
    return TruthValue(s, c)
