from utility.generic import DEFAULT_TRUTH_VALUE as tv
from opencog.atomspace import types as t

def inherit(inher_link):
    a, b = inher_link.out
    return ATOMSPACE.add_link(t.InheritanceLink, [a, b], tv)
