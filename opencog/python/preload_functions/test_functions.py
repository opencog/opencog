# 
# Test Functions preloaded into the CogServer

from opencog.atomspace import types

def print_arguments(argOne, argTwo):
    print "argOne = ", argOne
    print "argTwo = ", argTwo
    atom = ATOMSPACE.add_link(types.ListLink, [argOne, argTwo])
    print "atom = ", atom
    return atom
