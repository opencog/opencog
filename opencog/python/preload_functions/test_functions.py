# 
# Test Functions preloaded into the CogServer

from opencog.atomspace import types, Atom, TruthValue

def print_arguments(argOne, argTwo):
    print "argOne = ", argOne
    print "argTwo = ", argTwo
    atom = ATOMSPACE.add_link(types.ListLink, [argOne, argTwo])
    print "atom = ", atom
    return atom

def add_link(atom_one, atom_two):
    print "atom_one = ", atom_one
    print "atom_two = ", atom_two
    link_atom = ATOMSPACE.add_link(types.ListLink, [atom_one, atom_two])
    print "link_atom = ", link_atom
    return link_atom

def bogus_tv(atom_one, atom_two) :
    return TruthValue(0.6, 0.234)
