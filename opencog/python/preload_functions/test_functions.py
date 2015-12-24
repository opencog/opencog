# 
# Test Functions preloaded into the CogServer

from opencog.atomspace import types, Atom, TruthValue
from opencog.type_constructors import *
from test_helpers.bind_helpers import increment_green, increment_red

def print_arguments(argOne, argTwo):
    print "argOne = ", argOne
    print "argTwo = ", argTwo
    atom = ATOMSPACE.add_link(types.ListLink, [argOne, argTwo])
    print "atom = ", atom
    return atom

def add_link(atom_one, atom_two):
    link_atom = ATOMSPACE.add_link(types.ListLink, [atom_one, atom_two])
    return link_atom

def bogus_tv(atom_one, atom_two):
    return TruthValue(0.6, 0.234)

def stop_go(atom):
    compare_green = ConceptNode("green light")
    compare_red = ConceptNode("red light")
    if atom == compare_green:
        print "got green"
        increment_green()
        return TruthValue(1,1)
    elif atom == compare_red:
        print "got red"
        increment_red()
        return TruthValue(0,1)
    else:
        print "got no match :-("
        assert(false)

    return TruthValue(0,0)
