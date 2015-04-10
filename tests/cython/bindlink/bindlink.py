from unittest import TestCase
import os

from opencog.atomspace import AtomSpace, TruthValue, Atom, Handle, types
from opencog.bindlink import    stub_bindlink, bindlink, single_bindlink,\
                                crisp_logic_bindlink, pln_bindlink,\
                                execute_atom, evaluate_atom
from opencog.utilities import initialize_opencog, finalize_opencog
from opencog.type_constructors import *

__author__ = 'Curtis Faith'

def assert_equals(object_one, object_two):
    if (object_one != object_two):
        print "assert_equals FAILED: one ", object_one, ", two ", object_two

def assert_true(test):
    if (not test):
        print "assert_true FAILED: test ", test

def add_link(atom_one, atom_two):
    return ListLink(atom_one, atom_two)

atomspace = AtomSpace()

# Get the config file name in a manner not dependent on the
# starting working directory.
full_path = os.path.realpath(__file__)
config_file_name = os.path.dirname(full_path) + "/bindlink_test.conf"
print config_file_name

# Initialize Python
initialize_opencog(atomspace, config_file_name)

# Define several animals and something of a different type as well
InheritanceLink( ConceptNode("Frog"),       ConceptNode("animal"))
InheritanceLink( ConceptNode("Zebra"),      ConceptNode("animal"))
InheritanceLink( ConceptNode("Deer"),       ConceptNode("animal"))
InheritanceLink( ConceptNode("Spaceship"),  ConceptNode("machine"))

# Define a graph search query
bindlink_handle =  \
        BindLink(
            # The variable node to be grounded.
            VariableNode("$var"),

            # The pattern to be grounded.
            ImplicationLink(
                InheritanceLink(
                    VariableNode("$var"),
                    ConceptNode("animal")
                ),

                # The grounding to be returned.
                VariableNode("$var")
            )

        # Bindlink needs a handle, not an atom.
        ).h
    
# Remember the starting atomspace size. This test should not
# change the atomspace.
starting_size = atomspace.size()

# Run bindlink.
result = stub_bindlink(atomspace, bindlink_handle)
assert_true(result is not None and result.value() > 0)

# Check the ending atomspace size, it should be the same.
ending_size = atomspace.size()
assert_equals(ending_size, starting_size)

# Remember the starting atomspace size.
starting_size = atomspace.size()

# Run bindlink.
result = bindlink(atomspace, bindlink_handle)
assert_true(result is not None and result.value() > 0)

# Check the ending atomspace size, it should have added one SetLink.
ending_size = atomspace.size()
assert_equals(ending_size, starting_size + 1)

# The SetLink should have three items in it.
atom = atomspace[result]
assert_equals(atom.arity, 3)
assert_equals(atom.type, types.SetLink)

result = execute_atom(atomspace, 
        ExecutionOutputLink( 
            GroundedSchemaNode("py: add_link"),
            ListLink(
                ConceptNode("one"),
                ConceptNode("two") 
            )
        )
    )
list_link = ListLink(
        ConceptNode("one"),
        ConceptNode("two")
    )
assert_equals(result, list_link)

finalize_opencog()
del atomspace

