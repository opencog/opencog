from unittest import TestCase
import os

from opencog.atomspace import AtomSpace, TruthValue, Atom, Handle, types
from opencog.bindlink import    stub_bindlink, bindlink, single_bindlink,\
                                crisp_logic_bindlink, pln_bindlink,\
                                satisfaction_link, execute_atom, evaluate_atom
from opencog.utilities import initialize_opencog, finalize_opencog
from opencog.type_constructors import *
from test_helpers.bind_helpers import green_count, red_count, initialize_counts, increment_green, increment_red

__author__ = 'Curtis Faith'


class BindlinkTest(TestCase):

    bindlink_handle = None

    def setUp(self):
        self.atomspace = AtomSpace()
        print "setUp - atomspace = ", self.atomspace

        # Get the config file name in a manner not dependent on the
        # starting working directory.
        full_path = os.path.realpath(__file__)
        config_file_name = os.path.dirname(full_path) + "/bindlink_test.conf" 

        # Initialize Python
        initialize_opencog(self.atomspace, config_file_name)
        
        # Define several animals and something of a different type as well
        InheritanceLink( ConceptNode("Frog"),       ConceptNode("animal"))
        InheritanceLink( ConceptNode("Zebra"),      ConceptNode("animal"))
        InheritanceLink( ConceptNode("Deer"),       ConceptNode("animal"))
        InheritanceLink( ConceptNode("Spaceship"),  ConceptNode("machine"))

        # Define a graph search query
        self.bindlink_handle =  \
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
                # bindlink needs a handle
                ).h
    
    def tearDown(self):
        finalize_opencog()
        del self.atomspace

    def test_stub_bindlink(self):

        # Remember the starting atomspace size. This test should not
        # change the atomspace.
        starting_size = self.atomspace.size()

        # Run bindlink.
        result = stub_bindlink(self.atomspace, self.bindlink_handle)
        self.assertTrue(result is not None and result.value() > 0)

        # Check the ending atomspace size, it should be the same.
        ending_size = self.atomspace.size()
        self.assertEquals(ending_size, starting_size)


    def test_bindlink(self):

        # Remember the starting atomspace size.
        starting_size = self.atomspace.size()

        # Run bindlink.
        result = bindlink(self.atomspace, self.bindlink_handle)
        self.assertTrue(result is not None and result.value() > 0)

        # Check the ending atomspace size, it should have added one SetLink.
        ending_size = self.atomspace.size()
        self.assertEquals(ending_size, starting_size + 1)

        # The SetLink should have three items in it.
        atom = self.atomspace[result]
        self.assertEquals(atom.arity, 3)
        self.assertEquals(atom.type, types.SetLink)


    def test_single_bindlink(self):

        # Remember the starting atomspace size.
        starting_size = self.atomspace.size()

        # Run bindlink.
        result = single_bindlink(self.atomspace, self.bindlink_handle)
        self.assertTrue(result is not None and result.value() > 0)

        # Check the ending atomspace size, it should have added one SetLink.
        ending_size = self.atomspace.size()
        self.assertEquals(ending_size, starting_size + 1)

        # The SetLink should have one item in it.
        atom = self.atomspace[result]
        self.assertEquals(atom.arity, 1)
        self.assertEquals(atom.type, types.SetLink)


    def test_crisp_logic_bindlink(self):

        # Remember the starting atomspace size.
        starting_size = self.atomspace.size()

        # Run bindlink.
        result = crisp_logic_bindlink(self.atomspace, self.bindlink_handle)
        self.assertTrue(result is not None and result.value() > 0)

        # Check the ending atomspace size, it should have added one SetLink.
        ending_size = self.atomspace.size()
        self.assertEquals(ending_size, starting_size + 1)

        # The SetLink should have three items in it.
        atom = self.atomspace[result]
        self.assertEquals(atom.arity, 3)
        self.assertEquals(atom.type, types.SetLink)


    def test_pln_bindlink(self):

        # Remember the starting atomspace size.
        starting_size = self.atomspace.size()

        # Run bindlink.
        result = pln_bindlink(self.atomspace, self.bindlink_handle)
        self.assertTrue(result is not None and result.value() > 0)

        # Check the ending atomspace size, it should have added one SetLink.
        ending_size = self.atomspace.size()
        self.assertEquals(ending_size, starting_size + 1)

        # The SetLink is empty. ??? Should it be.
        atom = self.atomspace[result]
        self.assertEquals(atom.arity, 0)
        self.assertEquals(atom.type, types.SetLink)

    def test_satisfy(self):
        satisfaction_handle = SatisfactionLink(
            VariableList(),  # no variables
            SequentialAndLink(
                EvaluationLink(
                    GroundedPredicateNode("py: test_functions.stop_go"),
                    ListLink(
                        ConceptNode("green light")
                    )
                ),
                EvaluationLink(
                    GroundedPredicateNode("py: test_functions.stop_go"),
                    ListLink(
                        ConceptNode("green light")
                    )
                ),
                EvaluationLink(
                    GroundedPredicateNode("py: test_functions.stop_go"),
                    ListLink(
                        ConceptNode("red light")
                    )
                ),
                EvaluationLink(
                    GroundedPredicateNode("py: test_functions.stop_go"),
                    ListLink(
                        ConceptNode("traffic ticket")
                    )
                )
            )
        ).h
 
        result = satisfaction_link(self.atomspace, satisfaction_handle)
        self.assertTrue(result is not None and result.mean <= 0.5)
        self.assertEquals(green_count(), 2)
        self.assertEquals(red_count(), 1)

    def test_execute_atom(self):
        result = execute_atom(self.atomspace, 
                ExecutionOutputLink( 
                    GroundedSchemaNode("py: test_functions.add_link"),
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
        self.assertEquals(result, list_link)

    def test_evaluate_atom(self):
        result = evaluate_atom(self.atomspace,
                EvaluationLink( 
                    GroundedPredicateNode("py: test_functions.bogus_tv"),
                    ListLink(
                        ConceptNode("one"),
                        ConceptNode("two") 
                    )
                )
            )
        self.assertEquals(result, TruthValue(0.6, 0.234))
