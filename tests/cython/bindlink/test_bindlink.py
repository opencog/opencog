from unittest import TestCase

from opencog.atomspace import AtomSpace, TruthValue, Atom, Handle, types
from opencog.bindlink import    stub_bindlink, bindlink, single_bindlink,\
                                crisp_logic_bindlink, pln_bindlink,\
                                validate_bindlink
import opencog.scheme_wrapper as scheme
from opencog.scheme_wrapper import load_scm, scheme_eval, scheme_eval_h

__author__ = 'Curtis Faith'


scheme_preload = [  
                    "opencog/atomspace/core_types.scm",
                    "opencog/scm/utilities.scm" 
                 ]


class BindlinkTest(TestCase):

    bindlink_handle = None

    def setUp(self):
        self.atomspace = AtomSpace()
        scheme.__init__(self.atomspace)
        for scheme_file in scheme_preload:
            load_scm(self.atomspace, scheme_file)
        
        # Define several animals and something of a different type as well
        scheme_animals = \
            '''
            (InheritanceLink (ConceptNode "Frog") (ConceptNode "animal"))
            (InheritanceLink (ConceptNode "Zebra") (ConceptNode "animal"))
            (InheritanceLink (ConceptNode "Deer") (ConceptNode "animal"))
            (InheritanceLink (ConceptNode "Spaceship") (ConceptNode "machine"))
            '''
        scheme_eval_h(self.atomspace, scheme_animals)

        # Define a graph search query
        bind_link_query = \
            '''
            (BindLink
                ;; The variable to be bound
                (VariableNode "$var")
                (ImplicationLink
                    ;; The pattern to be searched for
                    (InheritanceLink
                        (VariableNode "$var")
                        (ConceptNode "animal")
                    )
                    ;; The value to be returned.
                    (VariableNode "$var")
                )
            )
            '''
        self.bindlink_handle = scheme_eval_h(self.atomspace, bind_link_query)
        

    def tearDown(self):
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


    def test_validate_bindlink(self):

        # Remember the starting atomspace size. This test should not
        # change the atomspace.
        starting_size = self.atomspace.size()

        # Run bindlink.
        result = validate_bindlink(self.atomspace, self.bindlink_handle)
        self.assertTrue(result is not None and result.value() > 0)

        # Check the ending atomspace size, it should be the same.
        ending_size = self.atomspace.size()
        self.assertEquals(ending_size, starting_size)

        # This should return the BindLink which has two items in it, i.e.
        # the VariableNode and ImplicationLink.
        atom = self.atomspace[result]
        self.assertEquals(atom.arity, 2)
        self.assertEquals(atom.type, types.BindLink)


    def test_bindlink(self):

        # Remember the starting atomspace size.
        starting_size = self.atomspace.size()

        # Run bindlink.
        result = bindlink(self.atomspace, self.bindlink_handle)
        self.assertTrue(result is not None and result.value() > 0)

        # Check the ending atomspace size, it should have added one ListLink.
        ending_size = self.atomspace.size()
        self.assertEquals(ending_size, starting_size + 1)

        # The ListLink should have three items in it.
        atom = self.atomspace[result]
        self.assertEquals(atom.arity, 3)
        self.assertEquals(atom.type, types.ListLink)


    def test_single_bindlink(self):

        # Remember the starting atomspace size.
        starting_size = self.atomspace.size()

        # Run bindlink.
        result = single_bindlink(self.atomspace, self.bindlink_handle)
        self.assertTrue(result is not None and result.value() > 0)

        # Check the ending atomspace size, it should have added one ListLink.
        ending_size = self.atomspace.size()
        self.assertEquals(ending_size, starting_size + 1)

        # The ListLink should have one item in it.
        atom = self.atomspace[result]
        self.assertEquals(atom.arity, 1)
        self.assertEquals(atom.type, types.ListLink)


    def test_crisp_logic_bindlink(self):

        # Remember the starting atomspace size.
        starting_size = self.atomspace.size()

        # Run bindlink.
        result = crisp_logic_bindlink(self.atomspace, self.bindlink_handle)
        self.assertTrue(result is not None and result.value() > 0)

        # Check the ending atomspace size, it should have added one ListLink.
        ending_size = self.atomspace.size()
        self.assertEquals(ending_size, starting_size + 1)

        # The ListLink should have three items in it.
        atom = self.atomspace[result]
        self.assertEquals(atom.arity, 3)
        self.assertEquals(atom.type, types.ListLink)


    def test_pln_bindlink(self):

        # Remember the starting atomspace size.
        starting_size = self.atomspace.size()

        # Run bindlink.
        result = pln_bindlink(self.atomspace, self.bindlink_handle)
        self.assertTrue(result is not None and result.value() > 0)

        # Check the ending atomspace size, it should have added one ListLink.
        ending_size = self.atomspace.size()
        self.assertEquals(ending_size, starting_size + 1)

        # The ListLink is empty. ??? Should it be.
        atom = self.atomspace[result]
        self.assertEquals(atom.arity, 0)
        self.assertEquals(atom.type, types.ListLink)
