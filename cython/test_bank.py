from unittest import TestCase

from opencog.atomspace import AtomSpace, TruthValue, Atom
from opencog.atomspace import types, is_a, get_type, get_type_name, create_child_atomspace

from opencog.type_constructors import *
from opencog.utilities import initialize_opencog, finalize_opencog

from time import sleep

class AtomSpaceTest(TestCase):

    def setUp(self):
        self.space = AtomSpace()
        initialize_opencog(self.space)

    def tearDown(self):
        finalize_opencog()
        del self.space

    def test_attention_value(self):
        node = Node("test")

        # check values come back as assigned
        node.sti = 1
        node.lti = 2
        node.vlti = 3
        assert node.sti == 1
        assert node.lti == 2
        assert node.vlti == 3

        # Check increment and decrement for vlti
        node.decrement_vlti()
        assert node.vlti == 2
        node.increment_vlti()
        assert node.vlti == 3

        # Check dictionary setting and getting of av property.
        node.av = {"sti": 4, "lti": 5, "vlti": 6}
        assert node.sti == 4
        assert node.lti == 5
        assert node.vlti == 6
        assert node.av == {"sti": 4, "lti": 5, "vlti": 6}

    def test_get_by_av(self):
        a1 = ConceptNode("test1")
        a2 = ConceptNode("test2")
        a3 = InheritanceLink(a1, a2)
        a4 = ConceptNode("test4")
        a5 = ConceptNode("test5")

        a1.sti = 10
        a2.sti = 5
        a3.sti = 4
        a4.sti = 1

        #ImportanceIndex is Asynchronus give it some time
        sleep(1)

        result = self.space.get_atoms_by_av(4, 10)
        print ("The atoms-by-av result is ", result)
        assert len(result) == 3
        assert set(result) == set([a1, a2, a3])
        assert a4 not in result

        result = self.space.get_atoms_in_attentional_focus()
        assert len(result) == 4
        assert set(result) == set([a1, a2, a3, a4])

    def test_attention_value(self):
        a = Node("test2")

        self.assertEqual(a.av, {'lti': 0, 'sti': 0, 'vlti': False})

        # test set av
        a.av = { "sti": 10, "lti": 1, "vlti": True }
        self.assertEqual(a.av, {'sti': 10, 'lti': 1, 'vlti': True})

    def test_af_bindlink(self):
        atom = af_bindlink(self.atomspace, self.bindlink_atom)
        # The SetLink is empty. ??? Should it be.
        self._check_result_setlink(atom, 0)
