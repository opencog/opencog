import unittest
from unittest import TestCase

from opencog.type_constructors import *
from opencog.utilities import initialize_opencog, finalize_opencog
from opencog.bank import AttentionBank, af_bindlink


class AttentionBankTest(TestCase):

    def setUp(self):
        self.atomspace = AtomSpace()
        initialize_opencog(self.atomspace)

    def tearDown(self):
        finalize_opencog()
        del self.atomspace

    def test_attention_value(self):
        attention_bank = AttentionBank(self.atomspace)

        node = ConceptNode("Socrates")

        attention_bank.set_sti(node, 2.0)
        self.assertEqual(2.0, attention_bank.get_sti(node))

        attention_bank.set_lti(node, 3.0)
        self.assertEqual(3.0, attention_bank.get_lti(node))

        self.assertEqual(0.0, attention_bank.get_vlti(node))

        attention_bank.inc_vlti(node)
        self.assertEqual(1.0, attention_bank.get_vlti(node))

        attention_bank.inc_vlti(node)
        self.assertEqual(2.0, attention_bank.get_vlti(node))

        attention_bank.dec_vlti(node)
        self.assertEqual(1.0, attention_bank.get_vlti(node))

    def test_get_handles_by_av(self):
        attention_bank = AttentionBank(self.atomspace)

        node1 = ConceptNode("Socrates")
        node2 = ConceptNode("Einstein")
        attention_bank.set_sti(node1, 2.0)
        attention_bank.set_sti(node2, 3.0)

        atoms = attention_bank.get_atoms_by_av(1.0, 4.0)
        self.assertEqual(2, len(atoms))
        self.assertEqual(set([node1, node2]), atoms)

        atoms = attention_bank.get_atoms_by_av(1.0, 2.5)
        self.assertEqual(1, len(atoms))
        self.assertEqual(set([node1]), atoms)

        atoms = attention_bank.get_atoms_by_av(2.5, 3.5)
        self.assertEqual(1, len(atoms))
        self.assertEqual(set([node2]), atoms)

        atoms = attention_bank.get_atoms_by_av(4, 5)
        self.assertEqual(0, len(atoms))

    def test_get_handles_by_av_different_as1(self):
        attention_bank = AttentionBank(self.atomspace)

        node1 = ConceptNode("node1")
        attention_bank.set_sti(node1, 2.0)

        atoms = attention_bank.get_atoms_by_av(1.0, 4.0)
        self.assertEqual(1, len(atoms))
        self.assertEqual(set([node1]), atoms)

    def test_get_handles_by_av_different_as2(self):
        attention_bank = AttentionBank(self.atomspace)

        node2 = ConceptNode("node2")
        attention_bank.set_sti(node2, 2.0)

        atoms = attention_bank.get_atoms_by_av(1.0, 4.0)
        self.assertEqual(1, len(atoms))
        self.assertEqual(set([node2]), atoms)

    def test_atoms_in_attentional_focus(self):
        attention_bank = AttentionBank(self.atomspace)

        node1 = ConceptNode("node1")
        node2 = ConceptNode("node2")
        node3 = ConceptNode("node3")
        attention_bank.set_sti(node1, 1.0)
        attention_bank.set_sti(node3, 3.0)

        atoms = attention_bank.get_atoms_in_attentional_focus()
        self.assertEqual(2, len(atoms))
        self.assertEqual(set([node1, node3]), atoms)

    def test_attention_bind_link(self):
        attention_bank = AttentionBank(self.atomspace)

        socrates = ConceptNode("Socrates")
        einstein = ConceptNode("Einstein")
        peirce = ConceptNode("Peirce")
        man = ConceptNode("man")

        attention_bank.set_sti(socrates, 1)
        attention_bank.set_lti(socrates, 2)

        link1 = InheritanceLink(socrates, man)
        link2 = InheritanceLink(einstein, man)
        link3 = InheritanceLink(peirce, man)

        attention_bank.set_av(link2, 35, 10)

        bind_link = BindLink(
            VariableNode("$X"),
            InheritanceLink(VariableNode("$X"), man),
            VariableNode("$X")
        )

        res = af_bindlink(self.atomspace, bind_link)

        self.assertEqual(1, len(res.out))
        self.assertEqual(einstein, res.out[0])


if __name__ == '__main__':
    unittest.main()
