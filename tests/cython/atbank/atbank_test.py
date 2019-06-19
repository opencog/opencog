import unittest
from unittest import TestCase

from opencog.type_constructors import *
from opencog.utilities import initialize_opencog, finalize_opencog
from opencog.atbank import AttentionBank, af_bindlink


class AttentionBankTest(TestCase):

    def setUp(self):
        self.atomspace = AtomSpace()
        initialize_opencog(self.atomspace)

    def tearDown(self):
        finalize_opencog()
        del self.atomspace

    def test_attention_bank(self):
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
