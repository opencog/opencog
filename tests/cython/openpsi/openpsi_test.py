import unittest
from unittest import TestCase

import time
from opencog.type_constructors import *
from opencog.utilities import initialize_opencog, finalize_opencog
from opencog.scheme_wrapper import scheme_eval
from opencog.bindlink import execute_atom
from opencog.openpsi import *


# Called by OpenPsi Action
def eat_apple(apple):
    # Mark apple as handled
    InheritanceLink(apple, ConceptNode("handled"))
    return ConceptNode("finished")


class OpenPsiTest(TestCase):

    def setUp(self):
        self.atomspace = AtomSpace()
        initialize_opencog(self.atomspace)

    def tearDown(self):
        finalize_opencog()
        del self.atomspace

    def testopenpsi(self):
        scheme_eval(self.atomspace, "(use-modules (opencog) (opencog exec) (opencog openpsi))")

        openpsi = OpenPsi(self.atomspace)

        goal = ConceptNode("goal")

        context = [
            InheritanceLink(
                VariableNode("$APPLE"),
                ConceptNode("apple")),
            AbsentLink(
                InheritanceLink(
                    VariableNode("$APPLE"),
                    ConceptNode("handled")))
        ]

        action = ExecutionOutputLink(
            GroundedSchemaNode("py: eat_apple"),
            ListLink(
                VariableNode("$APPLE")))

        # Call 'psi-component' from scheme to create the psi component loop
        scheme_eval(self.atomspace, '(define component (psi-component "test-component"))')

        component = ConceptNode("test-component")

        rule = openpsi.add_rule(context, action, goal, TruthValue(1.0, 1.0), component)

        self.assertFalse(openpsi.is_rule(goal))
        self.assertTrue(openpsi.is_rule(rule.get_rule_atom()))
        self.assertEqual(ConceptNode("goal"), rule.get_goal())

        categories = openpsi.get_categories()
        self.assertEqual(2, len(categories))
        self.assertTrue(component in categories)
        self.assertFalse(ConceptNode("new-category") in categories)

        new_category = openpsi.add_category(ConceptNode("new-category"))
        self.assertEqual(ConceptNode("new-category"), new_category)
        categories = openpsi.get_categories()
        self.assertEqual(3, len(categories))
        self.assertTrue(new_category in categories)

        self.assertEqual(context, rule.get_context())

        scheme_eval(self.atomspace, "(psi-run component)")

        # Apples are handled by OpenPsi loop
        InheritanceLink(ConceptNode("apple-1"), ConceptNode("apple"))
        InheritanceLink(ConceptNode("apple-2"), ConceptNode("apple"))

        delay = 0.02
        time.sleep(delay)
        scheme_eval(self.atomspace, "(psi-halt component)")

        handled_apples = GetLink(
            InheritanceLink(
                VariableNode("$APPLE"),
                ConceptNode("handled")))

        result_set = execute_atom(self.atomspace, handled_apples)
        result1 = result_set.out[0]
        result2 = result_set.out[1]
        self.assertEqual(result1, ConceptNode("apple-1"))
        self.assertEqual(result2, ConceptNode("apple-2"))


if __name__ == '__main__':
    unittest.main()
