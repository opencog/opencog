import unittest
from unittest import TestCase

import time
from opencog.type_constructors import *
from opencog.utilities import pop_default_atomspace
from opencog.scheme_wrapper import scheme_eval
from opencog.execute import execute_atom
from opencog.openpsi import *

import __main__


# Called by OpenPsi Action
def eat_apple(apple):
    # Mark apple as handled
    InheritanceLink(apple, ConceptNode("handled"))
    return ConceptNode("finished")


__main__.eat_apple = eat_apple


class OpenPsiTest(TestCase):

    @classmethod
    def setUpClass(cls):
        global atomspace
        atomspace = AtomSpace()
        set_default_atomspace(atomspace)

    @classmethod
    def tearDownClass(cls):
        pop_default_atomspace()
        global atomspace
        del atomspace

    def test_create_rule(self):
        openpsi = OpenPsi(atomspace)

        goal = openpsi.create_goal("goal")

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

        component = openpsi.create_component("test-component")

        rule = openpsi.add_rule(context, action, goal, TruthValue(1.0, 1.0), component)

        self.assertFalse(openpsi.is_rule(goal))
        self.assertTrue(openpsi.is_rule(rule.get_rule_atom()))
        self.assertEqual(ConceptNode("goal"), rule.get_goal())

        categories = openpsi.get_categories()
        print("duuuude its cat=", categories)
        self.assertEqual(2, len(categories))
        self.assertTrue(component in categories)
        self.assertFalse(ConceptNode("new-category") in categories)

        new_category = openpsi.add_category(ConceptNode("new-category"))
        self.assertEqual(ConceptNode("new-category"), new_category)
        categories = openpsi.get_categories()
        self.assertEqual(3, len(categories))
        self.assertTrue(new_category in categories)

        self.assertEqual(context, rule.get_context())
        self.assertEqual(action, rule.get_action())

    def test_run_openpsi(self):
        openpsi = OpenPsi(atomspace)

        goal = openpsi.create_goal("goal-run")

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

        component = openpsi.create_component("test-component-run")

        openpsi.add_rule(context, action, goal, TruthValue(1.0, 1.0), component)

        openpsi.run(component)

        # Apples are handled by OpenPsi loop
        InheritanceLink(ConceptNode("apple-1"), ConceptNode("apple"))
        InheritanceLink(ConceptNode("apple-2"), ConceptNode("apple"))

        delay = 5
        time.sleep(delay)
        openpsi.halt(component)

        handled_apples = GetLink(
            InheritanceLink(
                VariableNode("$APPLE"),
                ConceptNode("handled")))

        result_set = execute_atom(atomspace, handled_apples)
        result1 = result_set.out[0]
        result2 = result_set.out[1]
        self.assertEqual(result1, ConceptNode("apple-1"))
        self.assertEqual(result2, ConceptNode("apple-2"))


if __name__ == '__main__':
    unittest.main()
