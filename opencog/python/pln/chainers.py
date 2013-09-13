__author__ = 'ramin'

from pln.rules.rules import Rule
import utility.tree as tree

from opencog.atomspace import types

import random

# @todo opencog/atomspace/AttentionBank.h defines a getAttentionalFocusBoundary method, which should eventually be used instead of this
def get_attentional_focus(atomspace, attentional_focus_boundary=0):
    nodes = atomspace.get_atoms_by_type(types.Atom)
    attentional_focus = []
    for node in nodes:
        if node.av['sti'] > attentional_focus_boundary:
            attentional_focus.append(node)
    return attentional_focus


class AbstractChainer(object):
    def add_rule(self, rule):
        assert isinstance(rule, Rule)
        assert type(rule) != Rule

        self.rules.append(rule)

    def _select_one_matching(self, template):
        attentional_focus = get_attentional_focus(self.atomspace)

        matching_atoms = tree.find(template, attentional_focus)

        if len(matching_atoms) == 0:
            return None
        else:
            return self._selectOne(matching_atoms, self.atomspace)

    def _selectOne(self, links, atomspace):
        assert type(links[0]) != tree.Tree

        max = sum([link.av['sti'] for link in links])
        pick = random.uniform(0, max)
        current = 0
        for link in links:
            current += link.av['sti']
            if current >= pick:
                return link

    def _select_rule(self):
        return random.choice(self.rules)

class Chainer(AbstractChainer):
    def __init__(self, atomspace):
        self.atomspace = atomspace
        self.rules = []

    def _apply_forward(self, rule):
        # randomly choose suitable atoms for this rule's inputs
            # choose a random atom matching the first input to the rule
            # choose a random atom matching the second input to the rule, compatible with the first input
            # etc
            # it can fail if there are no compatible atoms
        # if all inputs are found, then
            # apply the rule and create the output
            # give it an appropriate TV (using the formula and possibly revision)
            # give it an STI boost
            # record this inference in the InferenceHistoryRepository

        generic_inputs = tree.standardize_apart(rule.inputs)
        specific_inputs = []
        specific_outputs = []
        self._find_inputs_recursive(specific_inputs, specific_outputs, generic_inputs)

        # TODO sometimes finding input 2 will bind a variable in input 1 - don't handle that yet

        return self._apply_rule(rule, specific_inputs, specific_outputs)

    def _apply_rule(self, rule, inputs, outputs):
        input_tvs = [i.tv for tv in inputs]
        
        # support Rules with multiple outputs later
        assert len(rule.outputs) == 1
        output_tree = rule.outputs[0]
        assert type(output_tree) == tree.Tree

        output_tv = rule.formula(input_tvs)

        output_atom = tree.atom_from_tree(output_tree)
        output_atom.tv = output_tv

        return output_atom

    def _find_inputs_recursive(self, return_inputs, return_outputs, remaining_inputs):
        template = remaining_inputs[0]
        atom = self._select_one_matching(template)
        
        if atom == None:
            return None

        # Find the substitution that would change 'template' to 'atom'
        substitution = tree.unify(template, atom, {})
        assert(substitution != None)

        remaining_inputs = remaining_inputs[1:]
        if len(remaining_inputs) == 0:
            return [] # no more inputs to be added

        remaining_inputs = tree.subst_conjunction(substitution, remaining_inputs)

        return_inputs.append(atom)

        return self._find_inputs_recursive(return_inputs, return_outputs, remaining_inputs)
        
    def forward_step(self):
        rule = self._select_rule()

        results = self.apply_forward(rule, inputs)

        return results

    def backward_step(self):
        rule = self._select_rule()

        outputs = self.select_outputs(rule)

        if outputs != None:
            requirements = self.evaluate_backward(rule, outputs)

        return results

