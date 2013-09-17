__author__ = 'jade'

from pln.rules.rules import Rule
from pln.logic import Logic

from opencog.atomspace import types, Atom

import random

# @todo opencog/atomspace/AttentionBank.h defines a getAttentionalFocusBoundary method, which should eventually be used instead of this
def get_attentional_focus(atomspace, attentional_focus_boundary=0):
    nodes = atomspace.get_atoms_by_type(types.Atom)
    attentional_focus = []
    for node in nodes:
        if node.av['sti'] > attentional_focus_boundary:
            attentional_focus.append(node)
    return attentional_focus

class AbstractChainer(Logic):
    def __init__(self, atomspace):
        Logic.__init__(self, atomspace)
        self.rules = []

    def add_rule(self, rule):
        assert isinstance(rule, Rule)
        assert type(rule) != Rule

        self.rules.append(rule)

    def _select_one_matching(self, template, require_nonzero_tv = True):
        attentional_focus = get_attentional_focus(self._atomspace)

        matching_atoms = self.find(template, attentional_focus)

        if len(matching_atoms) > 0:
            return self._selectOne(matching_atoms)
        else:
            # if it can't find anything in the attentional focus, try the whole atomspace.

            atoms = self._atomspace.get_atoms_by_type(types.Atom)
            if require_nonzero_tv:
                atoms = [atom for atom in atoms if atom.tv.count > 0]

            matching_atoms = self.find(template, atoms)
            return random.choice(matching_atoms)

    def _selectOne(self, atoms):
        assert type(atoms[0]) == Atom

        max = sum([atom.av['sti'] for atom in atoms])
        pick = random.randrange(0, max)
        current = 0
        for atom in atoms:
            current += atom.av['sti']
            if current >= pick:
                return atom

        assert False

    def _select_rule(self):
        return random.choice(self.rules)

    def valid_structure(self, atom):
        '''Does a kind of 'type-check' to see if an Atom's structure makes sense.
           The forward chainer is very creative and will come up with anything allowed by the Rules
           otherwise.'''
        if atom.type == types.InheritanceLink:
            is_between_nodes = atom.out[0].is_node() and atom.out[1].is_node()
            not_self_link    = atom.out[0] != atom.out[1]
            return is_between_nodes and not_self_link
        else:
            return True

class Chainer(AbstractChainer):
    def __init__(self, atomspace, stimulateAtoms=False):
        AbstractChainer.__init__(self, atomspace)

        self._stimulateAtoms = stimulateAtoms

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

        (generic_inputs, generic_outputs) = rule.standardize_apart_input_output(self)
        specific_inputs = []
        specific_outputs = []
        empty_substitution = {}
        self._find_inputs_recursive(specific_inputs, specific_outputs, 
                                    generic_inputs, generic_outputs, empty_substitution)

        # TODO sometimes finding input 2 will bind a variable in input 1 - don't handle that yet

        if not self.valid_structure(specific_outputs[0]):
            return None

        return self._apply_rule(rule, specific_inputs, specific_outputs)

    def _apply_rule(self, rule, inputs, outputs):
        input_tvs = [i.tv for i in inputs]
        
        # support Rules with multiple outputs later
        assert len(outputs) == 1
        output_atom = outputs[0]
        assert type(output_atom) == Atom

        output_tv = rule.calculate(input_tvs)

        output_atom.tv = output_tv

        if self._stimulateAtoms:
            self._give_stimulus(output_atom)
            for i in inputs:
                self._give_stimulus(i)

        return (output_atom, inputs)

    def _give_stimulus(self, atom):
        # TODO hack - it should use the actual stimulus system to be compatible with ECAN
        atom.av = {'sti':atom.av['sti']+10, 'lti':atom.av['lti']+3}

    def _find_inputs_recursive(self, return_inputs, return_outputs, remaining_inputs, generic_outputs, subst_so_far):
        # base case of recursion
        if len(remaining_inputs) == 0:
            # set the outputs after you've found all the inputs
            # mustn't use '=' because it will discard the original reference and thus have no effect
            return_outputs += self.substitute_list(subst_so_far, generic_outputs)
            return

        # normal case of recursion

        template = remaining_inputs[0]
        atom = self._select_one_matching(template)
        
        if atom is None:
            print 'unable to match:',template
        assert(atom != None)

        # Find the substitution that would change 'template' to 'atom'
        substitution = self.unify(template, atom, subst_so_far)
        assert(substitution != None)

        remaining_inputs = remaining_inputs[1:]

        remaining_inputs = self.substitute_list(substitution, remaining_inputs)

        return_inputs.append(atom)

        return self._find_inputs_recursive(return_inputs, return_outputs, remaining_inputs, generic_outputs, substitution)
        
    def forward_step(self):
        rule = self._select_rule()

        results = self._apply_forward(rule)

        return results

    def backward_step(self):
        rule = self._select_rule()

        outputs = self.select_outputs(rule)

        if outputs != None:
            requirements = self.evaluate_backward(rule, outputs)

        return results

