__author__ = 'jade'

from pln.rules.rules import Rule
from pln.logic import Logic
from pln.formulas import revisionFormula

from opencog.atomspace import types, Atom

import random
from collections import defaultdict

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
            if len(matching_atoms) == 0:
                return None
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

    def log_failed_inference(self,message):
        print 'Attempted invalid inference:',message

class Chainer(AbstractChainer):
    def __init__(self, atomspace, stimulateAtoms=False, agent=None):
        AbstractChainer.__init__(self, atomspace)

        # It stores a reference to the MindAgent object so it can stimulate atoms.
        self._stimulateAtoms = stimulateAtoms
        self._agent = agent

        # For every atom, store the atoms used to produce it (including the atoms used to produce them).
        # This prevents cycles (very important) as well as repeating the same inference.
        # Map from Atom -> set(Atom)
        # Default value is the empty set
        self.trails = defaultdict(set)
        self.produced_from = defaultdict(set)

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
        found = self._find_inputs_recursive(specific_inputs, specific_outputs, 
                                    generic_inputs, generic_outputs, empty_substitution)
        if not found:
            return None

        # TODO sometimes finding input 2 will bind a variable in input 1 - don't handle that yet

        # Sanity checks

        if not self.valid_structure(specific_outputs[0]):
            self.log_failed_inference('invalid structure')
            return None

        # TODO only supporting one output
        if self._compute_trail_and_check_cycles(specific_outputs[0], specific_inputs):
            self.log_failed_inference('cycle detected')
            return None

        if self._is_repeated(rule, specific_outputs[0], specific_inputs):
            self.log_failed_inference('repeated inference')
            return None

        return self._apply_rule(rule, specific_inputs, specific_outputs)

    def _find_inputs_recursive(self, return_inputs, return_outputs, remaining_inputs, generic_outputs, subst_so_far):
        '''Recursively find suitable inputs and outputs for a Rule. Chooses them at random based on STI. Store them in return_inputs and return_outputs (lists of Atoms). Return True if inputs were found, False otherwise.'''
        # base case of recursion
        if len(remaining_inputs) == 0:
            # set the outputs after you've found all the inputs
            # mustn't use '=' because it will discard the original reference and thus have no effect
            return_outputs += self.substitute_list(subst_so_far, generic_outputs)
            return True

        # normal case of recursion

        template = remaining_inputs[0]
        atom = self._select_one_matching(template)
        
        if atom is None:
            print 'unable to match:',template
        return False

        # Find the substitution that would change 'template' to 'atom'
        substitution = self.unify(template, atom, subst_so_far)
        assert(substitution != None)

        remaining_inputs = remaining_inputs[1:]

        remaining_inputs = self.substitute_list(substitution, remaining_inputs)

        return_inputs.append(atom)

        return self._find_inputs_recursive(return_inputs, return_outputs, remaining_inputs, generic_outputs, substitution)

    def _apply_rule(self, rule, inputs, outputs):
        output_tvs = rule.compute(inputs)

        for (atom, new_tv) in zip(outputs, output_tvs):
            self._revise_tvs(atom, atom.tv, new_tv)

        if self._stimulateAtoms:
            for atom in outputs:
                self._give_stimulus(atom)
            for atom in inputs:
                self._give_stimulus(atom)

        return (output_atoms, inputs)

    def _revise_tvs(self, atom, new_tv):
        old_tv = atom.tv

        revised_tv = revisionFormula([old_tv, new_tv])
        atom.tv = revised_tv

    def _give_stimulus(self, atom):
        # Arbitrary
        STIMULUS_PER_ATOM = 10
        self._agent.stimulate_atom(atom, STIMULUS_PER_ATOM)

    def _compute_trail_and_check_cycles(self, output, inputs):
        ''' Recursively find the atoms used to produce output (the inference trail). If there is a cycle, return True.
            Otherwise return False'''
        trail = self.trails[output]

        # Check for cycles before adding anything into the trails
        if output in inputs:
            return True
        for atom in inputs:
            input_trail = self.trails[atom]

            if output in input_trail:
                return None

        for atom in inputs:
            # Reusing the same Atom more than once is bad
            # (maybe? What if you're combining it with a different atom to produce a new TV for the same result?)
            #if atom in trail:
            #    return None
            trail.add(atom)

        for atom in inputs:
            input_trail = self.trails[atom]
            trail |= input_trail

        return False

    def _is_repeated(self, rule, output, inputs):
        # TODO fixme - it doesn't handle UnorderedLinks properly (such as AndLinks).
        # Such as And(A B) or And(B A)

        # Record the exact list of atoms used to produce an output one time. (Any atom can be
        # produced multiple ways using different Rules and inputs.)
        # Return True if this exact inference has been applied before

        # In future this should record to the Inference History Repository atomspace
        # convert the inputs to a tuple so they can be stored in a set.
        inputs = tuple(inputs)
        productions = self.produced_from[output]
        if inputs in productions:
            return True
        else:
            productions.add(inputs)
            return False

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

