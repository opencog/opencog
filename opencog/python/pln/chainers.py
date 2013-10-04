__author__ = 'jade'

from pln.rules.rules import Rule
from pln.logic import Logic
from pln.formulas import revisionFormula

from opencog.atomspace import types, Atom, AtomSpace, TruthValue

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
        # If the template is a specific atom, just return that!
        if len(self.variables(template)) == 0:
            return template

        # TODO backward chaining doesn't work, because this method will pick up Atoms
        # that have variables in them (including the queries left over from other inferences)!!!

        attentional_focus = get_attentional_focus(self._atomspace)

        atom = self._select_from(template, attentional_focus, require_nonzero_tv, useAF=True)

        if not atom:
            # if it can't find anything in the attentional focus, try the whole atomspace.
            if template.type == types.VariableNode:
                root_type = types.Atom
            else:
                root_type = template.type
            all_atoms = self._atomspace.get_atoms_by_type(root_type)
            atom = self._select_from(template, all_atoms, require_nonzero_tv, useAF=False)

        return atom

    def _select_from(self, template, atoms, require_nonzero_tv, useAF):
        # never allow inputs with variables?
        ground_results = True

        atoms = self.find(template, atoms)

        if require_nonzero_tv:
            atoms = [atom for atom in atoms if atom.tv.count > 0]

        if ground_results:
            atoms = [atom for atom in atoms if len(self.variables(atom)) == 0]

        if len(atoms) == 0:
            return None

        if useAF:
            return self._selectOne(atoms)
        else:
            # selectOne doesn't work if the STI is below 0 i.e. outside of the attentional focus.
            return random.choice(atoms)

    def _selectOne(self, atoms):
        # The score should always be an int or stuff will get weird. sti is an int but TV mean and conf are not
        def sti_score(atom):
            return atom.av['sti']

        def mixed_score(atom):
            return int(sti_score(atom)*atom.tv.mean*atom.tv.confidence)

        score = sti_score

        assert type(atoms[0]) == Atom

        max = sum([score(atom) for atom in atoms])
        pick = random.randrange(0, max)
        current = 0
        for atom in atoms:
            current += score(atom)
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
            #is_between_nodes = atom.out[0].is_node() and atom.out[1].is_node()
            not_self_link    = atom.out[0] != atom.out[1]
            return not_self_link
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

        self.trail_atomspace = AtomSpace()
        # TODO actually load and save these. When loading it, rebuild the indexes above.

    def forward_step(self):
        rule = self._select_rule()

        results = self._apply_forward(rule)

        return results

    def backward_step(self):
        rule = self._select_rule()

        results = self._apply_backward(rule)

        return results

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
                                    generic_inputs, generic_outputs, empty_substitution, require_nonzero_tv=True)
        if not found:
            return None

        # handle rules that create their output in a custom way, not just using templates
        if hasattr(rule, 'custom_compute'):
            (specific_outputs, output_tvs) = rule.custom_compute(specific_inputs)
        else:
            output_tvs = rule.calculate(specific_inputs)

        # TODO sometimes finding input 2 will bind a variable in input 1 - don't handle that yet

        if self._validate(rule, specific_inputs, specific_outputs):
            return self._apply_rule(rule, specific_inputs, specific_outputs, output_tvs)
        else:
            return None

    def _validate(self, rule, inputs, outputs):
        print rule, map(str,inputs), map(str,outputs)
        # Sanity checks
        if not self.valid_structure(outputs[0]):
            self.log_failed_inference('invalid structure')
            return False

        if self._compute_trail_and_check_cycles(outputs[0], inputs):
            self.log_failed_inference('cycle detected')
            return False

        if self._is_repeated(rule, outputs, inputs):
            self.log_failed_inference('repeated inference')
            return False

        return True

    def _find_inputs_recursive(self, return_inputs, return_outputs, remaining_inputs, generic_outputs, subst_so_far, require_nonzero_tv=True):
        '''Recursively find suitable inputs and outputs for a Rule. Chooses them at random based on STI. Store them in return_inputs and return_outputs (lists of Atoms). Return True if inputs were found, False otherwise.'''
        remaining_inputs = self.substitute_list(subst_so_far, remaining_inputs)

        # base case of recursion
        if len(remaining_inputs) == 0:
            # set the outputs after you've found all the inputs
            # mustn't use '=' because it will discard the original reference and thus have no effect
            return_outputs += self.substitute_list(subst_so_far, generic_outputs)
            return True

        # normal case of recursion

        template = remaining_inputs[0]
        atom = self._select_one_matching(template, require_nonzero_tv)
        
        if require_nonzero_tv and atom is None:
            print 'unable to match:',template
            return False

        assert atom != None
        # Find the substitution that would change 'template' to 'atom'
        substitution = self.unify(template, atom, subst_so_far)
        if substitution == None:
            import pdb; pdb.set_trace()

        remaining_inputs = remaining_inputs[1:]

        return_inputs.append(atom)

        return self._find_inputs_recursive(return_inputs, return_outputs, remaining_inputs, generic_outputs, substitution, require_nonzero_tv)

    def _apply_rule(self, rule, inputs, outputs, output_tvs):
        for (atom, new_tv) in zip(outputs, output_tvs):
            self._revise_tvs(atom, new_tv)

        if self._stimulateAtoms:
            for atom in outputs:
                self._give_stimulus(atom)
            for atom in inputs:
                self._give_stimulus(atom)

        return (rule, inputs, outputs)

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

    def _is_repeated(self, rule, outputs, inputs):
        # TODO fixme - it doesn't handle UnorderedLinks properly (such as AndLinks).
        # Such as And(A B) or And(B A)

        # Record the exact list of atoms used to produce an output one time. (Any atom can be
        # produced multiple ways using different Rules and inputs.)
        # Return True if this exact inference has been applied before

        # In future this should record to the Inference History Repository atomspace
        # convert the inputs to a tuple so they can be stored in a set.
        inputs = tuple(inputs)
        outputs = tuple(outputs)
        productions = self.produced_from[outputs]
        if inputs in productions:
            return True
        else:
            productions.add(inputs)
            self._add_to_inference_repository(rule, outputs, inputs)
            return False

    def _add_to_inference_repository(self, rule, outputs, inputs):
        TA = self.trail_atomspace
        L = TA.add_link
        N = TA.add_node

        # create new lists of inputs and outputs for the separate trails atomspace
        inputs  = [self.transfer_atom(TA, a) for a in inputs]
        outputs = [self.transfer_atom(TA, a) for a in outputs]

        L(types.ExecutionLink, [
            N(types.GroundedSchemaNode, rule.name),
            L(types.ListLink, inputs),
            L(types.ListLink, outputs)])

    def _apply_backward(self, rule):
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
        found = self._choose_outputs_inputs_recursive(specific_inputs, specific_outputs, 
                                    generic_inputs, generic_outputs, empty_substitution)

        print rule, map(str,specific_outputs), map(str,specific_inputs)
        if not found:
            return None

        # If it doesn't find suitable inputs, then it can still stimulate the atoms, but not assign a TruthValue
        # Stimulating the inputs makes it more likely to find them in future.

        if not self._validate(rule, specific_inputs, specific_outputs):
            return None

        if self._all_nonzero_tvs(specific_inputs):
            output_tvs = rule.calculate(specific_inputs)

            return self._apply_rule(rule, specific_inputs, specific_outputs, output_tvs)
        else:
            if self._stimulateAtoms:
#                for atom in specific_outputs:
#                    self._give_stimulus(atom)
                for atom in specific_inputs:
                    self._give_stimulus(atom)
            return (specific_outputs, specific_inputs)

    def _choose_outputs_inputs_recursive(self, return_inputs, return_outputs, all_inputs, remaining_outputs, subst_so_far):
        '''Choose some outputs for a Rule, and then choose suitable inputs. Chooses them at random based on STI. Store them in return_inputs and return_outputs (lists of Atoms).'''
        # base case of recursion
        # After choosing outputs, use the other helper function to choose inputs.
        # If you pass it subst_so_far, it will choose inputs compatible with the
        # outputs we've already chosen!
        if len(remaining_outputs) == 0:
            inputs_found = self._find_inputs_recursive(return_inputs, return_outputs=[], remaining_inputs=all_inputs, generic_outputs=[], subst_so_far=subst_so_far, require_nonzero_tv=True)

            return inputs_found

        # normal case of recursion
        # First choose the outputs, then the inputs.
        # The outputs are atoms already in the AtomSpace (but sometimes without a TruthValue)
        # The inputs can be existing atoms. They will be created with 0TV otherwise.
        # You can use find_inputs_recursive to choose the inputs after choosing outputs

        template = remaining_outputs[0]
        atom = self._select_one_matching(template, require_nonzero_tv=False)
    
        if atom is None:
            self.log_failed_inference('backward chainer: unable to find target atom matching:'+str(template))
            return False

        # Find the substitution that would change 'template' to 'atom'
        substitution = self.unify(template, atom, subst_so_far)
        assert(substitution != None)

        remaining_outputs = remaining_outputs[1:]

        remaining_outputs = self.substitute_list(substitution, remaining_outputs)

        return_outputs.append(atom)

        return self._choose_outputs_inputs_recursive(return_inputs, return_outputs, all_inputs, remaining_outputs, substitution)


