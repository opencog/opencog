__author__ = 'jade'

from pln.rules.rules import Rule
from pln.logic import Logic
from pln.formulas import revisionFormula
from pln.rules import rules

from opencog.atomspace import types, Atom, AtomSpace, TruthValue

import random
from collections import defaultdict

'''There are lots of possible heuristics for choosing atoms. It also depends on the kind of rule, and will have a HUGE effect on the system. (i.e. if you choose less useful atoms, you will waste a lot of time / it will take exponentially longer to find something useful / you will find exponentially more rubbish! and since all other parts of opencog have combinatorial explosions, generating rubbish is VERY bad!)

The algorithm doesn't explicitly distinguish between depth-first and breadth-first search, but you could bias it. You can set a parameter for how much STI to give each atom that is used (or produced) by forward chaining (which should be set relative to the other AI processes!). If it's low, then PLN will tend to stay within the current "bubble of attention". If it's high, then PLN will tend to "move the bubble of attention" so it chains together strings (or rather DAGs) of inferences.

I really think we should make a reasonably fast algorithm, and then add rules one at a time, adding useful heuristics simultaneously (so that PLN will actually be useful rather than just get in the AI's way). It looks to me like this set of heuristics would be enough to make PLN not spiral crazily out of control.

* choose atoms with probability proportional to their STI
* higher confidence atoms are usually better
* higher strength atoms are often better (but obviously for finding Not(A) you don't want A to be strong)

It's possible to do a lot of learning online, and it's both efficient and a nice control strategy. The system should give high STI to things that happened recently or are near the agent

Cognitive synergy:
* After you have clustering and fishgram working, you can do reasoning on the results of those processes 
* Dimensional embedding tells you which similarity links (or inheritance links) are likely to be good (see the opencog book)
* PLN planning: you could have various special heuristics for hierarchical planning
* Use MOSES

You can (and should) also think carefully about how to represent data so that more useful inferences are shorter. This will generally help a LOT!!!!!!!!!!

Special cases for different kinds of rules:
* For deduction, it would be really useful to explicitly notice a hierarchy of more abstract/specific concepts (as described in the RealWorldReasoning book).
And any time you find Inheritance A B, also calculate the intensional and extensional parts separately
* For space/time reasoning, you should prefer things that are close in space/time (and in fact, for reasoning about objects generally. You can find nearby things using the space/time servers
* You can probably get past toddler-level AI without having to use predicate logic much. And it may be easier to convert 2+ place predicates into 1-place predicates (see EvaluationToMemberRule)
'''

class AbstractChainer(Logic):
    '''Has important utility methods for chainers.'''
    def __init__(self, atomspace):
        Logic.__init__(self, atomspace)
        self.rules = []

    def add_rule(self, rule):
        assert isinstance(rule, Rule)
        assert type(rule) != Rule

        self.rules.append(rule)

    def log_failed_inference(self,message):
        print 'Attempted invalid inference:',message

    # Finds a list of candidate atoms and then matches all of them against the template.
    # Uses the Attentional Focus where possible but will search the whole AtomSpace if necessary.
    def _select_one_matching(self, template, s = {}, allow_zero_tv = False):
        # If the template is a specific atom, just return that!
        if len(self.variables(template)) == 0:
            return template

        atoms = self.atomspace.get_atoms_in_attentional_focus()
        atom = self._select_atom(template, s, atoms)
        if not atom:
            # if it can't find anything in the attentional focus, try the whole atomspace. (it actually still uses indexes to find a subset of the links, that is more likely to be useful)
            atoms = self.lookup_atoms(template, s)
            atom = self._select_atom(template, s, atoms)

        return atom

    def _select_fromAF(self, template, substitution):
        # never allow inputs with variables. (not even for backchaining targets)
        ground_results = True

        atoms = self.find(template, substitution)

        if len(atoms) == 0:
            return None

        return self._selectOne(atoms)

    def _select_atom(self, template, substitution, atoms):
        # This method will sample atoms before doing any filtering, and will only apply the filters on as many atoms as it needs to.

        # The atomspace lookup and the shuffle are both O(N)...
        # But if you shuffle it you're guaranteed to eventually find a suitable atom if one exists.
        # The correct option would be to do it inside the atomspace.
        # Sample without replacement, or shuffle a little bit at a time.
        # Or even being able to get results N atoms at a time would help a lot!
        random.shuffle(atoms)
        #atoms = random.sample(atoms, len(atoms))

        # O(N*the percentage of atoms that are useful)
        for atom in atoms:
            if self.wanted_atom(atom, template, substitution, ground=False):
                return atom
        return None

    def _selectOne(self, atoms):
        # The score should always be an int or stuff will get weird. sti is an int but TV mean and conf are not
        def sti_score(atom):
            sti = atom.av['sti']
            return max(sti, 1)

        def mixed_score(atom):
            s = int(10*sti_score(atom) + 100*(atom.tv.mean+0.01) + 100*(atom.tv.confidence+0.01))
            return s

        score = sti_score
        #score = mixed_score

        assert type(atoms[0]) == Atom

        total = sum([score(atom) for atom in atoms])
        pick = random.randrange(0, total)
        current = 0
        for atom in atoms:
            current += score(atom)
            if current >= pick:
                return atom

        assert False

    def _select_rule(self):
        if not self.learnRuleFrequencies:
            return random.choice(self.rules)
        else:
            def score(rule):
                return self.rule_count[rule]

            max = sum([score(rule) for rule in self.rules])
            pick = random.randrange(0, max)
            current = 0
            for rule in self.rules:
                current += score(rule)
                if current >= pick:
                    self.rule_count[rule] += 1
                    return rule

    def valid_structure(self, atom):
        '''Does a kind of 'type-check' to see if an Atom's structure makes sense.
           The forward chainer is very creative and will come up with anything allowed by the Rules
           otherwise.'''
        if atom.arity == 2:
            # heuristically assume that all selflinks are invalid!
            self_link = atom.out[0] == atom.out[1]
            # don't allow inheritancelinks (or anything else?) with two variables. (These are rapidly created as backchaining targets with DeductionRule)
            both_variables = self.is_variable(atom.out[0]) and self.is_variable(atom.out[1])
            return not self_link and not both_variables
        else:
            return True

    def count_objects(self):
        all_object_nodes = self.atomspace.get_atoms_by_type(types.ObjectNode)
        return len(all_object_nodes)

    def count_members(self, conceptnode):
        var = self.new_variable()
        template = self.link(types.MemberLink, [var, conceptnode])
        members = self.find(template)
        self.atomspace.remove(var)
        return len(members)

    def node_tv(self, conceptnode):
        '''Calculate the probability of any object being a member of this conceptnode.
        Only works for predicates whose domain is ObjectNodes. It will often change as new information is received'''
        N = self.count_objects()
        p = self.count_members(conceptnode)*1.0/N
        return TruthValue(p, N)

    def update_all_node_probabilities(self):
        for node in self.atomspace.get_atoms_by_type(types.ConceptNode):
            node.tv = self.node_tv(node)

class InferenceHistoryIndex(object):
    def __init__(self):
        self.rule_to_io = {}

    def record_new_application(self, rule, inputs, outputs):
        outputs = tuple(outputs)
        inputs = tuple(inputs)

        try:
            output_to_inputs = self.rule_to_io[rule]
        except KeyError:
            output_to_inputs = self.rule_to_io[rule] = defaultdict(set)
            assert output_to_inputs != None

        input_tuple_set = output_to_inputs[outputs]

        if inputs in input_tuple_set:
            return False
        else:
            input_tuple_set.add(inputs)

    def lookup_all_applications(self, rule, outputs):
        try:
            output_to_inputs = self.rule_to_io[rule]
        except KeyError:
            return []
        input_tuple_set = output_to_inputs[outputs]
        return input_tuple_set

class Chainer(AbstractChainer):
    '''PLN forward/backward chainer. It chooses Atoms randomly based on STI. It can do a single
       backward or forward step. By running the step repeatedly it can simulate chaining, without
       needing a messy specialized chaining algorithm. It can do a mixture of forward and backward
       chaining and easily interoperate with other OpenCog processes.

       It lets you use any Rule frequencies (and can learn them). You should set stimulateAtoms=True
       if you want it to create chains (otherwise it will just make shallow inferences based on a whole bunch
       of Atoms, which is sometimes good too).

       It can check for repeated inferences and cycles. PLN truth value revision is supported.'''

    ### public interface

    def __init__(self, atomspace, stimulateAtoms=False, agent=None, learnRuleFrequencies=False, preferAttentionalFocus=False):
        AbstractChainer.__init__(self, atomspace)

        # It stores a reference to the MindAgent object so it can stimulate atoms.
        self._stimulateAtoms = stimulateAtoms
        self._agent = agent
        self._preferAttentionalFocus = preferAttentionalFocus
        self.learnRuleFrequencies = learnRuleFrequencies

        self.atomspace = atomspace

        # For every atom, store the atoms used to produce it (including the atoms used to produce them).
        # This prevents cycles (very important) as well as repeating the same inference.
        # Map from Atom -> set(Atom)
        # Default value is the empty set
        self.trails = defaultdict(set)
        #self.produced_from = defaultdict(set)
        self.history_index = InferenceHistoryIndex()

        #self.history_atomspace = AtomSpace()
        # TODO actually load and save these. When loading it, rebuild the indexes above.

        # Record how often each Rule is used. To bias the Rule frequencies.
        # It will take longer to adapt if you set this higher (this is important so it won't
        # get a crazy feedback loop).
        initial_frequency = 100

        def constant_factory():
            return initial_frequency
        if learnRuleFrequencies:
            self.rule_count = defaultdict(constant_factory)

    def forward_step(self, rule=None):
        if rule is None:
            rule = self._select_rule()
        print rule

        results = self._apply_forward(rule)

        return results

    def backward_step(self, rule=None, target_atoms=None):
        if rule is None:
            rule = self._select_rule()

        results = self._apply_backward(rule, target_outputs=target_atoms)

        return results

    ### forward chaining implementation

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

        try:
            (generic_inputs, generic_outputs, created_vars) = rule.standardize_apart_input_output(self)
            specific_inputs = []
            empty_substitution = {}
            subst = self._choose_inputs(specific_inputs, generic_inputs, empty_substitution)
            if subst is None:
                return None
            # set the outputs after you've found all the inputs
            # mustn't use '=' because it will discard the original reference and thus have no effect
            specific_outputs = self.substitute_list(subst, generic_outputs)
        finally:
            # delete the query atoms after you've finished using them.
            # recursive means it will delete the new variable nodes and the links
            # containing them (but not the existing nodes and links)
            for var in created_vars:
                self.atomspace.remove(var, recursive=True)

        return self.apply_rule(rule, specific_inputs, specific_outputs)

    def apply_bulk(self, rule):
        '''Apply a rule to every possible input. It's much more efficient (for that case) than calling apply_forward(rule) repeatedly. So I don't have to include backtracking, it only works for rules with one input template.'''
        (generic_inputs, generic_outputs, created_vars) = rule.standardize_apart_input_output(self)
        output_atoms = []

        assert len(generic_inputs) == 1
        template = generic_inputs[0]
        input_atoms = self.find(template)

        for input in input_atoms:
            subst = self.unify(template, input, {})
            outputs = self.substitute_list(subst, generic_outputs)
            self.apply_rule(rule, [input], outputs)
            output_atoms+=generic_outputs

        for var in created_vars:
            self.atomspace.remove(var, recursive=True)

        return output_atoms

    def _choose_inputs(self, return_inputs, input_templates, subst_so_far, allow_zero_tv=False):
        '''Find suitable inputs and outputs for a Rule. Chooses them at random based on STI. Store them in return_inputs and return_outputs (lists of Atoms). Return the substitution if inputs were found, None otherwise.'''
        return_inputs += [x for x in input_templates]

        for i in xrange(0, len(input_templates)):
            template = input_templates[i]

            atom = self._select_one_matching(template, subst_so_far)
            
            if atom != None:
                # Find the substitution that would change 'template' to 'atom'
                # Alternatively this could be returned by _select_one_matching
                subst_so_far = self.unify(template, atom, subst_so_far)
                #if subst_so_far == None:
                #    import pdb; pdb.set_trace()

                return_inputs[i] = atom
            else:
                if not allow_zero_tv:
                    print 'unable to match:',template
                    return None
                # This means it won't be able to produce the output, but choosing some inputs is still essential for backward chaining.
                # Just specialize the rest of the inputs. These "input" will actually just be 0-tv atoms, and it can become a BC target later.
                #query = self.substitute(subst_so_far, template)
                #return_inputs[i] = query
                return_inputs[i:] = self.substitute_list(subst_so_far, input_templates[i:])
                return subst_so_far

        return subst_so_far

    ### backward chaining implementation

    def delete_queries(self, created_vars, subst):
        # Delete any variables that have been bound to something (they're no longer required).
        # Variables that haven't been bound become backward chaining queries
        for var in created_vars:
            if subst is None or (var in subst and subst[var] != var):
                self.atomspace.remove(var, recursive=True)

    def _apply_backward(self, rule, target_outputs=None):
        # target outputs is the exact series of outputs to produce. If you don't specify, it will choose outputs for you

        # choose outputs if needed, and then choose some inputs to apply this rule with.

        '''
        GENERALLY-ish
        get a target to constrain the inputs
        get the inputs
        refill the target, based on extra constraints in the input

        EXAMPLE
        Inh cat breathe
        DeductionRule finds these target inputs
        Inh cat ?
        Inh ? breathe

        suppose we can prove Inh cat animal, Inh animal breathe

        Inh cat ?
        invert animal->cat

        Inh cat ?
        InversionRule
        Inh ? cat
        match to Inh animal cat
        rewrite target Inh cat animal

        '''

        (generic_inputs, generic_outputs, created_vars) = rule.standardize_apart_input_output(self)
        subst = {}

        if target_outputs is None:
            # This variable isn't really used; now we just use the substitution instead
            initial_outputs = []
            subst = self._choose_outputs(initial_outputs, generic_outputs, subst)
        else:
            for (template, atom) in zip(generic_outputs, target_outputs):
                subst = self.unify(template, atom, subst)

        if not subst:
            self.delete_queries(created_vars, subst)
            return None

        specific_inputs = []
        subst = self._choose_inputs(specific_inputs, generic_inputs, subst, allow_zero_tv = True)
        found = len(subst) > 0

        final_outputs = self.substitute_list(subst, generic_outputs)

        self.delete_queries(created_vars, subst)
        if not found:
            return None

        # If it doesn't find suitable inputs, then it can still stimulate the atoms, but not assign a TruthValue
        # Stimulating the inputs makes it more likely to find them in future.

        if self._all_nonzero_tvs(specific_inputs):
            return self.apply_rule(rule, specific_inputs, final_outputs)
        else:
            if self._stimulateAtoms:
#                for atom in specific_outputs:
#                    self._give_stimulus(atom)
                for atom in specific_inputs:
                    self._give_stimulus(atom)
            return (rule, specific_inputs, final_outputs)

    def _choose_outputs(self, return_outputs, output_templates, subst_so_far):

        for i in xrange(0, len(output_templates)):
            template = output_templates[i]

            atom = self._select_one_matching(template, subst_so_far, allow_zero_tv=True)
            if atom is None:
                self.log_failed_inference('backward chainer: unable to find target atom matching:'+str(template))
                return None

            # Find the substitution that would change 'template' to 'atom'
            # Alternatively this could be returned by _select_one_matching
            subst_so_far = self.unify(template, atom, subst_so_far)
            if subst_so_far == None:
                import pdb; pdb.set_trace()

            return_outputs.append(atom)

        return subst_so_far

    ### stuff used by both forward and backward chaining

    def apply_rule(self, rule, inputs, outputs):
        '''Called by both the backward and forward chainers. inputs and outputs are found generically for every rule, but some rules have special case code and will create their own outputs.'''
        if hasattr(rule, 'custom_compute'):
            (outputs, output_tvs) = rule.custom_compute(inputs, outputs)
            if len(outputs) == 0:
                return None
            return self._apply_rule(rule, inputs, outputs, output_tvs, revise=True)
        elif hasattr(rule, 'temporal_compute'):
            # Lookup all inputs ever found by the chainer, and then use the special temporal computation instead of revision.
            past_input_tuples = self.history_index.lookup_all_applications(rule, outputs)
            all_input_tuples = [inputs]+past_input_tuples

            (outputs, output_tvs) = rule.temporal_compute(all_input_tuples)
            return self._apply_rule(rule, inputs, outputs, output_tvs, revise=False)
        else:
            if len(outputs) == 0:
                return None
            output_tvs = rule.calculate(inputs)
            if output_tvs is None:
                return None
            return self._apply_rule(rule, inputs, outputs, output_tvs, revise=True)

    def _apply_rule(self, rule, inputs, outputs, output_tvs, revise=True):
        '''Helper for apply_rule'''
        if not self._validate(rule, inputs, outputs):
            return None

        if revise:
            assert isinstance(output_tvs, list)

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

    ### automatically reject some inferences based on various problems

    def _validate(self, rule, inputs, outputs):
        # some of the validations might not make sense for backward chaining

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
        # Record the exact list of atoms used to produce an output one time. (Any atom can be
        # produced multiple ways using different Rules and inputs.)
        # Return True if this exact inference has been applied before

        # In future this should record to the Inference History Repository atomspace
        # convert the inputs to a tuple so they can be stored in a set.

        # About unordered links
        # Such as And(A B) or And(B A)
        # The AtomSpace will create only one representation of each unordered link.
        # So this algorithm should still work okay for unordered links.

        # TODO it should work for Rules where the input order doesn't matter (e.g. AndRule)
        # currently it won't notice repetition for different input orders

        new = self.history_index.record_new_application(rule, inputs=inputs, outputs=outputs)
        if not new:
            return False
        else:
            #self._add_to_inference_repository(rule, outputs, inputs)
            return True

    def _add_to_inference_repository(self, rule, outputs, inputs):
        TA = self.history_atomspace
        L = TA.add_link
        N = TA.add_node

        # create new lists of inputs and outputs for the separate history atomspace
        inputs  = [self.transfer_atom(TA, a) for a in inputs]
        outputs = [self.transfer_atom(TA, a) for a in outputs]

        L(types.ExecutionLink, [
            N(types.GroundedSchemaNode, rule.name),
            L(types.ListLink, [
                L(types.ListLink, inputs),
                L(types.ListLink, outputs)
            ])
        ])

    def load_inference_repository(self):
        # TODO fill the history atomspace
        TA = self.history_atomspace

        for link in TA.get_atoms_by_type(types.ExecutionLink):
            rule_name = link.out[0]
            list_link = link.out[1]
            inputs= list_link.out[0]
            outputs= list_link.out[1]

            rule = self.lookup_rule(rule_name)
            inputs  = [self.transfer_atom(self.atomspace, a) for a in inputs]
            outputs = [self.transfer_atom(self.atomspace, a) for a in outputs]

    def lookup_rule(self, rule_name):
        for rule in self.rules:
            if rule.name == rule_name or rule.full_name == rule_name:
                return rule

        raise ValueError("lookup_rule: rule doesn't exist "+rule_name)

    def test_rules(self, sample_count=20):
        for rule in self.rules:
            self.test_rule(rule, sample_count)

    def test_rule(self, rule, sample_count):
        # Do a series of samples; different atoms with the same rules.
        import random; random.seed(0)

        print 'Testing',rule,'in forward chainer'

        for i in xrange(0, sample_count):
            self.forward_step(rule=rule)

        print 'Testing',rule,'in backward chainer'

        for i in xrange(0, sample_count):
            self.backward_step(rule=rule)

