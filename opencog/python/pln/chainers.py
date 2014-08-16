__author__ = 'jade'

import random
from collections import defaultdict

from pln.logic import Logic
from pln.rules import *
from opencog.atomspace import types, Atom, AtomSpace, TruthValue
from opencog import logger

"""
Todo:
The API for adding Rules and Links to a PLN Chainer could probably be
made easier to use: it could be redesigned so that you would just pass
a list of rule types and link types when you instantiate the Chainer,
without having to be aware of the different sets of rules that require
different argument sets.
"""

LOG_LEVEL = "FINE"
LOG_DEFAULT_FILENAME = "/tmp/pln.log"
DEFAULT_STIMULUS_PER_ATOM = 10

'''
There are lots of possible heuristics for choosing atoms. It also
depends on the kind of rule, and will have a HUGE effect on the system.
(i.e. if you choose less useful atoms, you will waste a lot of time /
it will take exponentially longer to find something useful / you will
find exponentially more rubbish! and since all other parts of opencog
have combinatorial explosions, generating rubbish is VERY bad!)

The algorithm doesn't explicitly distinguish between depth-first and
breadth-first search, but you could bias it. You can set a parameter
for how much STI to give each atom that is used (or produced) by
forward chaining (which should be set relative to the other AI
processes!). If it's low, then PLN will tend to stay within the current
"bubble of attention". If it's high, then PLN will tend to "move the
bubble of attention" so it chains together strings (or rather DAGs)
of inferences.

I really think we should make a reasonably fast algorithm, and then add
rules one at a time, adding useful heuristics simultaneously (so that
PLN will actually be useful rather than just get in the AI's way). It
looks to me like this set of heuristics would be enough to make PLN not
spiral crazily out of control.

* choose atoms with probability proportional to their STI
* higher confidence atoms are usually better
* higher strength atoms are often better (but obviously for finding
  Not(A) you don't want A to be strong)

It's possible to do a lot of learning online, and it's both efficient
and a nice control strategy. The system should give high STI to things
that happened recently or are near the agent

Cognitive synergy:
* After you have clustering and fishgram working, you can do reasoning
  on the results of those processes
* Dimensional embedding tells you which similarity links (or inheritance
  links) are likely to be good (see the opencog book)
* PLN planning: you could have various special heuristics for
  hierarchical planning
* Use MOSES

You can (and should) also think carefully about how to represent data so
that more useful inferences are shorter. This will generally help a
LOT!!!!!!!!!!

Special cases for different kinds of rules:
* For deduction, it would be really useful to explicitly notice a
hierarchy of more abstract/specific concepts (as described in the
RealWorldReasoning book).
And any time you find Inheritance A B, also calculate the intensional
and extensional parts separately
* For space/time reasoning, you should prefer things that are close in
  space/time (and in fact, for reasoning about objects generally. You
  can find nearby things using the space/time servers
* You can probably get past toddler-level AI without having to use
  predicate logic much. And it may be easier to convert 2+ place
  predicates into 1-place predicates (see GeneralEvaluationToMemberRule)
'''


class AbstractChainer(Logic):
    """
    Has important utility methods for chainers.
    """
    def __init__(self, atomspace, log):
        self.log = log
        Logic.__init__(self, atomspace, self.log)
        self.rules = []
        self.random = random.Random(0)

    def add_rule(self, rule):
        assert isinstance(rule, Rule)

        self.rules.append(rule)

        for template in rule._inputs + rule._outputs:
            template.av = {'vlti': 1}

    # Todo: Should this be a static method?
    def log_failed_inference(self, message):
        self.log.debug("Attempted invalid inference: {0}".format(message))

    # Finds a list of candidate atoms and then matches all of them
    # against the template. Uses the Attentional Focus where possible
    # but will search the whole AtomSpace if necessary.
    def _select_one_matching(self,
                             template,
                             s={},
                             rule=None,
                             other_inputs=None,
                             allow_zero_tv=False):
        # If the template is a specific atom, just return that!
        if len(self.variables(template)) == 0:
            return template

        atoms = self.atomspace.get_atoms_in_attentional_focus()
        self.log.debug("Found in attentional focus: {0}.".format(len(atoms)))
        atom = self._select_atom(template,
                                 s,
                                 atoms,
                                 allow_zero_tv,
                                 rule,
                                 other_inputs)
        if not atom:
            self.log.debug("No atoms found in attentional focus")
            # if it can't find anything in the attentional focus, try
            # the whole atomspace. (it actually still uses indexes to
            # find a subset of the links, that is more likely to be
            # useful)
            atoms = self.lookup_atoms(template, s)
            self.log.debug("Lookup atoms returned {0} results"
                           .format(len(atoms)))
            atom = self._select_atom(template,
                                     s,
                                     atoms,
                                     allow_zero_tv,
                                     rule,
                                     other_inputs)
            self.log.debug("Select atom returned:\n{0}".format(atom))

        return atom

    # Todo: What needs to be done with this method?
    def disabled__select_fromAF(self, template, substitution):
        # never allow inputs with variables. (not even for
        # backchaining targets)
        ground_results = False

        atoms = self.find(template, substitution)

        if len(atoms) == 0:
            return None

        return self._selectOne(atoms)

    def _select_atom(self,
                     template,
                     substitution,
                     atoms,
                     allow_zero_tv,
                     rule=None,
                     other_inputs=None):
        # This method will sample atoms before doing any filtering, and
        # will only apply the filters on as many atoms as it needs to.

        # The atomspace lookup and the shuffle are both O(N)...
        # But if you shuffle it you're guaranteed to eventually find a
        # suitable atom if one exists. The correct option would be to
        # do it inside the atomspace. Sample without replacement, or
        # shuffle a little bit at a time. Or even being able to get
        # results N atoms at a time would help a lot!
        self.random.shuffle(atoms)
        #atoms = self.random.sample(atoms, len(atoms))

        if len(atoms) is 0:
            self.log.debug("Warning: _select_atom called with empty list")
            return None

        # O(N*the percentage of atoms that are useful)
        num_attempts = 0
        for atom in atoms:
            num_attempts += 1
            if self.wanted_atom(atom,
                                template,
                                substitution,
                                ground=False,
                                allow_zero_tv=allow_zero_tv):
                if rule is None:
                    self.log.debug("Found atom after {0} attempts".
                                   format(num_attempts))
                    return atom
                if rule.valid_inputs(other_inputs+[atom]):
                    self.log.debug("Found atom after {0} attempts".
                                   format(num_attempts))
                    return atom
                else:
                    self.log_failed_inference(
                        "Attempt #" + str(num_attempts) + "{0} produced " +
                        "invalid input, trying another one " +
                        str(other_inputs + [atom]))

        self.log.debug("Unable to find wanted atom after {0} attempts".
                       format(num_attempts))
        return None

    def _selectOne(self, atoms):
        # The score should always be an int or stuff will get weird.
        # sti is an int but TV mean and conf are not
        def sti_score(atom):
            sti = atom.av['sti']
            return max(sti, 1)

        # Todo: The function 'mixed_score' is never used
        def mixed_score(atom):
            # Todo: What does this formula mean?
            s = int(10 * sti_score(atom) + 100 * (atom.tv.mean + 0.01) +
                    100 * (atom.tv.confidence + 0.01))
            return s

        score = sti_score
        #score = mixed_score

        assert type(atoms[0]) == Atom

        total = sum([score(atom) for atom in atoms])
        pick = self.random.randrange(0, total)
        current = 0
        for atom in atoms:
            current += score(atom)
            if current >= pick:
                return atom

        assert False

    def _select_rule(self):
        if not self.learnRuleFrequencies:
            return self.random.choice(self.rules)
        else:
            def score(rule):
                return self.rule_count[rule]

            max = sum([score(rule) for rule in self.rules])
            pick = self.random.randrange(0, max)
            current = 0
            for rule in self.rules:
                current += score(rule)
                if current >= pick:
                    self.rule_count[rule] += 1
                    return rule

    def valid_structure(self, atom):
        """
        Does a kind of 'type-check' to see if an Atom's structure makes
        sense. The forward chainer is very creative and will come up
        with anything allowed by the Rules otherwise.
        """
        # Reject And(And(...) ...)
        if atom.type in BOOLEAN_LINKS:
            nested_boolean = any(outgoing_atom.type == atom.type
                                 for outgoing_atom in atom.out)
            if nested_boolean:
                return False
        elif atom.arity == 2:
            # heuristically assume that all selflinks are invalid!
            self_link = atom.out[0] == atom.out[1]
            # don't allow inheritancelinks (or anything else?) with two
            # variables. (These are rapidly created as backchaining
            # targets with DeductionRule)
            both_variables = self.is_variable(atom.out[0]) \
                and self.is_variable(atom.out[1])
            if self_link or both_variables:
                return False

        return True

    def count_objects(self):
        all_object_nodes = self.atomspace.get_atoms_by_type(types.ObjectNode)
        return len(all_object_nodes)

    def count_members(self, conceptnode):
        return len(self.find_members(conceptnode))

    def find_members(self, conceptnode):
        var = self.new_variable()
        template = self.link(types.MemberLink, [var, conceptnode])
        members = self.lookup_atoms(template, {})
        members = [m for m in members if self.wanted_atom(m, template, ground=True)]
        self.atomspace.remove(var)
        return members

    def find_eval_links(self, predicatenode):
        var = self.new_variable()
        template = self.link(types.EvaluationLink, [predicatenode, var])
        evals = self.lookup_atoms(template, {})
        evals = [e for e in evals if self.wanted_atom(e, template, ground=True)]
        self.atomspace.remove(var)
        return evals

    def node_tv(self, conceptnode):
        """
        Calculate the probability of any object being a member of this
        conceptnode. Only works for predicates whose domain is
        ObjectNodes. It will often change as new information is received
        """
        N = self.count_objects()
        p = self.count_members(conceptnode) * 1.0 / N
        return TruthValue(p, N)

    def update_all_node_probabilities(self):
        for node in self.atomspace.get_atoms_by_type(types.ConceptNode):
            node.tv = self.node_tv(node)


class AtomSpaceBasedInferenceHistory:
    """
    Use the AtomSpace to record inference history. It has two main uses. The
    chainer can lookup an inference to see whether it has already been performed.
    And after a successful inference, it can look up the produced Atom to reconstruct
    the tree of rules that were applied to find it.
    """
    def __init__(self, chainer, main_atomspace, history_atomspace):
        """
        Currently the history_atomspace is the same as the main atomspace (but
        it could/should be a separate one).
        """
        self._chainer = chainer
        self.log = chainer.log
        self._main_atomspace = main_atomspace
        self._history_atomspace = history_atomspace
        self._all_applications = set()

    def rule_application_is_new(self, rule, inputs, outputs):
        """
        Record a rule application in the history/main atomspace and check if it is new.
        Return true if it is a new rule application or false if it has been performed
        before.
        """

        TA = self._history_atomspace
        L = TA.add_link
        N = TA.add_node

        # create new lists of inputs and outputs for the separate history atomspace
        #inputs  = [self.transfer_atom(TA, a) for a in inputs]
        #outputs = [self.transfer_atom(TA, a) for a in outputs]

        app = L(types.ExecutionLink, [
            N(types.GroundedSchemaNode, rule.name),
            L(types.ListLink, [
                L(types.ListLink, inputs),
                L(types.ListLink, outputs)
            ])
        ])

        if app not in self._all_applications:
            self._all_applications.add(app)
            return True
        else:
            return False

    def lookup_input_tuples_by_output_tuple_and_rule(self, rule, output):
        """
        Lookup every input tuple that the given Rule has used to create this output.
        Used by temporal rules
        """
        # rule in the arguments is a Rule object, but _get_rule returns a Node

        # Find all the apps for that output, and then select only the
        # ones for this rule
        apps = self._lookup_applications_by_output_atom(output)
        relevant_apps = [app for app in apps if self._get_rule(app).name == rule.name]
        input_tuples = [tuple(self._get_inputs(app)) for app in relevant_apps]
        return input_tuples

    # TODO only works for rules with one output. This is hard to fix
    def _lookup_applications_by_output_atom(self, output):
        # ExecutionLink
        #   rule
        #   ListLink
        #       ListLink inputs
        #       ListLink outputs
        TA = self._history_atomspace
        L = TA.add_link
        N = TA.add_node

        rule = self._chainer.new_variable()
        inputs = self._chainer.new_variable()
        outputs = L(types.ListLink, [output])

        template = L(types.ExecutionLink, [
            rule,
            L(types.ListLink, [
                inputs,
                outputs
            ])
        ])
        
        apps = self._chainer.lookup_atoms(template, {})
        apps = [a for a in apps if self._chainer.wanted_atom(a, template, ground=False, allow_zero_tv=True)]
        # Reject old templates. They should be deleted really
        apps = [app for app in apps if not self._get_rule(app).name.startswith('$')]
        # It will find the template as one of the matches
        #apps.remove(template)

        self._history_atomspace.remove(template)
        self._history_atomspace.remove(inputs, recursive=True)
        self._history_atomspace.remove(rule, recursive=True)

        assert all(app in self._history_atomspace for app in apps)
        return apps

    def _get_inputs(self, application):
        """
        Returns the inputs for a Rule application. application is the atom
        representing the rule application.
        """
        inputs = application.out[1].out[0].out

        return inputs

    def _get_outputs(self, application):
        outputs = application.out[1].out[1].out

        return outputs

    def _get_rule(self, application):
        rule = application.out[0]

        return rule

    def check_cycles(self, inputs, outputs):
        """
        Recursively find the atoms used to produce output (the
        inference trail). If there is a cycle, return True. Otherwise
        return False. It has to be used before rule_application_is_new
        because we don't want to add the application to perform the application
        at all if there is a cycle (and hence don't want to add it to the
        history.
        """
        for output_atom in outputs:        
            if output_atom in inputs:
                return True
            for input_atom in inputs:
                history = self.lookup_history_for_atom(input_atom)
                for application in history:
                    if output_atom in self._get_inputs(application):
                        return True

        return False

    def lookup_history_for_atom(self, atom, history=None):
        """
        Lookup the entire inference history for an Atom. It is returned as a list
        of Atoms representing the rule applications
        """
        if history is None: history = []

        apps = self._lookup_applications_by_output_atom(atom)
        history+=apps
        assert all(app in self._history_atomspace for app in history)
        if len(apps) == 2: assert apps[0] != apps[1]

        for app in apps:
            inputs = self._get_inputs(app)
            for input_atom in inputs:
                self.lookup_history_for_atom(input_atom, history)

        # It goes through the tree of applications in top-down fashion (i.e.
        # from the last conclusion to the first. Reversing the list means it
        # goes from the premises to the conclusion.
        assert all(app in self._history_atomspace for app in history)
        return reversed(history)
        
    def print_application(self, application):
        assert application in self._history_atomspace

        self.log.debug("Using {0}\n".format(self._get_rule(application).name))
        for input_atom in self._get_inputs(application):
            assert input_atom in self._history_atomspace
            self.log.debug("{0}\n".format(input_atom))
        self.log.debug("|=")
        for output_atom in self._get_outputs(application):
            assert output_atom in self._history_atomspace
            self.log.debug("{0}\n".format(output_atom))

    def print_history(self, atom):
        history = self.lookup_history_for_atom(atom)
        assert all(app in self._history_atomspace for app in history)

        self.log.debug("Inference trail:")
        for application in history:
            self.print_application(application)

    def get_history(self):
        """
        Returns all of the applications in the inference history
        """
        return self._all_applications


class Chainer(AbstractChainer):
    """
    PLN forward/backward chainer. It chooses Atoms randomly based on
    STI. It can do a single backward or forward step. By running the
    step repeatedly it can simulate chaining, without needing a messy
    specialized chaining algorithm. It can do a mixture of forward
    and backward chaining and easily interoperate with other OpenCog
    processes.

    It lets you use any Rule frequencies (and can learn them). You
    should set stimulateAtoms=True if you want it to create chains
    (otherwise it will just make shallow inferences based on a whole
    bunch of Atoms, which is sometimes good too).

    It can check for repeated inferences and cycles. PLN truth value
    revision is supported.
    """

    ### public interface

    def __init__(self,
                 atomspace,
                 stimulateAtoms=False,
                 agent=None,
                 learnRuleFrequencies=False,
                 preferAttentionalFocus=False,
                 allow_output_with_variables=False,
                 allow_backchaining_with_variables=False,
                 delete_temporary_variables=False,
                 check_cycles=True,
                 check_repeats=True,
                 log=None):

        if log is None:
            self.log = logger.create_logger(LOG_DEFAULT_FILENAME)
        else:
            self.log = logger
        self.log.set_level(LOG_LEVEL)
        self.log.fine("Initializing PLN MindAgent")

        AbstractChainer.__init__(self, atomspace, self.log)

        # It stores a reference to the MindAgent object so it can
        # stimulate atoms.
        self.atomspace = atomspace
        self._stimulateAtoms = stimulateAtoms
        self._agent = agent
        self._preferAttentionalFocus = preferAttentionalFocus
        self.learnRuleFrequencies = learnRuleFrequencies
        self._allow_output_with_variables = allow_output_with_variables
        self._allow_backchaining_with_variables = allow_backchaining_with_variables
        self._delete_temporary_variables = delete_temporary_variables
        self._check_cycles = check_cycles
        self._check_repeats = check_repeats
        # You have to record the inference history in order to check cycles
        if not check_repeats: assert check_cycles

        self.atomspace = atomspace

        self.history = AtomSpaceBasedInferenceHistory(chainer=self, main_atomspace=atomspace, history_atomspace=atomspace)

        # Record how often each Rule is used. To bias the Rule
        # frequencies. It will take longer to adapt if you set this
        # higher (this is important so it won't get a crazy feedback
        # loop).
        initial_frequency = 100

        def constant_factory():
            return initial_frequency
        if learnRuleFrequencies:
            self.rule_count = defaultdict(constant_factory)

    def forward_step(self, rule=None):
        self.log.debug("=============================== Running forward step")
        if rule is None:
            rule = self._select_rule()
        results = self._apply_forward(rule)

        if results is not None:
            self.log.debug("Forward step results:\n{0}".format(results))
        else:
            self.log.debug("Forward step returned no results")

        return results

    def backward_step(self, rule=None, target_atoms=None):
        self.log.debug("=============================== Running backward step")

        if rule is None:
            rule = self._select_rule()

        results = self._apply_backward(rule, target_outputs=target_atoms)

        if results is not None:
            self.log.debug("Backward step results:\n{0}".format(results))
        else:
            self.log.debug("Backward step returned no results")

        return results

    ### forward chaining implementation

    def _apply_forward(self, rule):
        """
        randomly choose suitable atoms for this rule's inputs
            * choose a random atom matching the first input to the rule
            * choose a random atom matching the second input to the
              rule, compatible with the first input
            * etc
            * it can fail if there are no compatible atoms
        if all inputs are found, then
            * apply the rule and create the output
            * give it an appropriate TV (using the formula and possibly
              revision)
            * give it an STI boost
            * record this inference in the InferenceHistoryRepository
        """
        self.log.debug(str(rule))

        try:
            (generic_inputs, generic_outputs, created_vars) = \
                rule.standardize_apart_input_output(self)
            specific_inputs = []
            empty_substitution = {}
            subst = self._choose_inputs(rule,
                                        specific_inputs,
                                        generic_inputs,
                                        empty_substitution)
            if subst is None:
                return None

            # set the outputs after you've found all the inputs
            # In Modus Ponens for example: if variable B refers to
            # (Member $T Trains) and $T refers to Thomas
            # The first line will replace B with (Member $T Trains)
            specific_outputs = self.substitute_list(subst, generic_outputs)
            # The second line will replace (Member $T Trains) with
            # (Member Thomas Trains), which is what we really need
            specific_outputs = self.substitute_list(subst, specific_outputs)
        finally:
            # delete the query atoms after you've finished using them.
            # recursive means it will delete the new variable nodes and
            # the links containing them (but not the existing nodes and
            # links)
            if self._delete_temporary_variables:
                for var in created_vars:
                    if var in self.atomspace:
                        self.atomspace.remove(var, recursive=True)

        return self.apply_rule(rule, specific_inputs, specific_outputs)

    def apply_bulk(self, rule):
        """
        Apply a rule to every possible input. It's much more efficient
        (for that case) than calling apply_forward(rule) repeatedly.
        So I don't have to include backtracking, it only works for
        rules with one input template.
        """
        (generic_inputs, generic_outputs, created_vars) = \
            rule.standardize_apart_input_output(self)
        output_atoms = []

        assert len(generic_inputs) == 1
        template = generic_inputs[0]
        input_atoms = self.find(template)

        for input in input_atoms:
            subst = self.unify(template, input, {})
            outputs = self.substitute_list(subst, generic_outputs)
            self.apply_rule(rule, [input], outputs)
            output_atoms += generic_outputs

        for var in created_vars:
            self.atomspace.remove(var, recursive=True)

        return output_atoms

    def _choose_inputs(self,
                       rule,
                       return_inputs,
                       input_templates,
                       subst_so_far,
                       allow_zero_tv=False):
        """
        Find suitable inputs and outputs for a Rule. Chooses them at
        random based on STI. Store them in return_inputs and
        return_outputs (lists of Atoms). Return the substitution if
        inputs were found, None otherwise.
        """
        return_inputs += [x for x in input_templates]

        for i in xrange(0, len(input_templates)):
            input_templates = self.substitute_list(subst_so_far,
                                                   input_templates)
            template = input_templates[i]

            atom = self._select_one_matching(template,
                                             subst_so_far,
                                             rule,
                                             input_templates[0:i])
            
            if atom is not None:
                # Find the substitution that would change 'template' to
                # 'atom'
                # Alternatively this could be returned by
                # _select_one_matching
                subst_so_far = self.unify(template, atom, subst_so_far)
                #if subst_so_far == None:
                #    import pdb; pdb.set_trace()

                return_inputs[i] = atom
            else:
                if not allow_zero_tv:
                    #self.log.debug("Unable to match: {0}".format(template))
                    return None
                # This means it won't be able to produce the output,
                # but choosing some inputs is still essential for
                # backward chaining.
                # Just specialize the rest of the inputs. These "input"
                # will actually just be 0-tv atoms, and it can become a
                # BC target later.
                #query = self.substitute(subst_so_far, template)
                #return_inputs[i] = query
                return_inputs[i:] = self.substitute_list(subst_so_far,
                                                         input_templates[i:])
                return subst_so_far

        return subst_so_far

    ### backward chaining implementation

    def delete_queries(self, created_vars, subst):
        """
        Delete any variables that have been bound to something
        (they're no longer required). Variables that haven't been
        bound become backward chaining queries
        """
        if self._delete_temporary_variables:
            for var in created_vars:
                if subst is None or (var in subst and subst[var] != var):
                    self.atomspace.remove(var, recursive=True)

    def _apply_backward(self, rule, target_outputs=None):
        """
        target outputs is the exact series of outputs to produce. If
        you don't specify, it will choose outputs for you

        choose outputs if needed, and then choose some inputs to
        apply this rule with.
        """
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

        (generic_inputs, generic_outputs, created_vars) = \
            rule.standardize_apart_input_output(self)
        subst = {}

        if target_outputs is None:
            # This variable isn't really used; now we just use the
            # substitution instead
            initial_outputs = []
            subst = self._choose_outputs(initial_outputs,
                                         generic_outputs,
                                         subst)
        else:
            for (template, atom) in zip(generic_outputs, target_outputs):
                subst = self.unify(template, atom, subst)

        if not subst:
            self.delete_queries(created_vars, subst)
            return None

        specific_inputs = []
        subst = self._choose_inputs(rule,
                                    specific_inputs,
                                    generic_inputs,
                                    subst,
                                    allow_zero_tv=True)
        found = len(subst) > 0

        final_outputs = self.substitute_list(subst, generic_outputs)
        # See apply_forward
        final_outputs = self.substitute_list(subst, final_outputs)

        self.delete_queries(created_vars, subst)
        if not found:
            return None

        # If it doesn't find suitable inputs, then it can still stimulate
        # the atoms, but not assign a TruthValue. Stimulating the inputs
        # makes it more likely to find them in future.

        # That is wrong because many rules have 0 inputs (and use
        # custom_compute instead)
        #if len(specific_inputs) == 0:
        #    self.log.debug("Error: specific_inputs == 0")
        #    return None
        if self._all_nonzero_tvs(specific_inputs):
            return self.apply_rule(rule, specific_inputs, final_outputs)
        else:
            if (not self._allow_backchaining_with_variables) \
                and any(self._contains_variables(atom)
                        for atom in specific_inputs+final_outputs):
                return None

            if self._stimulateAtoms:
                # Todo: Which of the following is correct?
#                for atom in specific_outputs:
#                    self._give_stimulus(atom)
                for atom in specific_inputs:
                    self._give_stimulus(atom)
            return rule, specific_inputs, final_outputs

    def _choose_outputs(self, return_outputs, output_templates, subst_so_far):

        for i in xrange(0, len(output_templates)):
            template = output_templates[i]

            atom = self._select_one_matching(template,
                                             subst_so_far,
                                             allow_zero_tv=True)
            if atom is None:
                self.log_failed_inference(
                    'backward chainer: unable to find target atom matching:' +
                    str(template))
                return None

            # Find the substitution that would change 'template' to 'atom'
            # Alternatively this could be returned by _select_one_matching
            subst_so_far = self.unify(template, atom, subst_so_far)
            if subst_so_far is None:
                import pdb
                pdb.set_trace()

            return_outputs.append(atom)

        return subst_so_far

    ### stuff used by both forward and backward chaining

    def apply_rule(self, rule, inputs, outputs):
        """
        Called by both the backward and forward chainers. inputs and
        outputs are found generically for every rule, but some rules
        have special case code and will create their own outputs.
        """
        if hasattr(rule, 'custom_compute'):
            (outputs, output_tvs) = rule.custom_compute(inputs, outputs)
            if len(outputs) == 0:
                return None
            return self._apply_rule(rule,
                                    inputs,
                                    outputs,
                                    output_tvs,
                                    revise=True)
        elif hasattr(rule, 'temporal_compute'):
            # Lookup all inputs ever found by the chainer, and then use
            # the special temporal computation instead of revision.
            past_input_tuples = \
                self.history.lookup_input_tuples_by_output_tuple_and_rule(rule, outputs)
            all_input_tuples = [inputs] + past_input_tuples

            (outputs, output_tvs) = rule.temporal_compute(all_input_tuples)
            return self._apply_rule(rule,
                                    inputs,
                                    outputs,
                                    output_tvs,
                                    revise=False)
        else:
            if len(outputs) == 0:
                return None
            output_tvs = rule.calculate(inputs)
            if output_tvs is None:
                return None
            return self._apply_rule(rule,
                                    inputs,
                                    outputs,
                                    output_tvs,
                                    revise=True)

    def _apply_rule(self, rule, inputs, outputs, output_tvs, revise=True):
        """
        Helper for apply_rule
        """
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

        return rule, inputs, outputs

    def _revise_tvs(self, atom, new_tv):
        old_tv = atom.tv

        revised_tv = revisionFormula([old_tv, new_tv])
        atom.tv = revised_tv

        # atom.tv= actually uses the "truth value merging" function in
        # the atomspace, which will always use the tv with the higher
        # confidence (which is always the revised TV, at least with
        # the current revisionFormula)
        assert atom.tv == revised_tv

    def _give_stimulus(self, atom, amount=DEFAULT_STIMULUS_PER_ATOM):
        if self._stimulateAtoms:
            self._agent.stimulate_atom(atom, amount)

    ### automatically reject some inferences based on various problems

    def _validate(self, rule, inputs, outputs):
        # some of the validations might not make sense for backward chaining

        # Sanity checks
        for i in outputs:
            if not self._allow_output_with_variables \
                and self._contains_variables(i):
                self.log_failed_inference('output contains variable(s)')
                return False

            if not self.valid_structure(i):
                self.log_failed_inference('invalid structure %s %s %s' %
                                          (rule, inputs, outputs))
                return False

        if self._check_cycles:
            if self.history.check_cycles(inputs, outputs):
                self.log_failed_inference('cycle detected')
                return False

        if self._check_repeats:
            if self._is_repeated(rule, outputs, inputs):
                self.log_failed_inference('repeated inference')
                return False

        return True

    def _contains_variables(self, output):
        return len(self.variables(output)) > 0

    def _is_repeated(self, rule, outputs, inputs):
        # Record the exact list of atoms used to produce an output one
        # time. (Any atom can be produced multiple ways using different
        # Rules and inputs.)
        # Return True if this exact inference has been applied before

        # About unordered links
        # Such as And(A B) or And(B A)
        # The AtomSpace will create only one representation of each
        # unordered link. So this algorithm should still work okay for
        # unordered links.

        new = self.history.rule_application_is_new(rule,
                                                        inputs=inputs,
                                                        outputs=outputs)
        if not new:
            return True
        else:
            return False

    def lookup_rule(self, rule_name):
        for rule in self.rules:
            if rule.name == rule_name or rule.full_name == rule_name:
                return rule

        raise ValueError("lookup_rule: rule doesn't exist " + rule_name)

    def test_rules(self, sample_count=20):
        for rule in self.rules:
            self.test_rule(rule, sample_count)

    def test_rule(self, rule, sample_count):
        # Do a series of samples; different atoms with the same rules.
        self.random.seed(0)
        self.log.debug("Testing {0} in forward chainer".format(rule))

        for i in xrange(0, sample_count):
            results = self.forward_step(rule=rule)
            if results:
                self.log.debug(results)

        self.log.debug("Testing {0} in backward chainer".format(rule))

        for i in xrange(0, sample_count):
            results = self.backward_step(rule=rule)
            if results:
                self.log.debug(results)

    def find_atom(self, atom, time_allowed=300, required_confidence=0):
        """
        Run inference until atom is proved with >required_confidence confidence,
        or time runs out (measured in seconds). With required_confidence=0, it
        will stop after any inference produces the target atom. With
        required_confidence=1 it will keep trying other inference paths until it
        runs out of time. (With any other value it will stop if it reaches that
        level of confidence)
        """
        self.log.debug("Trying to produce truth values for atom:\n{0}".
                       format(repr(atom)))

        import time
        start_time = time.time()

        while time.time() - start_time < time_allowed:
            if self._stimulateAtoms:
                self._give_stimulus(atom)

            self.backward_step()

            target_instances = self.get_target_instances(atom)
            if target_instances:
                for instance in target_instances:
                    self.log.debug("Target produced!")
                    self.log.debug(repr(instance))

                    self.log.debug("Inference steps")
                    self.history.print_history(instance)

                    if instance.tv.confidence > required_confidence:
                        return True

        self.log.debug(
            "Failed to find target in {0} seconds".format(time_allowed))
        return False

    def get_target_instances(self, target):
        """
        Ask the atomspace whether the target has been found (i.e. a TV
        with >0 confidence has already been produced by inference).
        The target is allowed to contain variables, in which case any
        instance of it is accepted (an instance is the target but with
        variables bound)
        """
        atoms = self.lookup_atoms(target, {})
        atoms = [atom for atom in atoms if
                 self.wanted_atom(atom, target, {}, allow_zero_tv=False, ground=False)]
        return atoms

    def get_query(self):
        return self.get_predicate_arguments('query')[0]
