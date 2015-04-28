from opencog.cogserver import MindAgent
from opencog.atomspace import types
from pln.chainers import Chainer
from pln.rules import *


class InferenceAgent(MindAgent):
    def __init__(self):
        self.chainer = None
        self.num_steps_per_cycle = 100

    def create_chainer(self, atomspace):
        # Note: using stimulateAtoms will cause a segfault if you
        # create the Agent from the Python shell (use the agents-start
        # command in the cogserver shell). It's because giving atoms
        # stimulus only works if the MindAgent is added to the
        # CogServer's list of agents.
        self.chainer = Chainer(atomspace,
                               stimulateAtoms=False,
                               agent=self,
                               learnRuleFrequencies=True)

        # Todo: Cleanup the following section where rules are added

        # ImplicationLink is MixedImplicationLink, you could also have
        # Extensional and Intensional Implication. etc. but that's a bit
        # much.
        # similarity_types =
        # [types.SimilarityLink,
        # types.ExtensionalSimilarityLink,
        # types.IntensionalSimilarityLink,
        # types.EquivalenceLink]
        #
        # conditional_probability_types =
        # [types.InheritanceLink,
        # types.SubsetLink,
        # types.IntensionalInheritanceLink,
        # types.ImplicationLink]

        # always use the mixed inheritance types, because human inference
        # is normally a mix of intensional and extensional
        conditional_probability_types = [types.InheritanceLink,
                                         types.ImplicationLink,
                                         types.PredictiveImplicationLink]
        similarity_types = [types.SimilarityLink,
                            types.EquivalenceLink]

        for link_type in conditional_probability_types:
            self.chainer.add_rule(InversionRule(self.chainer, link_type))
            self.chainer.add_rule(DeductionRule(self.chainer, link_type))
            self.chainer.add_rule(InductionRule(self.chainer, link_type))
            self.chainer.add_rule(AbductionRule(self.chainer, link_type))
            # Seems better than Modus Ponens - it doesn't make anything up
            self.chainer.add_rule(TermProbabilityRule(self.chainer, link_type))
            self.chainer.add_rule(ModusPonensRule(self.chainer, link_type))
            self.chainer.add_rule(PreciseModusPonensRule(self.chainer, link_type))

        for link_type in similarity_types:
            # SimilarityLinks don't require an InversionRule obviously
            self.chainer.add_rule(TransitiveSimilarityRule(self.chainer,
                                                           link_type))
            self.chainer.add_rule(SymmetricModusPonensRule(self.chainer,
                                                           link_type))

        self.chainer.add_rule(EvaluationImplicationRule(self.chainer))

        # These two Rules create mixed links out of intensional and
        # extensional links
        self.chainer.add_rule(InheritanceRule(self.chainer))
        self.chainer.add_rule(SimilarityRule(self.chainer))

        for link_type in conditional_probability_types:
            self.chainer.add_rule(AndToSubsetRule1(self.chainer, link_type))

            for N in xrange(2, 8):
                self.chainer.add_rule(AndToSubsetRuleN(self.chainer, link_type, N))

        # boolean links
        for rule in create_and_or_rules(self.chainer, 2, 8):
            self.chainer.add_rule(rule)

        for N in xrange(2, 8):
            self.chainer.add_rule(
                boolean_rules.AndBulkEvaluationRule(self.chainer, N))
        for N in xrange(3, 8):
            self.chainer.add_rule(
                boolean_rules.NegatedAndBulkEvaluationRule(self.chainer, N))

        # create probabilistic logical links out of MemberLinks

        self.chainer.add_rule(AndEvaluationRule(self.chainer))
        self.chainer.add_rule(OrEvaluationRule(self.chainer))

        # These two "macro rules" make the individual rules redundant
        self.chainer.add_rule(ExtensionalLinkEvaluationRule(self.chainer))
        self.chainer.add_rule(IntensionalLinkEvaluationRule(self.chainer))
        #self.chainer.add_rule(SubsetEvaluationRule(self.chainer))
        self.chainer.add_rule(NegatedSubsetEvaluationRule(self.chainer))
        #self.chainer.add_rule(
        #    ExtensionalSimilarityEvaluationRule(self.chainer))
        #self.chainer.add_rule(
        #    IntensionalInheritanceEvaluationRule(self.chainer))
        #self.chainer.add_rule(
        #    IntensionalSimilarityEvaluationRule(self.chainer))

        self.member_rules = [GeneralEvaluationToMemberRule(self.chainer),
            MemberToEvaluationRule(self.chainer)]
        self.member_rules += \
            create_general_evaluation_to_member_rules(self.chainer)
        for rule in self.member_rules:
            self.chainer.add_rule(rule)

        # It's important to have both of these
        self.chainer.add_rule(MemberToInheritanceRule(self.chainer))
        self.chainer.add_rule(InheritanceToMemberRule(self.chainer))

        # AttractionLink could be useful for causality
        self.chainer.add_rule(AttractionRule(self.chainer))

        self.chainer.add_rule(ScholemRule(self.chainer))

        boolean_transformation_rules = create_boolean_transformation_rules(self.chainer)
        for rule in boolean_transformation_rules:
            self.chainer.add_rule(rule)

        #self.chainer.add_rule(OntologicalInheritanceRule(self.chainer))

        #for rule in temporal_create_temporal_rules(self.chainer):
        #self.chainer.add_rule(rule)

        #higher_order_rules = []
        #for rule in self.chainer.rules:
        #higher_order_append(HigherOrderRule(self.chainer, rule))

        #contextual_rules = []
        #for rule in self.chainer.rules:
        #   contextual_append(ContextualRule(self.chainer, rule))

        #for rule in higher_order_rules + contextual_rules:
        #   self.chainer.add_rule(rule)

        #self.chainer.add_rule(AndToContextRule(self.chainer,
        #                                       types.InheritanceLink))

    def run(self, atomspace):
        if self.chainer is None:
            self.create_chainer(atomspace)
            # For simplicity, do nothing the first time. Silly APIs
            # mean you have to call run to set the atomspace
            return

        # Update all of the node probabilities at each step
        #self.chainer.update_all_node_probabilities()

        for i in xrange(0, self.num_steps_per_cycle):
            self.step()

    # Todo: The variable 'result' is never used
    def step(self):
        result = self.chainer.forward_step()
        result = self.chainer.backward_step()

'''
# test it with forgetting, updating and diffusion
scm-eval (load-scm-from-file "../wordpairs.scm")
loadpy pln
agents-start pln.InferenceAgent opencog::ForgettingAgent opencog::ImportanceUpdatingAgent opencog::ImportanceDiffusionAgent
'''


class TestInferenceAgent(InferenceAgent):
    def run(self, atomspace):
        if self.chainer is None:
            self.create_chainer(atomspace)

        self.chainer.find_atom(self.chainer.get_query(), time_allowed=300)


class ForwardInferenceAgent(InferenceAgent):
    # Todo: The variable 'result' is never used
    def step(self):
        result = self.chainer.forward_step()


class BackwardInferenceAgent(InferenceAgent):
    # Todo: The variable 'result' is never used
    def step(self):
        result = self.chainer.backward_step()
