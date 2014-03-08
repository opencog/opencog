"""
PLN representation of the "smokes" sample from Tuffy Markov Logic Networks

A test diary is located here:
https://github.com/opencog/test-datasets/tree/master/pln/tuffy/smokes/tests

More details on this sample are available here:
https://github.com/cosmoharrigan/tuffy/tree/master/samples/smoke
http://hazy.cs.wisc.edu/hazy/tuffy/doc/tuffy-manual.pdf

Instructions:
- Add the module path to your PYTHON_EXTENSION_DIRS in opencog.conf:
  ../opencog/python/pln/examples/tuffy/smokes
- Run the cogserver
- Clone the test-datasets repository:
  https://github.com/opencog/test-datasets
- From that repository, load this file into the cogserver:
  pln/tuffy/smokes/smokes.scm
- Run these commands in the cogserver:
  loadpy smokes_agent
  agents-start smokes_agent.InferenceAgent
- Use the Scheme shell to monitor the inference progress
"""

from opencog.cogserver import MindAgent
from opencog.atomspace import types
from pln.chainers import Chainer
from pln.rules import rules

__author__ = 'Cosmo Harrigan'

__VERBOSE__ = False

# Rules which take (chainer, link_type) as arguments
RULES_CHAINER_LINKTYPE = [rules.DeductionRule,
                          rules.InversionRule,
                          rules.ModusPonensRule,
                          rules.PreciseModusPonensRule,
                          rules.SymmetricModusPonensRule,
                          rules.TermProbabilityRule,
                          rules.TransitiveSimilarityRule]

# Rules which take (chainer) as argument
RULES_CHAINER = [rules.EvaluationToMemberRule,
                 rules.InheritanceRule,
                 rules.InheritanceToMemberRule,
                 rules.MemberToEvaluationRule,
                 rules.MemberToInheritanceRule,
                 rules.NegatedSubsetEvaluationRule,
                 rules.OrEvaluationRule,
                 rules.SimilarityRule,
                 rules.SubsetEvaluationRule,
                 rules.AndEvaluationRule]

LINK_TYPES = [types.InheritanceLink,
              types.EvaluationLink,
              types.ImplicationLink,
              types.PredictiveImplicationLink,
              types.SimilarityLink,
              types.EquivalenceLink,
              types.OrLink,
              types.AndLink,
              types.SimultaneousAndLink,
              types.ExtensionalSimilarityLink,
              types.IntensionalSimilarityLink,
              types.IntensionalInheritanceLink,
              types.ListLink,
              types.SubsetLink,
              types.SetLink,
              types.MemberLink,
              types.NotLink,
              types.FalseLink,
              types.TrueLink]


class InferenceAgent(MindAgent):
    def __init__(self):
        self.chainer = None

    def create_chainer(self, atomspace):
        self.chainer = Chainer(atomspace, stimulateAtoms=False)

        # Note:
        # The API for adding Rules and Links to a PLN Chainer is
        # hard to use. It could be redesigned so that you would
        # just pass a list of rule types and link types when you
        # instantiate the Chainer, without having to be aware of
        # the different sets of rules that require different
        # argument sets.

        for rule in RULES_CHAINER_LINKTYPE:
            for link_type in LINK_TYPES:
                print rule, link_type
                self.chainer.add_rule(rule(self.chainer, link_type))

        for rule in RULES_CHAINER:
            self.chainer.add_rule(rule(self.chainer))

    def run(self, atomspace):
        if self.chainer is None:
            self.create_chainer(atomspace)
            return

        result = self.chainer.forward_step()

        if __VERBOSE__:
            print result

        return result
