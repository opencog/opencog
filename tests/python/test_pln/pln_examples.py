"""
Runs PLN examples, similar to PLNUTest. Currently only works from the
cogserver due to linking issues.

Instructions:
Run the cogserver
loadpy pln_examples
py
from pln_examples import *
pln_examples = PLNExamples(ATOMSPACE)

To run one example:
pln_examples.run_pln_example('../tests/python/test_pln/scm/specific_rules/DeductionRule.scm')

To run all the examples:
pln_examples.test_all()
"""

from time import time
from opencog.atomspace import AtomSpace, Atom, types

try:
    from pln.chainers import Chainer
    from pln.rules import *
    import opencog.scheme_wrapper
except AttributeError:
    import unittest
    raise unittest.SkipTest("PLN automated unit test temporarily disabled - "
                            "can be ran manually in the cogserver")


class PLNExamples(object):
    def __init__(self, atomspace):
        self.atomspace = atomspace
        self.passed = []
        self.failed = []

    def test_all(self):
#        import plop.collector
#        plop_collector = plop.collector.Collector()
#        plop_collector.start()

        start = time()

        examples_directory = '../tests/python/test_pln/scm/'
        scheme_files = []

        import os
        for root, dirs, files in os.walk(examples_directory):
            for name in files:
                if name.endswith('.scm'):
                    scheme_files.append(os.path.join(root, name))
        
        scheme_files.sort() 
        print scheme_files

        for f in scheme_files:
            self.run_pln_example(f)

        print 'Passed %s out of %s tests' % (len(self.passed),
                                             len(self.passed + self.failed))
        if len(self.failed):
            print 'Failed tests:'
            for f in self.failed:
                print f
        
        print "Total time:", time() - start

#        plop_collector.stop()
#        profile_data = repr(dict(plop_collector.stack_counts))
#        f = open('pln_examples.plop_profile','w')
#        f.write(profile_data)
#        f.close()

    def run_pln_example(self, filename):
        self.atomspace.clear()
        assert len(self.atomspace) == 0
            
        # Todo: The variable 'tmp' is not used
        tmp = open(filename, 'r')
        if not opencog.scheme_wrapper.load_scm(self.atomspace, filename):
            print 'unable to load file', filename
            return
        print filename

        chainer = Chainer(self.atomspace,
                          stimulateAtoms=False,
                          agent=self,
                          learnRuleFrequencies=False,
                          allow_output_with_variables=True,
                          allow_backchaining_with_variables=True,
                          check_cycles=True)

        try:
            queries = chainer.get_predicate_arguments('query')
        except ValueError, e:
            try:
                queries = chainer.get_predicate_arguments('queries')
            except ValueError, e:
                print e
                return
        try:
            rules_nodes = chainer.get_predicate_arguments('rules')
        except ValueError, e:
            print e
            return
            
        print queries
        print rules_nodes

        # Todo: The variable 'all_rules' is not used
        all_rules = AllRules(self.atomspace, chainer)
        for r in rules_nodes:
            # Yes it is, right here!
            chainer.add_rule(all_rules.lookup_rule(r))

        print [r.name for r in chainer.rules]

        if len(queries) == 1:
            query = queries[0]
            if chainer.find_atom(query, time_allowed=360):
                self.passed.append(filename)
                return True
            else:
                self.failed.append(filename)
                return False
        else:
            for query in queries:
                if chainer.find_atom(query, time_allowed=600):
                    self.passed.append(filename)
                else:
                    self.failed.append(filename)
            return True

# Todo: Could the following be encapsulated?
class AllRules(object):
    def __init__(self, atomspace, chainer):
        # contains every rule
        self.chainer = Chainer(atomspace,
                               stimulateAtoms=False,
                               agent=self,
                               learnRuleFrequencies=False)

        # contains only some rules
        self.test_chainer = chainer

        conditional_probability_types = [types.InheritanceLink,
                                         types.ImplicationLink,
                                         types.PredictiveImplicationLink]
        similarity_types = [types.SimilarityLink, types.EquivalenceLink]

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
            self.chainer.add_rule(
                TransitiveSimilarityRule(self.chainer, link_type))
            self.chainer.add_rule(
                SymmetricModusPonensRule(self.chainer, link_type))

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

        self.chainer.add_rule(ExtensionalLinkEvaluationRule(self.chainer))
        self.chainer.add_rule(IntensionalLinkEvaluationRule(self.chainer))
        self.chainer.add_rule(SubsetEvaluationRule(self.chainer))
        self.chainer.add_rule(NegatedSubsetEvaluationRule(self.chainer))
        self.chainer.add_rule(
            ExtensionalSimilarityEvaluationRule(self.chainer))
        self.chainer.add_rule(
            IntensionalInheritanceEvaluationRule(self.chainer))
        self.chainer.add_rule(
            IntensionalSimilarityEvaluationRule(self.chainer))

        self.member_rules = [GeneralEvaluationToMemberRule(self.chainer),
            MemberToEvaluationRule(self.chainer)]
        self.member_rules += \
            create_general_evaluation_to_member_rules(self.chainer)
        for rule in self.member_rules:
            self.chainer.add_rule(rule)

        # It's important to have both of these
        self.chainer.add_rule(MemberToInheritanceRule(self.chainer))
#        self.chainer.add_rule(InheritanceToMemberRule(self.chainer))

        # AttractionLink could be useful for causality
        self.chainer.add_rule(AttractionRule(self.chainer))

        self.chainer.add_rule(ScholemRule(self.chainer))

        for rule in create_temporal_rules(self.chainer):
            self.chainer.add_rule(rule)

        boolean_transformation_rules = create_boolean_transformation_rules(self.chainer)
        for rule in boolean_transformation_rules:
            self.chainer.add_rule(rule)

    def lookup_rule(self, rule_schema_node):
        rule = self.chainer.lookup_rule(rule_schema_node.name)
        rule._chainer = self.test_chainer
        return rule

#if __name__ == '__main__':
#    atomspace = AtomSpace()
#    examplesRunner = PLNExamples(atomspace)
#    examplesRunner.test_all()
