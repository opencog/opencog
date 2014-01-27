'''Runs PLN examples, similar to PLNUTest. Currently only works from the cogserver due
to linking issues.'''
from time import time
from pprint import pprint

import util
from opencog.atomspace import AtomSpace, Atom
from pln.chainers import Chainer

import scheme_wrapper

class PLNExamples(object):
    def __init__(self, atomspace):
        self.atomspace = atomspace
        self.passed = []
        self.failed = []

    def test_all(self):
        import plop.collector
        plop_collector = plop.collector.Collector()
        plop_collector.start()

        start = time()

        examples_directory = '../tests/python/test_pln/scm/'
        scheme_files = []

        import os
        from os.path import join
        for root, dirs, files in os.walk(examples_directory):
            for name in files:
                if name.endswith('.scm'):
                    scheme_files.append(os.path.join(root, name))
        
        scheme_files.sort() 
        print scheme_files

        for f in scheme_files:
            self.run_pln_example(f)

        print 'Passed %s out of %s tests' % (len(passed), len(passed+failed))
        if len(failed):
            print 'Failed tests:'
            for f in failed:
                print f
        
        print "Total time:",time() - start

        plop_collector.stop()
        profile_data = repr(dict(plop_collector.stack_counts))
        f = open('pln_examples.plop_profile','w')
        f.write(profile_data)
        f.close()

    def run_pln_example(self, filename):
        self.atomspace.clear()
            
        tmp = open(filename,'r')
        scheme_wrapper.load_scm(self.atomspace, filename)
        print filename

        chainer = Chainer(atomspace, stimulateAtoms = False, agent = self, learnRuleFrequencies=False)

        try:
            query = chainer.get_predicate_arguments('query')[0]
            rules_nodes = chainer.get_predicate_arguments('rules')
        except ValueError, e:
            print e
            return
        print query
        print rules_nodes

        all_rules = AllRules(self.atomspace)
        for r in rules_nodes:
            chainer.add_rule(all_rules.lookup_rule(r))

        chainer.find_atom(query)

        if len(res):
            print 'PASSED'
            passed.append(f)
        else:
            print 'FAILED'
            failed.append(f)

class AllRules(object):
    def __init__(self, atomspace):
        self.chainer = Chainer(atomspace, stimulateAtoms = False, agent = self, learnRuleFrequencies=False)

        conditional_probability_types = [types.InheritanceLink, types.ImplicationLink, types.PredictiveImplicationLink]
        similarity_types = [types.SimilarityLink, types.EquivalenceLink]

        for link_type in conditional_probability_types:
            self.chainer.add_rule(rules.InversionRule(self.chainer, link_type))
            self.chainer.add_rule(rules.DeductionRule(self.chainer, link_type))
            self.chainer.add_rule(rules.InductionRule(self.chainer, link_type))
            self.chainer.add_rule(rules.AbductionRule(self.chainer, link_type))
            # Seems better than Modus Ponens - it doesn't make anything up
            self.chainer.add_rule(rules.TermProbabilityRule(self.chainer, link_type))
            self.chainer.add_rule(rules.ModusPonensRule(self.chainer, link_type))

        for link_type in similarity_types:
            # SimilarityLinks don't require an InversionRule obviously
            self.chainer.add_rule(rules.TransitiveSimilarityRule(self.chainer, link_type))
            self.chainer.add_rule(rules.SymmetricModusPonensRule(self.chainer, link_type))

        self.chainer.add_rule(predicate_rules.EvaluationImplicationRule(self.chainer))

        # These two Rules create mixed links out of intensional and extensional links
        self.chainer.add_rule(rules.InheritanceRule(self.chainer))
        self.chainer.add_rule(rules.SimilarityRule(self.chainer))

        # boolean links
        for rule in boolean_rules.create_and_or_rules(self.chainer, 2, 8):
            self.chainer.add_rule(rule)

        # create probabilistic logical links out of MemberLinks

        self.chainer.add_rule(rules.AndEvaluationRule(self.chainer))
        self.chainer.add_rule(rules.OrEvaluationRule(self.chainer))

        self.chainer.add_rule(rules.ExtensionalLinkEvaluationRule(self.chainer))
        self.chainer.add_rule(rules.IntensionalLinkEvaluationRule(self.chainer))
        self.chainer.add_rule(rules.SubsetEvaluationRule(self.chainer))
        self.chainer.add_rule(rules.NegatedSubsetEvaluationRule(self.chainer))
        self.chainer.add_rule(rules.ExtensionalSimilarityEvaluationRule(self.chainer))
        self.chainer.add_rule(rules.IntensionalInheritanceEvaluationRule(self.chainer))
        self.chainer.add_rule(rules.IntensionalSimilarityEvaluationRule(self.chainer))

        self.member_rules = [rules.EvaluationToMemberRule(self.chainer)]
        self.member_rules += rules.create_general_evaluation_to_member_rules(self.chainer)
        for rule in self.member_rules:
            self.chainer.add_rule(rule)

        # It's important to have both of these
        self.chainer.add_rule(rules.MemberToInheritanceRule(self.chainer))
#        self.chainer.add_rule(rules.InheritanceToMemberRule(self.chainer))

        # AttractionLink could be useful for causality
        self.chainer.add_rule(rules.AttractionRule(self.chainer))

        self.chainer.add_rule(quantifier_rules.ScholemRule(self.chainer))

    def lookup_rule(self, rule_schema_node):
        return self.chainer.lookup_rule(rule_schema_node.name)

#if __name__ == '__main__':
#    atomspace = AtomSpace()
#    examplesRunner = PLNExamples(atomspace)
#    examplesRunner.test_all()

