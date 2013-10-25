from opencog.cogserver import MindAgent
from opencog.atomspace import types

from pln.chainers import Chainer, get_attentional_focus
from pln.rules import rules, temporal_rules

class ForwardInferenceAgent(MindAgent):
    def __init__(self):
        self.chainer = None

    def create_chainer(self, atomspace):
        self.chainer = Chainer(atomspace, stimulateAtoms = False, agent = self, learnRuleFrequencies=True)

        # ImplicationLink is MixedImplicationLink, you could also have Extensional and Intensional Implication. etc. but that's a bit much.
#        similarity_types = [types.SimilarityLink, types.ExtensionalSimilarityLink, types.IntensionalSimilarityLink]
#            types.EquivalenceLink]
#        conditional_probability_types = [types.InheritanceLink, types.SubsetLink, types.IntensionalInheritanceLink, types.ImplicationLink]

        # always use the mixed inheritance types, because human inference is normally a mix of intensional and extensional
        conditional_probability_types = [types.InheritanceLink, types.ImplicationLink]
        similarity_types = [types.SimilarityLink, types.EquivalenceLink]

        for link_type in conditional_probability_types:
            self.chainer.add_rule(rules.InversionRule(self.chainer, link_type))
            self.chainer.add_rule(rules.DeductionRule(self.chainer, link_type))
            self.chainer.add_rule(rules.ModusPonensRule(self.chainer, link_type))

        # As a hack, use the standard DeductionRule for SimilarityLinks. It needs its own formula really.
        for link_type in similarity_types:
            self.chainer.add_rule(rules.DeductionRule(self.chainer, link_type))

        # These two Rules create mixed links out of intensional and extensional links
        self.chainer.add_rule(rules.InheritanceRule(self.chainer))
        self.chainer.add_rule(rules.SimilarityRule(self.chainer))

        # and/or/not
        self.chainer.add_rule(rules.NotCreationRule(self.chainer))
        self.chainer.add_rule(rules.NotEliminationRule(self.chainer))
        for rule in rules.create_and_or_rules(self.chainer, 1, 2):
            self.chainer.add_rule(rule)

        # create probabilistic logical links out of MemberLinks
        self.chainer.add_rule(rules.SubsetEvaluationRule(self.chainer))
        self.chainer.add_rule(rules.IntensionalInheritanceEvaluationRule(self.chainer))

        self.chainer.add_rule(rules.ExtensionalSimilarityEvaluationRule(self.chainer))
        self.chainer.add_rule(rules.IntensionalSimilarityEvaluationRule(self.chainer))

        self.chainer.add_rule(rules.EvaluationToMemberRule(self.chainer))
        for rule in rules.create_general_evaluation_to_member_rules(self.chainer):
            self.chainer.add_rule(rule)

        # It's important to have both of these
        self.chainer.add_rule(rules.MemberToInheritanceRule(self.chainer))
#        self.chainer.add_rule(rules.InheritanceToMemberRule(self.chainer))

        # AttractionLink could be useful for causality
        self.chainer.add_rule(rules.AttractionRule(self.chainer))

#        for rule in temporal_rules.create_temporal_rules(self.chainer):
#            self.chainer.add_rule(rule)

    def run(self, atomspace):
        # incredibly exciting futuristic display!
        #import os
        #os.system('cls' if os.name=='nt' else 'clear')

        def show_atoms(atoms):
            return ' '.join(str(atom)+str(atom.av) for atom in atoms)

        if self.chainer is None:
            self.create_chainer(atomspace)

        result = self.chainer.forward_step()
        if result:
            (rule, inputs, outputs) = result

            print '==== Inference ===='
            print rule.name, show_atoms(outputs), '<=', show_atoms(inputs)

            #print
            #print '==== Attentional Focus ===='
            #for atom in get_attentional_focus(atomspace)[0:30]:
            #    print str(atom), atom.av

            #print '==== Result ===='
            #print output
            #print '==== Trail ===='
            #print_atoms( self.chainer.trails[output] )
        else:
            print 'Invalid inference attempted'

        try:
            pass
        except AssertionError:
            import sys,traceback
            _,_,tb = sys.exc_info()
            traceback.print_tb(tb) # Fixed format

            tbInfo = traceback.extract_tb(tb)
            filename,line,func,text = tbInfo[-1]
            print ('An error occurred on line ' + str(line) + ' in statement ' + text)
            exit(1)
        except Exception, e:
            print e
            print e.args
            e.print_traceback()

            import sys,traceback
            _,_,tb = sys.exc_info()
            traceback.print_tb(tb) # Fixed format

            tbInfo = traceback.extract_tb(tb)
            filename,line,func,text = tbInfo[-1]
            print ('An error occurred on line ' + str(line) + ' in statement ' + text)
            exit(1)

    def monte_carlo_one_atom(self, atom, sample_count=100):
        old_tv = atom.tv
        print old_tv
        for i in xrange(0, sample_count):
            self.chainer.backward_step(target_atoms=[atom])
        print 'tv before sampling', old_tv
        print 'tv after sampling', atom.tv

def print_atoms(atoms):
    for atom in atoms:
        print atom

def show_atoms(atoms):
    return ' '.join(str(atom)+str(atom.av) for atom in atoms)

class BackwardInferenceAgent(ForwardInferenceAgent):
    def run(self, atomspace):
        # incredibly exciting futuristic display!
        #import os
        #os.system('cls' if os.name=='nt' else 'clear')

        def show_atoms(atoms):
            return ' '.join(str(atom)+str(atom.av) for atom in atoms)

        if self.chainer is None:
            self.create_chainer(atomspace)

        result = self.chainer.backward_step()
        if result:
            (rule, inputs, outputs) = result

            print '==== Inference ===='
            print rule.name, show_atoms(outputs), '<=', show_atoms(inputs)


'''
# test it with forgetting, updating and diffusion
scm-eval (load-scm-from-file "../wordpairs.scm")
loadpy pln
agents-start pln.ForwardInferenceAgent opencog::ForgettingAgent opencog::ImportanceUpdatingAgent opencog::ImportanceDiffusionAgent
'''
