from opencog.cogserver import MindAgent
from opencog.atomspace import types

from pln.chainers import Chainer, get_attentional_focus
from pln.rules import rules

class ForwardInferenceAgent(MindAgent):
    def __init__(self):
        self.chainer = None

    def create_chainer(self, atomspace):
        self.chainer = Chainer(atomspace, stimulateAtoms = True, agent = self)

        deduction_link_types = [types.InheritanceLink,
            types.SubsetLink, types.IntensionalInheritanceLink]
        for link_type in deduction_link_types:
            self.chainer.add_rule(rules.InversionRule(self.chainer, link_type))
            self.chainer.add_rule(rules.DeductionRule(self.chainer, link_type))

#        self.chainer.add_rule(rules.NotCreationRule(self.chainer))
#        self.chainer.add_rule(rules.NotEliminationRule(self.chainer))

#        for rule in rules.create_and_or_rules(self.chainer, 1, 5):
#            self.chainer.add_rule(rule)

        self.chainer.add_rule(EvaluationToMemberRule(self.chainer))
        self.chainer.add_rule(MemberToInheritanceRule(self.chainer))

    def run(self, atomspace):
        # incredibly exciting futuristic display!
        #import os
        #os.system('cls' if os.name=='nt' else 'clear')

        def show_atoms(atoms):
            return ' '.join(str(i)+str(output.av) for atom in atoms)

#        try:
        if self.chainer is None:
            self.create_chainer(atomspace)

        result = self.chainer.forward_step()
        if result:
            (outputs, inputs) = result

            print '==== Inference ===='
            print show_atoms(output), str(output.av), '<=',show_atoms(input)

            print
            print '==== Attentional Focus ===='
            for atom in get_attentional_focus(atomspace)[0:30]:
                print str(atom), atom.av

            #print '==== Result ===='
            #print output
            #print '==== Trail ===='
            #print_atoms( self.chainer.trails[output] )
        else:
            print 'Invalid inference attempted'
#        except Exception, e:
#            print e
#            print e.args
#            if hasattr(e, 'print_traceback'):
#                e.print_traceback()

def print_atoms(atoms):
    for atom in atoms:
        print atom

'''
# test it with forgetting, updating and diffusion
scm-eval (load-scm-from-file "../wordpairs.scm")
loadpy pln
agents-start pln.ForwardInferenceAgent opencog::ForgettingAgent opencog::ImportanceUpdatingAgent opencog::ImportanceDiffusionAgent
'''
