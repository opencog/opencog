from opencog.cogserver import MindAgent
from opencog.atomspace import types

from pln.chainers import Chainer, get_attentional_focus
from pln.rules import rules

class ForwardInferenceAgent(MindAgent):
    def __init__(self):
        self.chainer = None

    def create_chainer(self, atomspace):
        self.chainer = Chainer(atomspace, stimulateAtoms = False, agent = self)

#        deduction_link_types = [types.InheritanceLink]
##            types.SubsetLink, types.IntensionalInheritanceLink]
#        for link_type in deduction_link_types:
#            self.chainer.add_rule(rules.InversionRule(self.chainer, link_type))
#            self.chainer.add_rule(rules.DeductionRule(self.chainer, link_type))

#        self.chainer.add_rule(rules.NotCreationRule(self.chainer))
#        self.chainer.add_rule(rules.NotEliminationRule(self.chainer))

#        for rule in rules.create_and_or_rules(self.chainer, 1, 5):
#            self.chainer.add_rule(rule)

        self.chainer.add_rule(rules.EvaluationToMemberRule(self.chainer))
#        self.chainer.add_rule(rules.MemberToInheritanceRule(self.chainer))

    def run(self, atomspace):
        # incredibly exciting futuristic display!
        #import os
        #os.system('cls' if os.name=='nt' else 'clear')

        def show_atoms(atoms):
            return ' '.join(str(atom)+str(atom.av) for atom in atoms)

        try:
            if self.chainer is None:
                self.create_chainer(atomspace)

            result = self.chainer.forward_step()
            if result:
                (rule, inputs, outputs) = result

                print '==== Inference ===='
                print rule.name, show_atoms(outputs), '<=', show_atoms(inputs)

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

def print_atoms(atoms):
    for atom in atoms:
        print atom

def show_atoms(atoms):
    return ' '.join(str(atom)+str(atom.av) for atom in atoms)

class BackwardInferenceAgent(MindAgent):
    def __init__(self):
        self.chainer = None

    def create_chainer(self, atomspace):
        self.chainer = Chainer(atomspace, stimulateAtoms = False, agent = self)

        deduction_link_types = [types.InheritanceLink]
#            types.SubsetLink, types.IntensionalInheritanceLink]
        for link_type in deduction_link_types:
            self.chainer.add_rule(rules.InversionRule(self.chainer, link_type))
            self.chainer.add_rule(rules.DeductionRule(self.chainer, link_type))

#        self.chainer.add_rule(rules.NotCreationRule(self.chainer))
#        self.chainer.add_rule(rules.NotEliminationRule(self.chainer))

#        for rule in rules.create_and_or_rules(self.chainer, 1, 5):
#            self.chainer.add_rule(rule)

#        self.chainer.add_rule(EvaluationToMemberRule(self.chainer))
#        self.chainer.add_rule(MemberToInheritanceRule(self.chainer))

    def run(self, atomspace):
        # incredibly exciting futuristic display!
        #import os
        #os.system('cls' if os.name=='nt' else 'clear')

#        try:
        if self.chainer is None:
            self.create_chainer(atomspace)

        result = self.chainer.backward_step()
        if result:
            (outputs, inputs) = result

            if False:
                print '==== Inference ===='
                print show_atoms(outputs), '<=', show_atoms(inputs)

                print
                print '==== Attentional Focus ===='
                for atom in get_attentional_focus(atomspace)[0:30]:
                    print str(atom), atom.av

                #print '==== Result ===='
                #print output
                #print '==== Trail ===='
                #print_atoms( self.chainer.trails[output] )

            if self.chainer._all_nonzero_tvs(inputs):
                raw_input()

        else:
            print 'Invalid inference attempted'
#        except Exception, e:
#            print e
#            print e.args
#            if hasattr(e, 'print_traceback'):
#                e.print_traceback()


'''
# test it with forgetting, updating and diffusion
scm-eval (load-scm-from-file "../wordpairs.scm")
loadpy pln
agents-start pln.ForwardInferenceAgent opencog::ForgettingAgent opencog::ImportanceUpdatingAgent opencog::ImportanceDiffusionAgent
'''
