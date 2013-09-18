from opencog.cogserver import MindAgent

from pln.chainers import Chainer, get_attentional_focus
from pln.rules import rules

class ForwardInferenceAgent(MindAgent):
    def __init__(self):
        self.chainer = None

    def run(self, atomspace):
        if self.chainer is None:
            self.chainer = Chainer(atomspace, stimulateAtoms=True)

            self.chainer.add_rule(rules.InversionRule(self.chainer))
            self.chainer.add_rule(rules.DeductionRule(self.chainer))

        # incredibly exciting futuristic display!
        #import os
        #os.system('cls' if os.name=='nt' else 'clear')

        try:
            result = self.chainer.forward_step()
            if result:
                (output, inputs) = result

                #print '==== Inference ===='
                #print output,str(output.av),'<=',' '.join(str(i)+str(output.av) for i in inputs)

                #print
                #print '==== Attentional Focus ===='
                #for atom in get_attentional_focus(atomspace)[0:30]:
                #    print str(atom), atom.av

                print '==== Result ===='
                print output
                print '==== Trail ===='
                print_atoms( self.chainer.trails[output] )
            else:
                print 'Invalid inference attempted'
        except Exception, e:
            print e

def print_atoms(atoms):
    for atom in atoms:
        print atom

'''
# test it with forgetting, updating and diffusion
scm-eval (load-scm-from-file "../wordpairs.scm")
loadpy pln
agents-start pln.ForwardInferenceAgent opencog::ForgettingAgent opencog::ImportanceUpdatingAgent opencog::ImportanceDiffusionAgent
'''
