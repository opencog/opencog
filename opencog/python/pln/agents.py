from opencog.cogserver import MindAgent

from pln.chainers import Chainer, get_attentional_focus
from pln.rules import rules

class ForwardInferenceAgent(MindAgent):
    def run(self, atomspace):
        chainer = Chainer(atomspace, stimulateAtoms=True)

        chainer.add_rule(rules.InversionRule(chainer))
        chainer.add_rule(rules.DeductionRule(chainer))

        # incredibly exciting futuristic display!
        #import os
        #os.system('cls' if os.name=='nt' else 'clear')

        (output, inputs) = chainer.forward_step()
        print output,str(output.av),'<=',' '.join(str(i)+str(output.av) for i in inputs)

        print
        print '==== Attentional Focus ===='
        for atom in get_attentional_focus(atomspace)[0:30]:
            print str(atom), atom.av

'''
# test it with forgetting, updating and diffusion
scm-eval (load-scm-from-file "../wordpairs.scm")
loadpy pln
agents-start pln.ForwardInferenceAgent opencog::ForgettingAgent opencog::ImportanceUpdatingAgent opencog::ImportanceDiffusionAgent
'''
