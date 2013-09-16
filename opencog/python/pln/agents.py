from opencog.cogserver import MindAgent

from pln.chainers import Chainer
from pln.rules import rules

class ForwardInferenceAgent(MindAgent):
    def run(self, atomspace):
        chainer = Chainer(atomspace, stimulateAtoms=False)

        chainer.add_rule(rules.InversionRule(chainer))
        chainer.add_rule(rules.DeductionRule(chainer))

        (output, inputs) = chainer.forward_step()
        print output,'<=',' '.join(str(i) for i in inputs)

