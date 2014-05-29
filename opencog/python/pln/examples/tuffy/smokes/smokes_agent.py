"""
PLN representation of the "smokes" sample from Tuffy Markov Logic Networks

More details on this sample are available here:
https://github.com/opencog/opencog/tree/master/opencog/python/pln/examples/tuffy/smokes
https://github.com/cosmoharrigan/tuffy/tree/master/samples/smoke
http://hazy.cs.wisc.edu/hazy/tuffy/doc/tuffy-manual.pdf

Instructions:
Method 1 -- Running the example in a standalone Python environment:
- Run smokes_example.py

Method 2 -- Running the example within the cogserver:
- Add the module path to your PYTHON_EXTENSION_DIRS in opencog.conf:
  ../opencog/python/pln/examples/tuffy/smokes
- Run the cogserver
- Load these files into the cogserver:
  python/pln/examples/tuffy/smokes/smokes.scm,
  python/pln/examples/tuffy/smokes/extra-data.scm
- Run these commands in the cogserver:
  loadpy smokes_agent
  agents-start smokes_agent.InferenceAgent
- Use the Scheme shell to monitor the inference progress
"""

from opencog.cogserver import MindAgent
from opencog.atomspace import types
from pln.chainers import Chainer
from pln.rules import *

__author__ = 'Cosmo Harrigan'


class InferenceAgent(MindAgent):
    def __init__(self):
        self.chainer = None

    def create_chainer(self, atomspace, stimulate_atoms=True):
        self.chainer = Chainer(atomspace,
                               agent=self,
                               stimulateAtoms=stimulate_atoms,
                               allow_output_with_variables=False,
                               preferAttentionalFocus=True,
                               delete_temporary_variables=True)

        # ModusPonens:
        # Implication smokes(x) cancer(X)
        # smokes(Anna)
        # |= cancer(Anna)
        self.chainer.add_rule(
            ModusPonensRule(self.chainer, types.ImplicationLink))

    def run(self, atomspace):
        if self.chainer is None:
            self.create_chainer(atomspace)
            return

        if not check_result(atomspace):
            result = self.chainer.forward_step()
            return result


def check_result(atomspace):
    """
    Searches for EvaluationLinks where the first argument is: PredicateNode
    "cancer" and the target of the predicate is a ConceptNode (representing a
    person)
    """
    eval_links = atomspace.get_atoms_by_type(types.EvaluationLink)

    num_results = 0
    for eval_link in eval_links:
        out = [atom for atom in atomspace.get_outgoing(eval_link.h)
               if atom.is_a(types.PredicateNode) and atom.name == "cancer"]
        if out:
            list_link = atomspace.get_outgoing(eval_link.h)[1]
            argument = atomspace.get_outgoing(list_link.h)[0]
            if argument.is_a(types.ConceptNode):
                num_results += 1

    return num_results == 4
