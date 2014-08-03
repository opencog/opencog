"""
PLN representation of the "smokes" sample from Tuffy Markov Logic Networks

More details on this sample are available here:
https://github.com/opencog/opencog/tree/master/opencog/python/pln/examples/tuffy/smokes
https://github.com/cosmoharrigan/tuffy/tree/master/samples/smoke
http://hazy.cs.wisc.edu/hazy/tuffy/doc/tuffy-manual.pdf

Instructions:
Method 1 (preferred) -- Running the example with attention allocation using
 a Python control interface
- Follow the instructions here:
  https://github.com/opencog/external-tools/tree/master/attention

Method 2 -- Running the example in a standalone Python environment:
- Run smokes_example.py: python smokes_example.py

Method 3 -- Running the example within the cogserver:
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
from opencog.scheme_wrapper import scheme_eval_h, __init__

__author__ = 'Cosmo Harrigan'

TARGET_STIMULUS = 20


class InferenceAgent(MindAgent):
    def __init__(self):
        self.chainer = None
        self.query = None
        print "Initializing InferenceAgent."

    def create_chainer(self, atomspace, stimulate_atoms=True):
        """
        Creates the chainer for the "smokes" example. Optionally, you can
         define the target query before calling this method, by defining a
         Scheme expression named "query". For example, you can issue the
         Scheme expression "(define query hasCancer)" in reference to the
         predicate defined in smokes.scm before loading this agent. Once
         defined, the target query will receive stimulus at every time step.
         Stimulus requires the agent to be running in a CogServer.
         For a complete example that incorporates this behavior, see
         example.py here:
           https://github.com/opencog/external-tools/tree/master/attention
        """
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

        # stimulateAtoms is only enabled when the agent is ran inside the
        # CogServer, since the functionality requires a CogServer and
        # attention allocation
        if self.chainer._stimulateAtoms:
            self.query = scheme_eval_h(atomspace, "query")

    def run(self, atomspace):
        if self.chainer is None:
            self.create_chainer(atomspace)
            print "PLN Chainer created."
            return

        print "PLN continuing."

        if not check_result(atomspace):
            result = self.chainer.forward_step()

            if self.query is not None:
                # Allow the stimulus amount to be set dynamically by setting
                # a configuration atom in the atomspace
                list = atomspace.get_atoms_by_name(
                    types.PredicateNode, "CONFIG-StimulusAmount")
                if list is not None:
                    # Given the PredicateNode, walk to the NumberNode
                    list = atomspace.get_incoming(list[0].h)  # EvaluationLink
                    list = atomspace.get_outgoing(list[0].h)  # ListLink
                    list = atomspace.get_outgoing(list[1].h)  # NumberNode
                    value = atomspace.get_name(list[0].h)
                    TARGET_STIMULUS = int(value)
                    print "Target stimulus amount updated to {0}".\
                        format(TARGET_STIMULUS)

                self.chainer._give_stimulus(atomspace[self.query],
                                            TARGET_STIMULUS)

            return result


def check_result(atomspace):
    """
    Searches for 4 instances of an EvaluationLink where the first argument is:
      PredicateNode "cancer"
    and the target of the predicate is a ConceptNode (representing a person)
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

    result_found = (num_results == 4)
    print "Result found? {0}. {1} results satisfy the query.".\
        format(result_found, num_results)

    return result_found
