
from __future__ import print_function
from pprint import pprint
# from pln.examples.deduction import deduction_agent
from opencog.atomspace import types, AtomSpace, TruthValue
from opencog.cogserver_type_constructors import *
from agents.hobbs import HobbsAgent
from agents.dumpAgent import dumpAgent
from opencog.scheme_wrapper import load_scm,scheme_eval_h, scheme_eval, __init__

__author__ = 'Hujie Wang'

'''
This agent is purely for testing purposes, which can be used to test hobbsAgent in a standalone atomspace environment.
'''

atomspace = AtomSpace()
# __init__(atomspace)
scheme_eval(atomspace,  "(use-modules (opencog) (opencog exec))")
scheme_eval(atomspace,  "(use-modules (opencog nlp))")
scheme_eval(atomspace,  "(use-modules (opencog nlp oc))")

#status2 = load_scm(atomspace, "opencog/nlp/anaphora/tests/atomspace.scm")

#init=initAgent()
#init.run(atomspace)
dump=dumpAgent()
dump.run(atomspace)
hobbsAgent = HobbsAgent()
hobbsAgent.run(atomspace)
scheme_eval(atomspace, 'getWords')
