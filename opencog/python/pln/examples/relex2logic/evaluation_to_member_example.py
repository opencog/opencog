"""
For running evaluation_to_member_agent.py without the cogserver
"""

from __future__ import print_function
from pln.examples.relex2logic import evaluation_to_member_agent
from opencog.atomspace import types, AtomSpace, TruthValue
from data import *
from pln.examples.interactive_agent import InteractiveAgent

__author__ = 'Cosmo Harrigan'

agent = InteractiveAgent(atomspace=atomspace,
                         agent=evaluation_to_member_agent.EvaluationToMemberAgent(),
                         num_steps=10,
                         print_starting_contents=True)
agent.run()
