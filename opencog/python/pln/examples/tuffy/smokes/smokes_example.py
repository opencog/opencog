"""
For testing smokes_agent.py without the cogserver
"""

from __future__ import print_function
from pln.examples.tuffy.smokes.smokes_agent import InferenceAgent
from opencog.atomspace import AtomSpace, TruthValue, types
from data import *
from interactive_agent import *

__author__ = 'Cosmo Harrigan'

agent = InteractiveAgent(atomspace=atomspace,
                         agent=InferenceAgent(),
                         num_steps=500,
                         print_starting_contents=True)
agent.run()