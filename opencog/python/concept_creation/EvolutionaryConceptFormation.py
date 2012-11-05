'''
Created on 18 Jun, 2012

@author: keyvan

'''

from opencog.atomspace import types
from opencog.cogserver import MindAgent

class EvolutionaryConceptFormationAgent(MindAgent):

    def run(self,atomspace):
        nodes = atomspace.get_atoms_by_type(types.Node)