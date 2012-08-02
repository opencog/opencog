'''
Created on 18 Jun, 2012

@author: keyvan

'''
class EvolutionaryConceptFormationAgent(opencog.cogserver.MindAgent):

    def run(self,atomspace): 
        nodes = atomspace.get_atoms_by_type(types.Node)