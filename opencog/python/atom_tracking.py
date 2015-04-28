from opencog.atomspace import AtomSpace, types, Atom, TruthValue, get_type_name
import opencog.cogserver

class AtomTrackingMindAgent(opencog.cogserver.MindAgent):
    new_atoms_this_cycle = []

    def __init__(self):
        self.cycles = 0

    def run(self,atomspace):
        self.cycles += 1

        AtomTrackingMindAgent.new_atoms_this_cycle = []

        next_atom = atomspace.next_new_atom()
        while next_atom != None:
            self.new_atoms_this_cycle.append(next_atom)
            next_atom = atomspace.next_new_atom()
