from utility.enum import Enum
from opencog.atomspace import AtomSpace, types, Atom, TruthValue
import opencog.cogserver

# @todo opencog/atomspace/AttentionBank.h defines a getAttentionalFocusBoundary method, which should eventually be used instead of this
def get_attentional_focus(atomspace, attentional_focus_boundary=0):
    nodes = atomspace.get_atoms_by_type(types.Node)
    attentional_focus = []
    for node in nodes:
        if node.av.sti > attentional_focus_boundary:
            attentional_focus.append(node)
    return attentional_focus

class HebbianMiningAgent(opencog.cogserver.MindAgent):

    def run(self, atomspace):
        attentional_focus = get_attentional_focus(atomspace)
        for node1 in attentional_focus:
            for node2 in attentional_focus:
                if node1 == node2:
                    continue
                print atomspace.add_link(types.AsymmetricHebbianLink, [node1, node2], TruthValue(0.5, 1))
                #print 'AssymetricHebbianLink between "' + node1.name + '" and "' + node2.name + '" generated'
                #print '\n'

