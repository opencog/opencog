from opencog.atomspace import AtomSpace, TruthValue, Atom
from opencog.atomspace import types as t

a = AtomSpace()

TV = TruthValue(1,1)
A = a.add_node(t.ConceptNode, 'A', TV)
B = a.add_node(t.ConceptNode, 'B', TruthValue(0.5,1))
C = a.add_node(t.ConceptNode, 'C', TV)
AB = a.add_link(t.InheritanceLink, [A, B], TV)
BC = a.add_link(t.InheritanceLink, [B, C], TV)
AC = a.add_link(t.InheritanceLink, [A, C])

print AB.incoming


a.print_list()


import logic
from tree import *

target = tree_from_atom(AC)
chainer = logic.Chainer(a)
results = chainer.bc(target)

print '\n---------------------------\n'

print results
print AC.tv