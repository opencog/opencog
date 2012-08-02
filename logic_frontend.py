import rpyc
from opencog.atomspace import Atom, Handle, TruthValue
from tree import tree_from_atom, atom_from_tree, FakeAtom, Tree
import tree

# Attempt to run PLN code on PyPy using RPyC. Due to use of Cython it was quite messy (not transparent).
# Very fast, but mostly due to using FakeAtom anyway. There are still hard-to-find bugs and I think the easiest
# way is to just use pure Python wherever possible, rather than trying to use a transparent library that is not
# transparent in practice (due to conflicts with Cython).

def rpyc_bc(atomspace, target):
    conn = rpyc.classic.connect('localhost', 18813)
    c = conn.modules['logic_backend'].Chainer(atomspace)
    print c.rules
    result_tree_list = c.bc(target)
    
    result_realtree_list = [tree_with_real_atoms(tr, atomspace) for tr in result_tree_list]
    tree.Atom = Atom
    result_Atom_list = [atom_from_tree(tr, atomspace) for tr in result_realtree_list]    

    for (tr, atom) in zip(result_tree_list, result_Atom_list):
        for tv in c.get_tvs(tr):
            atom.tv = TruthValue(tv.mean, tv.count)

    result_Handle_list = [tr.h for tr in result_Atom_list]
    return result_Handle_list

def tree_with_real_atoms(tr, a):
    #if isinstance(tr.op, Atom):
    if isinstance(tr.op, FakeAtom):
        return Atom(Handle(tr.op._handle_value), a)
    elif tr.is_leaf():
        return tr
    else:
        return Tree(tr.op, [tree_with_real_atoms(child, a) for child in tr.args])
