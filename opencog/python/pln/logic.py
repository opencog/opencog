from opencog.atomspace import types, Atom

from itertools import permutations

class Logic(object):
    '''A base class for chainers or other logical sytems. Contains various logical functions
       inspired by the AIMA chapter on first-order logic. They all operate directly on Atoms.'''
    def __init__(self, atomspace):
        self._atomspace = atomspace

    def variables(self, atom):
        '''Find all the variables in an expression (which may be repeated)'''
        if atom.is_node():
            if self.is_variable(atom):
                return [atom]
            else:
                return []
        else:
            result = []
            for o in atom.out:
                result += self.variables(o)
            return result

    def get_first_node(self, atom):
        '''Using a depth first search on the link, return the first Node found. If atom is a Node just return that.'''
        if atom.is_node() and not self.is_variable(atom):
            return atom
        else:
            for o in atom.out:
                ret = self.get_first_node(o)
                if not ret is None:
                    return ret
            return None

    def get_incoming_recursive(self, atom):
        inc = atom.incoming
        ret=[]
        ret+= inc
        for link in inc:
            ret+= self.get_incoming_recursive(link)
        return ret

    def new_variable(self):
        prefix = '$pln_var_'
        return self._atomspace.add_node(types.VariableNode, prefix, prefixed=True)

    def make_n_variables(self, N):
        return [self.new_variable() for i in xrange(0, N)]

    # @todo The AtomSpace has an ImportanceIndex which is much more efficient. This algorithm checks
    # every Atom's STI (in the whole AtomSpace)
    def filter_attentional_focus(self, atoms, attentional_focus_boundary=0):
        attentional_focus = []
        for atom in atoms:
            if atom.av['sti'] > attentional_focus_boundary:
                attentional_focus.append(atom)
        return attentional_focus

    def find(self, template, s={}, useAF=False, allow_zero_tv=False, ground=False):
        if template.type == types.VariableNode:
            root_type = types.Atom
            atoms = self.atomspace.get_atoms_by_type(root_type)
        else:
            # If the atom is a link with all variables below it, then lookup all links of that type
            # If it has any nodes (which aren't VariableNodes!), then lookup the incoming set for that node
            first_node = self.get_first_node(template)
            if first_node is None:
                root_type = template.type
                atoms = self.atomspace.get_atoms_by_type(root_type)
            else:
                print first_node
                atoms = self.get_incoming_recursive(first_node)
                print atoms

        if useAF:
            atoms = self.filter_attentional_focus(atoms)

        if not allow_zero_tv:
            atoms = [atom for atom in atoms if atom.tv.count > 0]

        results = [atom for atom in atoms if self.unify_together(atom, template, s)]
        
        if ground:
            results = [atom for atom in results if len(self.variables(atom)) == 0]

        return results

    def unify_together(self, x, y, s):
        return self.unify(x, y, s) != None

    def standardize_apart(self, atom, dic=None):
        '''Create a new link where all the variables in the link are replaced with new variables. dic creates a mapping of old variables to new ones'''
        assert isinstance(atom, Atom)

        # every time $v1 appears in the original expression, it must be replaced with the SAME $v1001
        if dic is None:
            dic = {}

        if atom.is_node():
            if self.is_variable(atom):
                if atom in dic:
                    return dic[atom]
                else:
                    var = self.new_variable()
                    dic[atom] = var
                    return var
            else:
                return atom
        else: # atom is a link
            outgoing = [self.standardize_apart(a, dic) for a in atom.out]

            sa_link = self.change_outgoing(atom, outgoing)
            return sa_link

    def substitute(self, substitution, atom):
        '''Substitute the substitution s into the expression x.
        Atoms are immutible; this function (like others) returns a new Link
        with variables replaced by their values in @substitution'''
        assert isinstance(substitution, dict)
        assert isinstance(atom, Atom)

        if atom.is_node():
            if self.is_variable(atom):
                value = substitution.get(atom, atom)
                assert isinstance(value, Atom)
                return value
            else:
                return atom
        else:
            outgoing = [self.substitute(substitution, o) for o in atom.out]
            return self.change_outgoing(atom, outgoing)

    def substitute_list(self, substitution, atoms):
        result = []
        for atom in atoms:
            result.append(self.substitute(substitution, atom))
        return result

    def unify(self, x, y, substitution = {}):
        '''Unify atoms x,y with substitution s; return a substitution that
        would make x,y equal, or None if x,y can not unify.'''

        if substitution == None:
            return None
        elif x == y:
            return substitution
        elif self.is_variable(x):
            return self._unify_variable(x, y, substitution)
        elif self.is_variable(y):
            return self._unify_variable(y, x, substitution)
        elif (not x.is_node()) and (not y.is_node()):
            if x.type != y.type:
                return None
            elif len(x.out) != len(y.out):    
                return None
            else:
                return self._unify_outgoing(x, y, substitution)
        else:
            return None

    def _unify_outgoing(self, x, y, substitution):
        assert isinstance(x, Atom)
        assert isinstance(y, Atom)
        if x.is_a(types.OrderedLink):
            return self._unify_outgoing_ordered(x.out, y.out, substitution)
        else:
            return self._unify_outgoing_unordered(x.out, y.out, substitution)

    def _unify_outgoing_ordered(self, x, y, substitution):
        # Try to unify the first argument of x with the first argument of y, then recursively
        # do the rest.
        if len(x) == 0:
            return substitution
        else:
            s_one_arg = self.unify(x[0], y[0], substitution)
            return self._unify_outgoing_ordered(x[1:], y[1:], s_one_arg)

    def _unify_outgoing_unordered(self, x, y, substitution):
        # A simple way to unify two UnorderedLinks
        # Try to unify x with every permutation of y.
        # Choose the first permutation that works (if there is one).
        # TODO handle this case: there is more than one permutation compatible with this expression,
        # but only some of them (because of variables) can be used anywhere else
        # That could only be handled by backtracking in the rest of the unify algorithm (but that's too complex)
        # TODO this may not be the most efficient way. Shouldn't matter for small links though...
        for new_y in permutations(y):
            s = self._unify_outgoing_ordered(x, new_y, substitution)
            if s != None:
                return s
        return None

    def _unify_variable(self, variable, atom, substitution):
        if variable in substitution:
            value = substitution[variable]
            return self.unify(value, atom, substitution)
        elif self._occurs_check(variable, atom, substitution):
            return None
        else:
            return self.add_binding(substitution, variable, atom)

    def _occurs_check(self, variable, atom, substitution):
        '''Return true if variable occurs anywhere in atom
        (or in substitute(substitution, atom), if substitution has a binding for atom).'''
        if variable == atom:
            return True
        elif self.is_variable(atom) and atom in substitution:
            value = substitution[atom]
            return self._occurs_check(variable, value, substitution)
        elif atom.is_node():
            return False
        else:
            # Check if it occurs in any sub-expressions (i.e. outgoing nested links)
            for o in atom.out:
                if self._occurs_check(variable, o, substitution):
                    return True
            return False

        assert False            

    def add_binding(self, substitution, variable, value):
        '''Copy the substitution and extend it by setting variable to value;
        return copy.'''
        s2 = substitution.copy()
        s2[variable] = value
        return s2

    def change_outgoing(self, link, outgoing):
        '''Returns a new link with the same type as @link but a different outgoing set. If you pass the same outgoing set, it will return the same Atom!'''
        return self._atomspace.add_link(link.type, outgoing)

    def is_variable(self, atom):
        return atom.is_a(types.VariableNode)

    # Miscellaneous helper functions

    def link(self, type, out):
        return self._atomspace.add_link(type, out)

    def node(self, type, name):
        return self._atomspace.add_node(type, name)

    def transfer_atom(self, new_atomspace, atom):
        '''transfer (or rather copy) an atom from one atomspace to another. Assumes that both AtomSpaces have the same list of Atom types!
        returns the equivalent of atom in new_atomspace. creates it if necessary, including the outgoing set of links.'''
        # The AtomSpace probably clones the TV objects, and it wouldn't matter much anyway
        #tv = TruthValue(atom.tv.mean, atom.tv.count)

        if atom.is_node():
            return new_atomspace.add_node(atom.type, atom.name, tv=atom.tv)
        else:
            outgoing = [self.transfer_atom(new_atomspace, out) for out in atom.out]
            return new_atomspace.add_link(atom.type, outgoing, tv=atom.tv)

    def _all_nonzero_tvs(self, atom_list):
        return all(atom.tv.count > 0 for atom in atom_list)

