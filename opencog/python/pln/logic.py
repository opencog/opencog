from opencog.atomspace import types, Atom

class Logic(object):
    '''A base class for chainers or other logical sytems. Contains various logical functions
       inspired by the AIMA chapter on first-order logic. They all operate directly on Atoms.'''
    def __init__(self, atomspace):
        self._atomspace = atomspace
        self._new_var_counter = 10**6 # arbitrary

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

    def new_variable(self):
        self._new_var_counter += 1
        
        var_name = '$pln_var_'+str(self._new_var_counter)
        return self._atomspace.add_node(types.VariableNode, var_name)

    def find(self, template, atoms):
        assert(isinstance(atoms, list))

        return [atom for atom in atoms if self.unify_together(atom, template)]

    def unify_together(self, x, y):
        return self.unify(x, y) != None

    def standardize_apart(self, atom, dic=None):
        '''Create a new link where all the variables in the link are replaced with new variables'''
        assert isinstance(atom, Atom)

        # every time $v1 appears in the original expression, it must be replaced with the SAME $v1001
        if dic == None:
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
            return self.change_outgoing(atom, outgoing)

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
#        elif x.is_node() != y.is_node():
#            # One is a Node and the other is a Link
#            return None
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
                return self._unify_outgoing(x.out, y.out, substitution)
        else:
            return None

    def _unify_outgoing(self, x, y, substitution):
        # Try to unify the first argument of x with the first argument of y, then recursively
        # do the rest.
        if len(x) == 0:
            return substitution
        else:
            s_one_arg = self.unify(x[0], y[0], substitution)
            return self._unify_outgoing(x[1:], y[1:], s_one_arg)

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
        '''Returns a new link with the same type as @link but a different outgoing set'''
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
        tv = TruthValue(atom.tv.mean, atom.tv.count)

        if atom.is_node():
            return new_atomspace.add_node(atom.type, atom.name, tv=tv)
        else:
            outgoing = [self.transfer_atom(new_atomspace, out) for out in atom.out]
            return new_atomspace.add_link(atom.type, outgoing, tv=tv)

    def _all_nonzero_tvs(self, atom_list):
        return all(atom.tv.count > 0 for atom in atom_list)

