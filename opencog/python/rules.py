try:
    from opencog.atomspace import TruthValue, confidence_to_count, types as t
except ImportError:
    from atomspace_remote import TruthValue, types as t, confidence_to_count, types as t

import formulas
from tree import *


def rules(a):
    rules = []

    # All existing Atoms
    for obj in a.get_atoms_by_type(t.Atom):
        # POLICY: Ignore all false things. This means you can never disprove something! But much more useful for planning!
        if obj.tv.count > 0 and obj.tv.mean > 0:
            tr = tree_from_atom(obj)
            # A variable with a TV could just prove anything; that's evil!
            if not tr.is_variable():
                
                # tacky filter
                if 'CHUNK' in str(tr):
                    continue
                
                r = Rule(tr, [], '[axiom]')
                r.tv = obj.tv
                rules.append(r)

    ## Deduction
    #for type in self.deduction_types:
    #    rules.append(Rule(T(type, 1,3), 
    #                                 [T(type, 1, 2),
    #                                  T(type, 2, 3), 
    #                                  Var(1),
    #                                  Var(2), 
    #                                  Var(3)],
    #                                name='Deduction', 
    #                                formula = formulas.deductionSimpleFormula))
    #
    ## Inversion
    #for type in self.deduction_types:
    #    rules.append(Rule( T(type, 2, 1), 
    #                                 [T(type, 1, 2),
    #                                  Var(1),
    #                                  Var(2)], 
    #                                 name='Inversion', 
    #                                 formula = formulas.inversionFormula))

    # ModusPonens
    for type in ['ImplicationLink']:
        rules.append(Rule(Var(2), 
                                     [T(type, 1, 2),
                                      Var(1) ], 
                                      name='ModusPonens', 
                                      formula = formulas.modusPonensFormula))

#       # MP for AndLink as a premise
#        for type in ['ImplicationLink']:
#            for size in xrange(5):
#                args = [new_var() for i in xrange(size+1)]
#                andlink = T('AndLink', args)
#
#                rules.append(Rule(Var(2), 
#                                             [T(type, andlink, 2),
#                                              andlink ], 
#                                              name='TheoremRule'))
    
   # ModusPonens for EvaluationLinks only
#        for type in ['ImplicationLink']:
#            conc = T('EvaluationLink', new_var(), new_var())
#            prem = T('EvaluationLink', new_var(), new_var())
#            imp = T('ImplicationLink', prem, conc)
#            
#            rules.append(Rule(conc, 
#                                         [imp, prem], 
#                                          name='ModusPonens_Eval'))

#        for type in ['ImplicationLink']:
#            conc = T('EvaluationLink', a.add_node(t.PredicateNode, 'B'))
#            prem = T('EvaluationLink', a.add_node(t.PredicateNode, 'A'))
#            imp = T('ImplicationLink', prem, conc)
#            
#            rules.append(Rule(conc, 
#                                         [imp, prem], 
#                                          name='ModusPonens_AB'))

    # AND/OR
    type = 'AndLink'
    for size in xrange(5):                
        args = [new_var() for i in xrange(size+1)]
        rules.append(Rule(T(type, args),
                           args,
                           type[:-4], 
                           formula = formulas.andSymmetricFormula))

    type = 'OrLink'
    for size in xrange(2):
        args = [new_var() for i in xrange(size+1)]
        rules.append(Rule(T(type, args),
                           args,
                           type[:-4], 
                           formula = formulas.orFormula))

    # Adding a NOT
    rules.append(Rule(T('NotLink', 1),
                       [ Var(1) ],
                       name = 'Not', 
                       formula = formulas.notFormula))

    # Link conversion
    rules.append(Rule(T('InheritanceLink', 1, 2),
                       [ T('SubsetLink', 1, 2) ],
                       name = 'SubsetLink=>InheritanceLink', 
                       formula = formulas.ext2InhFormula))

    # In planning, assume that an ExecutionLink (action) will be performed
    r = Rule(T('ExecutionLink', 1, 2),
                       [],
                       name = 'PerformAction')
    r.tv = TruthValue(1.0,confidence_to_count(1.0))
    rules.append(r)

#        # Producing ForAll/Bind/AverageLinks?
#        for type in ['ForAllLink', 'BindLink', 'AverageLink']:
#            rules.append(Rule(T(type, 1, 2),
#                               [ Var(2) ],
#                               name = type+' abstraction', 
#                               formula = formulas.identityFormula))

    # This may cause weirdness with things matching too eagerly...
#       # Both of these rely on the policy that tree_from_atom replaces VariableNodes in the AtomSpace with the variables the tree class uses.
#        fact = new_var()
#        list_link = new_var()
#        r = Rule(
#                        fact,
#                        [T('ForAllLink', list_link, fact )], 
#                        name = 'ForAll'     
#                    )
#        r.tv = True
#        rules.append(r)

    for atom in a.get_atoms_by_type(t.AverageLink):
        # out[0] is the ListLink of VariableNodes, out[1] is the expression
        tr = tree_from_atom(atom.out[1])
        r = Rule(tr, [], name='Average')
        r.tv = atom.tv
        rules.append(r)

    return rules

class Rule :
    def __init__ (self, head, goals, name, tv = TruthValue(0, 0), formula = None):
        self.head = head
        self.goals = goals

        self.name = name
        self.tv = tv
        self.formula = if_(formula, formula, formulas.identityFormula)

        #self.bc_depth = 0

    def __str__(self):
        return self.name

#    def __repr__ (self) :
#        rep = str(self.head)
#        sep = " :- "
#        for goal in self.goals :
#            rep += sep + str(goal)
#            sep = ","
#        return rep

    def __repr__ (self) :
        rep = self.name + ' '  + str(self.head) + ' ' + str(self.tv)
        #rep += ' '*self.bc_depth*3
        rep += '\n'
        for goal in self.goals :
            #rep += ' '*(self.bc_depth*3+3)
            rep += str(goal) + '\n'
        return rep

    def standardize_apart(self):
        head_goals = (self.head,)+tuple(self.goals)
        tmp = standardize_apart(head_goals)
        new_version = Rule(tmp[0], tmp[1:], name=self.name, tv = self.tv, formula=self.formula)

        return new_version

    def isomorphic(self, other):
        # One way: make conjunctions out of the rules to make
        # sure variable renamings are consistent across both
        # conclusion and premises
        self_conj = (self.head,)+tuple(self.goals)
        other_conj = (other.head,)+tuple(other.goals)

        return isomorphic_conjunctions_ordered(self_conj, other_conj)

    def canonical_tuple(self):
        try:
            return self._tuple
        except:
            conj = (self.head,)+tuple(self.goals)
            self._tuple = tuple(canonical_trees(conj))
            return self._tuple

    def unifies(self, other):
        self_conj = (self.head,)+tuple(self.goals)
        other_conj = (other.head,)+tuple(other.goals)

        return unify(self_conj, other_conj, {}) != None

    def subst(self, s):
        new_head = subst(s, self.head)
        new_goals = list(subst_conjunction(s, self.goals))
        new_rule = Rule(new_head, new_goals, name=self.name, tv = self.tv, formula = self.formula)
        return new_rule

#    def category(self):
#        '''Returns the category of this rule. It can be either an axiom, a PLN Rule (e.g. Modus Ponens), or an
#        application. An application is a PLN Rule applied to specific arguments.'''
#        if self.name == '[axiom]':
#            return 'axiom'
#        elif self.name.startswith('[application]'):
#            return 'application'
#        else:
#            return 'rule'
