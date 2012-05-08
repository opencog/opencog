try:
    from opencog.atomspace import TruthValue, confidence_to_count, types as t
except ImportError:
    from atomspace_remote import TruthValue, types as t, confidence_to_count, types as t

import formulas
from tree import *
import math

def rules(a, deduction_types):
    rules = []

    #path_rules(a)

    r = Rule(T('EvaluationLink',
                a.add(t.PredicateNode,'neighbor'),
                T('ListLink',
                    Var(1),
                    Var(2)
                )),
             [],
             name='Neighbors',
             match=match_neighbors)
    rules.append(r)
    #r = Rule(T('ImplicationLink',
    #            Var(1),
    #            Var(2)
    #            ),
    #         [],
    #         name='Neighbors',
    #         match=match_neighbors)
    #rules.append(r)

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
                
                r = Rule(tr, [], '[axiom]', tv = obj.tv)
                rules.append(r)

    # Just lookup the rule rather than having separate rules. Would be faster
    # with a large number of atoms (i.e. more scalable)
    #r = Rule(Var(123),[],
    #                  name='Lookup',
    #                  match=match_axiom)
    #rules.append(r)

    #r = Rule(T('EvaluationLink',
    #           a.add(t.PredicateNode,'+'),
    #           T('ListLink',
    #             Var(1),
    #             Var(2),
    #             Var(3))),
    #         [],
    #         name='PredicateEvaluation',
    #         match=match_predicate)
    #rules.append(r)
    #
    ## Deduction
    #for type in deduction_types:
    #    rules.append(Rule(T(type, 1,3), 
    #                                 [T(type, 1, 2),
    #                                  T(type, 2, 3), 
    #                                  Var(1),
    #                                  Var(2), 
    #                                  Var(3)],
    #                                name='Deduction', 
    #                                formula = formulas.deductionSimpleFormula))
    #
    # Inversion
    #for type in deduction_types:
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
    
    #type = 'OrLink'
    #for size in xrange(2):
    #    args = [new_var() for i in xrange(size+1)]
    #    rules.append(Rule(T(type, args),
    #                       args,
    #                       type[:-4], 
    #                       formula = formulas.orFormula))
    #
    ## Adding a NOT
    #rules.append(Rule(T('NotLink', 1),
    #                   [ Var(1) ],
    #                   name = 'Not', 
    #                   formula = formulas.notFormula))
    #
    ## Link conversion
    #rules.append(Rule(T('InheritanceLink', 1, 2),
    #                   [ T('SubsetLink', 1, 2) ],
    #                   name = 'SubsetLink=>InheritanceLink', 
    #                   formula = formulas.ext2InhFormula))

    # In planning, assume that an ExecutionLink (action) will be performed
    r = Rule(T('ExecutionLink', 1, 2),
                       [],
                       name = 'PerformAction',
                       tv = TruthValue(1.0,confidence_to_count(1.0)))
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

    for atom in a.get_atoms_by_type(t.ForAllLink):
        # out[0] is the ListLink of VariableNodes, out[1] is the expression
        tr = tree_from_atom(atom.out[1])
        r = Rule(tr, [], name='ForAll')
        r.tv = atom.tv
        rules.append(r)

    return rules

def path_rules(a):
    # Pathfinding experiments
    template =  T('LatestLink',
                    T('AtTimeLink',
                        Var(-1),
                        T('EvaluationLink',
                            T(a.add(t.PredicateNode, 'AGISIM_position')),
                            T('ListLink',
                                Var(0),
                                Var(1),
                                Var(2),
                                Var(3)
                            )
                        )
                    )
                )
    
    print template
    
    def change_position(coords):
        s = {Var(i+1):T(a.add(t.NumberNode, str(c))) for (i, c) in enumerate(coords)}
        return subst(s, template)
    
    def get_position(tr):
        s = unify(template, tr, {})
        return tuple(float(s[Var(i)].op.name) for i in [1,2,3])
    
    def get_name(tr):
        s = unify(template, tr, {})
        return s[Var(0)].op.name
    
    def is_block(tr):        
        return get_name(tr).startswith('id_CHUNK')
    
    lls = a.get_atoms_by_type(t.LatestLink)
    pls = find_tree(template,lls)
    print len(pls)
    
    taken_grids = set()
    # Find any objects/blocks and assume they are obstacles.
    for pl in pls:
        #if is_block(pl):
        #    continue
        (x,y,z) = get_position(pl)
        print (x,y,z), get_name(pl)
        (x,y,z) = tuple(map(math.floor,(x,y,z)))
        print (x,y,z), get_name(pl)
        taken_grids.add((x,y,z))
    
    # add in the ground blocks.
    ground = []
    z = 98
    for x in xrange(0,100):
        for y in xrange(0,100):
            #p1 = change_position((x,y,z))
            #ground.append(p1)
            p1c = (x,y,z)
            
            if (x,y,z+1) in taken_grids:
                print 'occupied'
                continue
            
            ns = [(x-1,y,z),(x+1,y,z),(x,y-1,z),(x,y+1,z)] #,(x,y,z-1),(x,y,z+1)]
            ns += [(x,y,z+1) for n in ns]
            for n in ns:
                #p2 = change_position((i,j,z))
                
                #r = T('ImplicationLink',
                #    a.add(t.ConceptNode,'at %s' % str(p1c)),
                #    a.add(t.ConceptNode,'at %s' % str(n))
                #)
                r = T('EvaluationLink',
                    a.add(t.PredicateNode,'neighbor'),
                    T('ListLink',
                        a.add(t.ConceptNode,'at %s' % str(p1c)),
                        a.add(t.ConceptNode,'at %s' % str(n))
                    )
                )
                ra = atom_from_tree(r, a)
                ra.tv = TruthValue(1,confidence_to_count(1.0))
                #print ra
    
    # add in any other surfaces (made of blocks)
    for pl in pls:
        if not is_block(pl):
            continue
        
        coords = get_position(pl)
        #print coords
        (x,y,z) = coords
        (x,y,z) = (x-0.5,y-0.5,z)
        
        if (x,y,z+1) in taken_grids:
            print 'occupied'
            continue
        
        ns = [(x-1,y,z),(x+1,y,z),(x,y-1,z),(x,y+1,z)]
        ns += [(x,y,z+1) for n in ns]
        #print pl
        for n in ns:
            neighbor_block_template = change_position(n)
            #print neighbor_block_template
            tmp = find_tree(neighbor_block_template, pls)
            # Pray there is nothing else at that exact coordinate
            assert len(tmp) < 2
            
            if len(tmp):
                # Can go straight from one block to another
                #tr = T('ImplicationLink',
                #    a.add(t.ConceptNode,'at %s' % str(coords)),
                #    a.add(t.ConceptNode,'at %s' % str(n))
                #)
                tr = T('EvaluationLink',
                    a.add(t.PredicateNode,'neighbor'),
                    T('ListLink',
                        a.add(t.ConceptNode,'at %s' % str(coords)),
                        a.add(t.ConceptNode,'at %s' % str(n))
                    )
                )
                #print tr
                atom_from_tree(tr, a).tv = TruthValue(1,confidence_to_count(1.0))
    
    return []
    
    #for obj in a.get_atoms_by_type(t.ObjectNode):
    #    if not obj.name.startswith('id_CHUNK'):
    #        continue
    #    
    #    block = T(obj)


def match_axiom(space,target):
    if isinstance(target.op, Atom):
        candidates = [target.op]
    else:
        candidates = space.get_atoms_by_type(target.get_type())
    
    candidates = [c for c in candidates if c.tv.count > 0]
    
    candidate_trees = (tree_from_atom(atom) for atom in candidates)
    candidate_tvs = (c.tv for c in candidates)
    
    return zip(candidate_trees, candidate_tvs)

def match_neighbors(space,target):
    print 'match_neighbors',target
    template = T('EvaluationLink',
                space.add(t.PredicateNode,'neighbor'),
                T('ListLink',
                    Var(1),
                    Var(2)
                )
     )
    s = unify(template, target, {})
    cube1 = s[Var(1)]
    
    #template = T('ImplicationLink',
    #    Var(2),
    #    Var(1)
    #)
    #s = unify(template, target, {})
    #cube1 = s[Var(1)]
    
    def get_coords(expr):
        name = expr.op.name
        tuple_str = name[3:]
        coords = tuple(map(int, tuple_str[1:-1].split(',')))
        return coords

    if cube1.is_variable() or not isinstance(cube1.op, Atom):
        return []
    (x,y,z) = get_coords(cube1)
    
    ns = [(x-1,y,z),(x+1,y,z),(x,y-1,z),(x,y+1,z)]
    # jumping onto a block
    ns += [(x,y,z+1) for n in ns]
    
    tv = TruthValue(1,confidence_to_count(1.0))
    candidates = []
    for n in ns:
        cube2 = Tree(space.add(t.ConceptNode, 'at %s' % str(n)))
        s = {Var(1):cube1, Var(2):cube2}
        cand = subst(s, template)
        candidates.append((cand,tv))

    return candidates

def match_predicate(space,target):
    # Actually it would be much better to use the standard target for this rule
    # + unify to find the places
    print 'match_predicate',target
    ll = target.args[1]
    print ll
    (n1,n2,n3) = ll.args
    
    candidates = []
    if (n1.get_type() == t.NumberNode and n2.get_type() == t.NumberNode):
        if (n3.get_type() == t.NumberNode):
            if int(n1.op.name) + int(n2.op.name) == int(n3.op.name):
                candidates.append((target, TruthValue(1.0,confidence_to_count(1.0))))
        elif (n3.is_variable()):
            addition = int(n1.op.name) + int(n2.op.name)
            s = {n3: Tree(space.add(t.NumberNode,str(addition)))}
            c = subst(s, target)
            print '>>>match_predicate:c',c
            candidates.append((c, None))
    
    return candidates

class Rule :
    def __init__ (self, head, goals, name, tv = TruthValue(0, 0),
                  formula = None, match = None):
        self.head = head
        self.goals = goals

        self.name = name
        self.tv = tv
        self.match = match
        self.formula = if_(formula, formula, formulas.identityFormula)

        if name == 'Lookup':
            assert len(goals) == 0

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
        rep += ' :- '
        for goal in self.goals :
            #rep += ' '*(self.bc_depth*3+3)
            rep += str(goal) + ' //'
        return rep

    def standardize_apart(self):
        head_goals = (self.head,)+tuple(self.goals)
        tmp = standardize_apart(head_goals)
        new_version = Rule(tmp[0], tmp[1:], name=self.name, tv = self.tv,
                           formula=self.formula, match = self.match)

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
        new_rule = Rule(new_head, new_goals, name=self.name, tv = self.tv,
                        formula = self.formula, match = self.match)
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
