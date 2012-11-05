try:
    from opencog.atomspace import TruthValue, confidence_to_count, types as t
except ImportError:
    from atomspace_remote import TruthValue, types as t, confidence_to_count, types as t

import formulas
from tree import *
import math

def evaluation_link_template(predicate = None, arguments = None):
    if predicate == None:
        predicate = new_var()
    if arguments == None:
        args_tree = new_var()
    else:
        args_tree = T('ListLink', arguments)
    
    return T('EvaluationLink',
                predicate,
                args_tree)

def actionDone_template(atomspace):
    return evaluation_link_template(
                            atomspace.add(t.PredicateNode, name='actionDone'),
                            [new_var()]
                        )

def rules(a, deduction_types):
    '''This function sets up all the Rules available in PLN. The concept of a Rule is the same as
    in higher-order logic. For example, Modus Ponens requires an ImplicationLink $1->$2 as well as the
    premise of the ImplicationLink ($1) and will produce $2. Every Rule comes with its own formula
    for calculating TruthValues. PLN works by applying a series of Rules to some Atoms to create new
    ones.'''
    
    # This function creates a list of Rules using the Rule constructor.
    # See the Rule class for more explanation of the arguments used.
    # In every step of chaining, the backward chainer has a target Atom (usually with some variables)
    # it is trying to find. It will apply any Rule whose head unifies with the target.
    # The function is called every time a Chainer is constructed.
    rules = []

    #path_rules(a)

    # PLN is able to do pathfinding (experimental). In the Minecraft world, there is a 3D grid
    # similar to that used by A*. This Rule determines the grid squares (or rather grid cubes)
    # that are next to a specified grid cube (i.e. the neighborhood, in the A* sense).
    # You can represent the grid as a network of OpenCog links, which is more suitable for PLN
    # than having an array (and more versatile in general).
    # This version of the Rule calculates the neighborhood dynamically. The function path_rules()
    # uses the simpler but much slower approach of adding thousands of links to the AtomSpace in advance.
    r = Rule(T('EvaluationLink',
                a.add(t.PredicateNode,'neighbor'),
                T('ListLink',
                    Var(1), # a ConceptNode representing the coordinates in a tacky format
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

    # You can add a separate Rule for every axiom (i.e. every Atom which has a TruthValue when PLN starts).
    # It won't add a new Rule for atoms that are created during the inference, rather they will be added to
    # the Proof DAG. It's simple to have a separate Rule for each axiom, but it's slow because the chainer
    # will try to unify a target against every axiom Rule.
    #for obj in a.get_atoms_by_type(t.Atom):
    #    # POLICY: Ignore all false things. This means you can never disprove something! But much more useful for planning!
    #    if obj.tv.count > 0 and obj.tv.mean > 0:
    #        tr = tree_from_atom(obj)
    #        # A variable with a TV could just prove anything; that's evil!
    #        if not tr.is_variable():
    #            
    #            # tacky filter
    #            if 'CHUNK' in str(tr):
    #                continue
    #            
    #            r = Rule(tr, [], '[axiom]', tv = obj.tv)
    #            rules.append(r)

    # Just lookup the rule rather than having separate rules. Would be faster
    # with a large number of atoms (i.e. more scalable). Some examples will break if
    # you use it due to bugs in the backward chainer.
    r = Rule(Var(123),[],
                      name='Lookup',
                      match=match_axiom_slow)
    rules.append(r)

    # A simple example Rule to test the mechanism. You're allowed to add a Rule which calls
    # any Python function to decide one of the Atoms. This one does the plus calculation.
    # The first two variables are the numbers to be added, and the third variable is the result.
    # i.e. the last variable is the return-value for the '+' function. This is a common pattern in
    # Prolog.
    r = Rule(T('EvaluationLink',
               a.add(t.PredicateNode,'+'),
               T('ListLink',
                 Var(1),
                 Var(2),
                 Var(3))),
             [],
             name='PredicateEvaluation',
             match=match_predicate)
    rules.append(r)

    # The three main logical rules in PLN. The code creates different versions of the Rule
    # for different kinds of Links.
    # The classic Modus Ponens rule, used (and over-emphasized) in classical logic everywhere.
    for ty in ['ImplicationLink', 'PredictiveImplicationLink']:
        rules.append(Rule(Var(2), 
                                     [T(ty, 1, 2),
                                      Var(1) ], 
                                      name='ModusPonens '+ty,
                                      formula = formulas.modusPonensFormula))
    
    # The PLN DeductionRule. Not to be confused with ModusPonens.
    for type in deduction_types:
        rules.append(Rule(T(type, 1,3), 
                                     [T(type, 1, 2),
                                      T(type, 2, 3), 
                                      Var(1),
                                      Var(2), 
                                      Var(3)],
                                    name='Deduction', 
                                    formula = formulas.deductionSimpleFormula))
    
    # PLN InversionRule, which reverses an ImplicationLink. It's based on Bayes' Theorem.
    for type in deduction_types:
        rules.append(Rule( T(type, 2, 1), 
                                     [T(type, 1, 2),
                                      Var(1),
                                      Var(2)], 
                                     name='Inversion', 
                                     formula = formulas.inversionFormula))

    # Calculating logical And/Or/Not. These have probabilities attached,
    # but they work similarly to the Boolean versions.
    type = 'AndLink'
    for size in xrange(5):                
        args = [new_var() for i in xrange(size+1)]
        rules.append(Rule(T(type, args),
                           args,
                           type[:-4], 
                           formula = formulas.andSymmetricFormula))
    
    type = 'OrLink'
    for size in xrange(1,2):
        args = [new_var() for i in xrange(size+1)]
        rules.append(Rule(T(type, args),
                           args,
                           type[:-4], 
                           formula = formulas.orFormula))
    
    # Calculate (NotLink A) using A. There's currently no Rule to find A using (NotLink A)
    rules.append(Rule(T('NotLink', 1),
                       [ Var(1) ],
                       name = 'Not', 
                       formula = formulas.notFormula))
    
    # PLN's heuristic Rules to convert one kind of link to another. There are other
    # variations on this Rule defined in the PLN book, but not implemented yet.
    rules.append(Rule(T('InheritanceLink', 1, 2),
                       [ T('SubsetLink', 1, 2) ],
                       name = 'SubsetLink=>InheritanceLink', 
                       formula = formulas.ext2InhFormula))

    # Used by planning. An ExecutionLink indicates an action being performed, so we can
    # assume that the action will be performed as part of the plan (i.e. if any action
    # occurs in the plan, set the TV to full probability and confidence).
    #r = Rule(T('ExecutionLink', 1, 2),
    #                   [],
    #                   name = 'PerformAction',
    #                   tv = TruthValue(1.0,confidence_to_count(1.0)))
    #rules.append(r)
    # TOdo this should be actionSuccess not just actionDone
    r = Rule(actionDone_template(a),
                [],
                name = 'PerformAction',
                tv = TruthValue(1.0,confidence_to_count(1.0)))
    rules.append(r)
    
    # If something is true at the current time, you can use it as the start of the plan.
    # First find the latest time (hack)
    current_time = 0
    for time in a.get_atoms_by_type(t.TimeNode):
        timestamp = int(time.name)
        if timestamp > current_time:
            current_time = timestamp
    current_time_node = a.add(t.TimeNode, str(current_time))
    # Then create the Rule
    # It's essential to store the template so it will have the same variables in both
    # the head and the goal
    template = evaluation_link_template()
    r = Rule(template,
                [T('AtTimeLink', current_time_node, template)],
                name = 'AtCurrentTime'
                )
    rules.append(r)

#        # Producing ForAll/Bind/AverageLinks.
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

    # If an Atom is available in an Average/ForAll quantifier, you want to be
    # able to produce the Atom itself and send it through the other steps of the
    # inference.
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

    rules += temporal_rules(a)

    # Return every Rule specified above.
    return rules

def temporal_rules(atomspace):
    rules = []
    rules.append(Rule(T('BeforeLink', 1, 2),
                        [ ],
                        name='Before',
                        match = create_temporal_matching_function(formulas.beforeFormula)
                        )
                )
    return rules

def lookup_times(tree, atomspace):
    '''For the specified Tree (which can contain a Node or Link), search for
    all the AtTimeLinks. Return a dictionary from timestamp (as a string) to
    a float, representing the fuzzy truth value. (Ignore confidence which is
    probably OK.)'''
    template = T('AtTimeLink',
                 Var(0),
                 tree)
    attimes = find(template, atomspace.get_atoms_by_type(t.AtTimeLink))
    
    dist = {}
    for link in attimes:
        assert isinstance(link, Atom)
        time = link.out[0].name
        fuzzy_tv = link.out[1]
        dist[time] = fuzzy_tv
    
    return dist

def create_temporal_matching_function(formula):
    def match_temporal_relationship(space,target):
        assert isinstance(target, Tree)
        assert not target.args[0].is_variable()
        assert not target.args[1].is_variable()
        
        distribution_event1 = lookup_times(target.args[0], space)
        distribution_event2 = lookup_times(target.args[1], space)
        
        strength = formula(distribution_event2, distribution_event2)
        
        # I'm not sure what to choose for this
        count = 1
        tv = TruthValue(strength, count)
        
        return [(target,tv)]

    return match_temporal_relationship

#def path_rules(a):
#    # These are some tacky and inefficient (but conceptually interesting) pathfinding experiments.
#    template =  T('LatestLink',
#                    T('AtTimeLink',
#                        Var(-1),
#                        T('EvaluationLink',
#                            T(a.add(t.PredicateNode, 'AGISIM_position')),
#                            T('ListLink',
#                                Var(0),
#                                Var(1),
#                                Var(2),
#                                Var(3)
#                            )
#                        )
#                    )
#                )
#    
#    #print template
#    
#    def change_position(coords):
#        s = {Var(i+1):T(a.add(t.NumberNode, str(c))) for (i, c) in enumerate(coords)}
#        return subst(s, template)
#    
#    def get_position(tr):
#        s = unify(template, tr, {})
#        return tuple(float(s[Var(i)].op.name) for i in [1,2,3])
#    
#    def get_name(tr):
#        s = unify(template, tr, {})
#        return s[Var(0)].op.name
#    
#    def is_block(tr):        
#        return get_name(tr).startswith('id_CHUNK')
#    
#    lls = a.get_atoms_by_type(t.LatestLink)
#    pls = find_tree(template,lls)
#    #print len(pls)
#    
#    taken_grids = set()
#    # Find any objects/blocks and assume they are obstacles.
#    for pl in pls:
#        #if is_block(pl):
#        #    continue
#        (x,y,z) = get_position(pl)
#        #print (x,y,z), get_name(pl)
#        (x,y,z) = tuple(map(math.floor,(x,y,z)))
#        #print (x,y,z), get_name(pl)
#        taken_grids.add((x,y,z))
#    
#    # add in the ground blocks.
#    ground = []
#    z = 98
#    for x in xrange(0,100):
#        for y in xrange(0,100):
#            #p1 = change_position((x,y,z))
#            #ground.append(p1)
#            p1c = (x,y,z)
#            
#            if (x,y,z+1) in taken_grids:
#                #print 'occupied'
#                continue
#            
#            ns = [(x-1,y,z),(x+1,y,z),(x,y-1,z),(x,y+1,z)] #,(x,y,z-1),(x,y,z+1)]
#            ns += [(x,y,z+1) for n in ns]
#            for n in ns:
#                #p2 = change_position((i,j,z))
#                
#                #r = T('ImplicationLink',
#                #    a.add(t.ConceptNode,'at %s' % str(p1c)),
#                #    a.add(t.ConceptNode,'at %s' % str(n))
#                #)
#                r = T('EvaluationLink',
#                    a.add(t.PredicateNode,'neighbor'),
#                    T('ListLink',
#                        a.add(t.ConceptNode,'at %s' % str(p1c)),
#                        a.add(t.ConceptNode,'at %s' % str(n))
#                    )
#                )
#                ra = atom_from_tree(r, a)
#                ra.tv = TruthValue(1,confidence_to_count(1.0))
#                #print ra
#    
#    # add in any other surfaces (made of blocks)
#    for pl in pls:
#        if not is_block(pl):
#            continue
#        
#        coords = get_position(pl)
#        #print coords
#        (x,y,z) = coords
#        (x,y,z) = (x-0.5,y-0.5,z)
#        
#        if (x,y,z+1) in taken_grids:
#            print 'occupied'
#            continue
#        
#        ns = [(x-1,y,z),(x+1,y,z),(x,y-1,z),(x,y+1,z)]
#        ns += [(x,y,z+1) for n in ns]
#        #print pl
#        for n in ns:
#            neighbor_block_template = change_position(n)
#            #print neighbor_block_template
#            tmp = find_tree(neighbor_block_template, pls)
#            # Pray there is nothing else at that exact coordinate
#            assert len(tmp) < 2
#            
#            if len(tmp):
#                # Can go straight from one block to another
#                #tr = T('ImplicationLink',
#                #    a.add(t.ConceptNode,'at %s' % str(coords)),
#                #    a.add(t.ConceptNode,'at %s' % str(n))
#                #)
#                tr = T('EvaluationLink',
#                    a.add(t.PredicateNode,'neighbor'),
#                    T('ListLink',
#                        a.add(t.ConceptNode,'at %s' % str(coords)),
#                        a.add(t.ConceptNode,'at %s' % str(n))
#                    )
#                )
#                #print tr
#                atom_from_tree(tr, a).tv = TruthValue(1,confidence_to_count(1.0))
#    
#    return []
#    
#    #for obj in a.get_atoms_by_type(t.ObjectNode):
#    #    if not obj.name.startswith('id_CHUNK'):
#    #        continue
#    #    
#    #    block = T(obj)

# See how they are used in rules() above.

# The slow one but still used because match_axiom still has bugs
def match_axiom_slow(space,target):
    if isinstance(target.op, Atom):
        candidates = [target.op]
    else:
        candidates = space.get_atoms_by_type(target.get_type())
    
    candidates = [c for c in candidates if c.tv.count > 0]
    
    candidate_trees = (tree_from_atom(atom) for atom in candidates)
    candidate_tvs = (c.tv for c in candidates)
    
    return zip(candidate_trees, candidate_tvs)

def match_axiom(space,target):
    if isinstance(target.op, Atom):
        candidates = [target.op]
    else:
        # The nodes in the target, as Atoms
        nodes = [tr.op for tr in target.flatten() if isinstance(tr.op, Atom) and tr.op.is_node()]

        if not len(nodes):
            return match_axiom_slow(target)
        
        # TODO choose the node with the smallest incoming set.
        n = nodes[0]
        candidates = find_links_upward(n)
        
    candidate_trees = (tree_from_atom(atom) for atom in candidates)
    candidate_tvs = (c.tv for c in candidates)
    
    return zip(candidate_trees, candidate_tvs)

def find_links_upward(atom):
    level = [link for link in atom.incoming if link.tv.count > 0]
    next_level = concat_lists([find_links_upward(link) for link in atom.incoming])
    
    return level + next_level

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
    #print 'match_predicate',target
    ll = target.args[1]
    #print ll
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

## @} 
class Rule :

    def __init__ (self, head, goals, name, tv = TruthValue(0, 0),
                  formula = None, match = None):
        '''@head is a Tree representing the structure of Atom (usually a Link)
    to be produced by this Rule. If it's a variable then any kind of Atom
    can be produced.
    @goals (list of Trees) specifies what kinds of Atoms
    are necessary to produce it. There should be variables used in the head
    that also appear in the goals. This means they will be the same atom.
    It's OK to reuse the same variable numbers in different Rules - they'll
    be converted to unique variables automatically.
    The @name of the Rule is just for logging/visualizing the inference process.
    You can use either @formula or @tv. If you specify a formula method from formulas.py
    it will be called with the TVs of the relevant Atoms, to calculate the TruthValue for
    the resulting Atom. If you set the tv parameter of the Rule, it will always use that TV.
    @match lets you use custom code to decide which atoms are produced (or used). The API for that
    is still in flux so not documented.'''
        self.head = head
        self.goals = goals

        self.name = name
        self.tv = tv
        self.match = match
        self.formula = if_(formula, formula, formulas.identityFormula)
        # Dingjie's tracing experiments
        self.path_pre = None
        self.path_axiom = None

        if name == 'Lookup':
            assert len(goals) == 0

        #self.bc_depth = 0

    def __str__(self):
        #rep = self.name + ' '  + str(self.head) + ' ' + str(self.tv)
        rep = self.name + ' '  + str(self.head) 
        rep += ' :- '
        for goal in self.goals :
            #rep += ' '*(self.bc_depth*3+3)
            rep += str(goal) + ','
        return rep

    def __repr__ (self) :
        rep = self.name + ' '  + str(self.head) + ' ' + str(self.tv)
        rep += ' :- '
        for goal in self.goals :
            #rep += ' '*(self.bc_depth*3+3)
            rep += str(goal) + ','
        return rep

    def standardize_apart(self):
        '''Create a new version of the Rule where both the head and goals use new
        variables. Important for unification.'''
        head_goals = (self.head,)+tuple(self.goals)
        tmp = standardize_apart(head_goals)
        new_version = Rule(tmp[0], tmp[1:], name=self.name, tv = self.tv,
                           formula=self.formula, match = self.match)

        return new_version

    def isomorphic(self, other):
        '''Check if both self and other are the same. Renaming a variable doesn't affect it.
        It's possible for the same target to be produced multiple times by the backward chainer,
        so this enables you to reuse the existing one.'''
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
	'''return true if could be unified to tree other '''
        self_conj = (self.head,)+tuple(self.goals)
        other_conj = (other.head,)+tuple(other.goals)

        return unify(self_conj, other_conj, {}) != None

    def subst(self, s):
	'''Create a new Rule where substitution s has been substituted into both the head and goals.'''
        new_head = subst(s, self.head)
        new_goals = list(subst_conjunction(s, self.goals))
        new_rule = Rule(new_head, new_goals, name=self.name, tv = self.tv,
                        formula = self.formula, match = self.match)
        return new_rule

