try:
    from opencog.atomspace import TruthValue, confidence_to_count, types as t
except ImportError:
    from atomspace_remote import TruthValue, types as t, confidence_to_count, types as t

import formulas
import temporalFormulas
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

def actionDone_template(atomspace, execution_link = None):
    if execution_link == None:
        execution_link = new_var()
    return evaluation_link_template(
                            atomspace.add(t.PredicateNode, name='actionDone'),
                            [execution_link]
                        )

def execution_link_template(schema, arguments = None):
    if arguments == None:
        args_tree = new_var()
    else:
        args_tree = T('ListLink', arguments)

    return T('ExecutionLink',
        schema,
        args_tree)


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

#    # PLN is able to do pathfinding (experimental). In the Minecraft world, there is a 3D grid
#    # similar to that used by A*. This Rule determines the grid squares (or rather grid cubes)
#    # that are next to a specified grid cube (i.e. the neighborhood, in the A* sense).
#    # You can represent the grid as a network of OpenCog links, which is more suitable for PLN
#    # than having an array (and more versatile in general).
#    # This version of the Rule calculates the neighborhood dynamically. The function path_rules()
#    # uses the simpler but much slower approach of adding thousands of links to the AtomSpace in advance.
#    r = Rule(T('EvaluationLink',
#                a.add(t.PredicateNode,'neighbor'),
#                T('ListLink',
#                    Var(1), # a ConceptNode representing the coordinates in a tacky format
#                    Var(2)
#                )),
#             [],
#             name='Neighbors',
#             match=match_neighbors)
#    rules.append(r)
#    #r = Rule(T('ImplicationLink',
#    #            Var(1),
#    #            Var(2)
#    #            ),
#    #         [],
#    #         name='Neighbors',
#    #         match=match_neighbors)
#    #rules.append(r)

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
                      match=match_axiom)
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

#    for type in ['AndLink', 'SimultaneousAndLink']:
#        # A rule to create an AndLink from two AndLinks.
#        # There's no point in using size 2 because the premises wouldn't be in AndLinks
#        for totalsize in xrange(4,11):
#            for size_a in xrange(2, totalsize-1):
#                size_b = totalsize - size_a
#
#                vars = [new_var() for x in xrange(totalsize)]
#                and_result = T(type, vars)
#
#                and_a = T(type, vars[0:size_a])
#                and_b = T(type, vars[size_a:totalsize])
#
#                r = Rule(and_result,
#                         [and_a,
#                          and_b],
#                         name = type[:-4]+'Partition %s/%s' % (size_a, size_b),
#                         formula = formulas.andPartitionFormula
#                         )
#                rules.append(r)
#
#        # A rule to create an AndLink from an AndLink and a single premise
#        for totalsize in xrange(3,11):
#            size_b = totalsize-1
#
#            vars = [new_var() for x in xrange(totalsize)]
#            and_result = T(type, vars)
#
#            thing_a = vars[0]
#            and_b = T(type, vars[1:totalsize])
#
#            r = Rule(and_result,
#                [thing_a,
#                 and_b],
#                name = type[:-4]+'AndBuilding %s' % (size_b,),
#                formula = formulas.andPartitionFormula
#            )
#            rules.append(r)

    # PLN's heuristic Rules to convert one kind of link to another. There are other
    # variations on this Rule defined in the PLN book, but not implemented yet.
    rules.append(Rule(T('InheritanceLink', 1, 2),
                       [ T('SubsetLink', 1, 2) ],
                       name = 'SubsetLink=>InheritanceLink', 
                       formula = formulas.ext2InhFormula))

    rules.append(Rule(T('SimilarityLink', 1, 2),
        [ T('InheritanceLink', 1, 2),
          T('InheritanceLink', 2, 1)],
        name = 'Inheritance=>Similarity',
        formula = formulas.inheritance2SimilarityFormula))

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

    rules += logical_rules(a, deduction_types)

    rules += quantifier_rules(a)

    rules += temporal_rules(a)
    
    #rules += planning_rules(a)

    #rules += subset_rules(a)

    # Return every Rule specified above.
    return rules

def logical_rules(atomspace, deduction_types):
    rules = []

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

    return rules

def quantifier_rules(atomspace):
    rules = []

    # If an Atom is available in an Average/ForAll quantifier, you want to be
    # able to produce the Atom itself and send it through the other steps of the
    # inference.
    for atom in atomspace.get_atoms_by_type(t.AverageLink):
        # out[0] is the ListLink of VariableNodes, out[1] is the expression
        tr = tree_from_atom(atom.out[1])
        r = Rule(tr, [], name='Average')
        r.tv = atom.tv
        rules.append(r)

    for atom in atomspace.get_atoms_by_type(t.ForAllLink):
        # out[0] is the ListLink of VariableNodes, out[1] is the expression
        tr = tree_from_atom(atom.out[1])
        r = Rule(tr, [], name='ForAll')
        r.tv = atom.tv
        rules.append(r)

    return rules

def subset_rules(atomspace):
    rules = []

    r = Rule(
        T('SubsetLink', new_var(), new_var()),
        [],
        name = 'SubsetEvaluation',
        match = match_subset
    )
    rules.append(r)

    r = Rule(
        T('IntensionalInheritanceLink', new_var(), new_var()),
        [],
        name = 'IntensionalInheritanceEvaluation',
        match = match_intensional_inheritance
    )
    rules.append(r)

    a,b = new_var(), new_var()
    rules.append(
        Rule(
            T('InheritanceLink', a, b),
            [T('SubsetLink', a, b),
             T('IntensionalInheritanceLink', a, b)],
            name = 'InheritanceEvaluation',
            formula = formulas.inheritanceFormula
        )
    )

    return rules

def planning_rules(atomspace):
    '''A bunch of dubious hacks for using PLN as a STRIPS-style planner. Which isn't a great idea anyway'''
    rules = []
    # Used by planning. An ExecutionLink indicates an action being performed, so we can
    # assume that the action will be performed as part of the plan (i.e. if any action
    # occurs in the plan, set the TV to full probability and confidence).
    #r = Rule(T('ExecutionLink', 1, 2),
    #                   [],
    #                   name = 'PerformAction',
    #                   tv = TruthValue(1.0,confidence_to_count(1.0)))
    #rules.append(r)
    # TOdo this should be actionSuccess not just actionDone
    r = Rule(actionDone_template(atomspace),
                [],
                name = 'PerformAction',
                tv = TruthValue(1.0,confidence_to_count(1.0)))
    rules.append(r)
    
    # If something is true at the current time, you can use it as the start of the plan.
    # First find the latest time (hack)
    current_time = 0
    for time in atomspace.get_atoms_by_type(t.TimeNode):
        timestamp = int(time.name)
        if timestamp > current_time:
            current_time = timestamp
    current_time_node = atomspace.add(t.TimeNode, str(current_time))
    # Then create the Rule
    # It's essential to store the template so it will have the same variables in both
    # the head and the goal
    template = evaluation_link_template()
    r = Rule(template,
                [T('AtTimeLink', current_time_node, template)],
                name = 'AtCurrentTime'
                )
    rules.append(r)
    
    # A hacky rule for determining SequentialAndLinks. It doesn't check that the things
    # happen after each other. It's just intended to find results from the PerformAction
    # or AtCurrentTime rules.
    type = 'SequentialAndLink'
    for size in xrange(10):
        args = [new_var() for i in xrange(size+1)]
        rules.append(Rule(T(type, args),
                           args,
                           type[:-4], 
                           formula = formulas.andSymmetricFormula))

    # A hacky rule for determining SimultaneousAndlinks.
    type = 'SimultaneousAndLink'
    for size in xrange(11):
        args = [new_var() for i in xrange(size+1)]
        rules.append(Rule(T(type, args),
            args,
            type[:-4],
            formula = formulas.andSymmetricFormula))


    return rules

def temporal_rules(atomspace):
    rules = []
    rules.append(Rule(T('BeforeLink', 1, 2),
                        [ ],
                        name='Before',
                        match = create_temporal_matching_function(temporalFormulas.beforeFormula)
                        )
                )
    rules.append(Rule(T('OverlapsLink', 1, 2),
                        [ ],
                        name='Overlaps',
                        match = create_temporal_matching_function(temporalFormulas.overlapsFormula)
                        )
                )
    rules.append(Rule(T('DuringLink', 1, 2),
                        [ ],
                        name='During',
                        match = create_temporal_matching_function(temporalFormulas.duringFormula)
                        )
                )
    rules.append(Rule(T('MeetsLink', 1, 2),
                        [ ],
                        name='Meets',
                        match = create_temporal_matching_function(temporalFormulas.meetsFormula)
                        )
                )
    rules.append(Rule(T('StartsLink', 1, 2),
                        [ ],
                        name='Starts',
                        match = create_temporal_matching_function(temporalFormulas.startsFormula)
                        )
                )
    rules.append(Rule(T('FinishesLink', 1, 2),
                        [ ],
                        name='Finishes',
                        match = create_temporal_matching_function(temporalFormulas.finishesFormula)
                        )
                )
    rules.append(Rule(T('EqualsLink', 1, 2),
                        [ ],
                        name='Equals',
                        match = create_temporal_matching_function(temporalFormulas.equalsFormula)
                        )
                )
    rules.append(Rule(T('AfterLink', 1, 2),
                        [ ],
                        name='After',
                        match = create_temporal_matching_function(temporalFormulas.afterFormula)
                        )
                )
    rules.append(Rule(T('Overlapped_byLink', 1, 2),
                        [ ],
                        name='Overlapped_by',
                        match = create_temporal_matching_function(temporalFormulas.overlapped_byFormula)
                        )
                )
    rules.append(Rule(T('ContainsLink', 1, 2),
                        [ ],
                        name='Contains',
                        match = create_temporal_matching_function(temporalFormulas.containsFormula)
                        )
                )
    rules.append(Rule(T('Met_byLink', 1, 2),
                        [ ],
                        name='Met_by',
                        match = create_temporal_matching_function(temporalFormulas.met_byFormula)
                        )
                )
    rules.append(Rule(T('Started_byLink', 1, 2),
                        [ ],
                        name='Started_by',
                        match = create_temporal_matching_function(temporalFormulas.started_byFormula)
                        )
                )
    rules.append(Rule(T('Finished_byLink', 1, 2),
                        [ ],
                        name='Finished_by',
                        match = create_temporal_matching_function(temporalFormulas.finished_byFormula)
                        )
                )

    # This Rule is important but the TV formula is wrong
    rules.append(Rule(T('BeforeLink', 1, 3),
                        [ T('BeforeLink', 1, 2), T('BeforeLink', 2, 3), Var(1), Var(2), Var(3) ],
                        name='BeforeTransitivityRule',
                        formula = formulas.deductionSimpleFormula
                        )
                )

    return rules

def lookup_times(tree, atomspace):
    '''For the specified Tree (which can contain a Node or Link), search for
    all the AtTimeLinks. Return a dictionary from timestamp (as an int) to
    a float, representing the fuzzy truth value. (Ignore confidence which is
    probably OK.)'''
    template = T('AtTimeLink',
                 Var(0),
                 tree)
    attimes = find(template, atomspace.get_atoms_by_type(t.AtTimeLink))
    
    dist = {}
    for link in attimes:
        assert isinstance(link, Atom)
        time = int(link.out[0].name)
        fuzzy_tv = link.tv.mean
        dist[time] = fuzzy_tv
    
    return dist

def create_temporal_matching_function(formula):
    def match_temporal_relationship(space,target):
        assert isinstance(target, Tree)
        # awkward technical limitation - can't look up (Before $x MyBirth) for example - all things that happened before i was born
        if target.args[0].is_variable() or target.args[1].is_variable():
            return []
        
        distribution_event1 = lookup_times(target.args[0], space)
        distribution_event2 = lookup_times(target.args[1], space)
        
        missing_data = (len(distribution_event1) == 0 or len(distribution_event2) == 0)
        error_message = "unable to find distribution for targets", target.args[0], distribution_event1, target.args[1], distribution_event2

        #assert not missing_data, error_message
        # Enable this instead of the assert after you finish debugging the rules.
        # For real-world use the assert is wrong - if you don't have the right data you should just not apply that Rule.
        if missing_data:
            print error_message
            # No info available for those events. So return no results
            return []

        strength = formula(distribution_event1, distribution_event2)
        
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

def match_wrapper(space, target, match):
    candidate_heads_tvs = match(space, target)

    heads_tvs = []
    for (h, tv) in candidate_heads_tvs:
        s = unify(h, target, {})
        if s != None:
            heads_tvs.append( (h,tv) )

    assert not (heads_tvs is None)
    return heads_tvs


def match_axiom_slow(space,target,candidates = None):
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
        smallest_set = [target.op]
    else:
        # The nodes in the target, as Atoms
        # TODO This may break if target is a Link with no Nodes underneath it!
        nodes = [tr.op for tr in target.flatten() if isinstance(tr.op, Atom) and tr.op.is_node()]

        links_of_type = space.get_atoms_by_type(target.get_type())
        
        smallest_set = links_of_type
        for n in nodes:
            candidates = find_links_upward(n)
            if len(candidates) < len(smallest_set):
                smallest_set = candidates
        
        #print target, len(smallest_set)

    # Then the chainer will try to unify against every candidate
    candidate_trees = (tree_from_atom(atom) for atom in smallest_set)
    candidate_tvs = (c.tv for c in smallest_set)
    
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

def match_subset(space,target):

    A, B = target.args
    #compatible = ((A.get_type() == t.ConceptNode and B.get_type() == t.ConceptNode) or
    #             (A.get_type() == t.PredicateNode and B.get_type() == t.PredicateNode))
    #if not compatible:
    #    return []
    if A.is_variable() or B.is_variable():
        return []

    print A, B

    def members(concept):
        '''For each member of concept, return the node and the strength of membership'''
        template = T('MemberLink', new_var(), concept)
        trees_tvs = match_wrapper(space, template, match_axiom)

        mems = [(tr.args[0], tv.mean) for (tr, tv) in trees_tvs]
        assert not (mems is None)
        return mems

    def all_member_links():
        '''Find all ObjectNodes (or other nodes) that are members of any concept.
        Returns a set of nodes (each node is wrapped in the Tree class)'''
        template = T('MemberLink', new_var(), new_var())
        trees_tvs = match_wrapper(space, template, match_axiom)

        mems = set(tr.args[0] for (tr, tv) in trees_tvs)
        return mems

    def non_members(concept):
        # Find the members of Not(A).
        # For example if A is 'the set of cats', then Not(A) is 'the set of things that aren't cats'.
        # So for every entity E in the world, (MemberLink E Not(cat)).tv.mean == 1 - (MemberLink E cat).tv.mean.
        # If the latter is not recorded in the AtomSpace, just assume it is 0.

        # Find all objects/etc that are members of anything
        # type: set(Tree)
        everything = all_member_links()

        # the type of members_of_concept is [(concept,frequency)]
        members_of_concept = members(concept)
        membershipStrengths = {member:strength for (member, strength) in members_of_concept}

        result = []
        for object in everything:
            membershipStrength = 0
            if object in membershipStrengths:
                membershipStrength = membershipStrengths[object]
            nonMembershipStrength = 1 - membershipStrength
            result.append( (object,nonMembershipStrength) )

        return result

    def evals(concept):
        template = T(
            'EvaluationLink',
                concept,
                T('ListLink', new_var())
        )
        trees_tvs = match_wrapper(space, template, match_axiom)

        mems = [(tr.args[1].args[0], tv.mean) for (tr, tv) in trees_tvs]
        return mems

    # Find the members of each concept
    # For single-argument predicates, this is the same as EvaluationLinks.
    # TODO: can't handle negated predicates in EvaluationLinks...
    # This means PredicateNodes can't be in IntensionalInheritanceLinks
    if A.get_type() == t.NotLink:
        # Members of Not(A)
        assert B.get_type() == t.ConceptNode
        memA = non_members(A)
        memB = members(B)
    else:
        # Members of A
        memA = members(A) + evals(A)
        memB = members(B) + evals(B)
#    memA = evals(A)
#    memB = evals(B)

    print memA
    print
    print memB

    # calculate P(x in B | x in A) = P(A^B) / P(A)
    # based on the fuzzy-weighted average
    #nodes_in_B = [m for (m,s) in memB]
    assert not (memB is None)
    nodesB = {member:strength for (member,strength) in memB}

    N_AB = 0
    for (mA, sA) in memA:
        if mA in nodesB:
            sB = nodesB[mA]

            # min is the definition of fuzzy-AND
            N_AB += min(sA,sB)
    
    #N_AB = sum(s for (m, s) in memA if m in nodes_in_B)
    N_A = sum(s for (m, s) in memA)
    if N_A > 0:
        P = N_AB*1.0 / N_A
        tv = TruthValue(P, confidence_to_count(1.0))
    else:
        # If there are no items in A then conditional probability is not defined, so give a zero confidence
        tv = TruthValue(0,0)

    return [(target, tv)]

def match_intensional_inheritance(space, target):
    A, B = target.args
    if A.get_type() != t.ConceptNode or B.get_type() != t.ConceptNode:
        return []

    def create_ASSOC(concept):
        # ASSOC(x, concept) = [Subset x concept - Subset(Not x, concept)]+

        assoc_name = 'ASSOC(%s)' % (concept.op.name,)
        assoc_node = space.add_node(t.ConceptNode, assoc_name)

        template = T('SubsetLink', new_var(), concept)
        trees_tvs = match_wrapper(space, template, match_axiom)

        # for each x that is a subset of concept
        has_any_members = False
        for (tr, tv) in trees_tvs:
            #print tr, tv
            x = tr.args[0]
            if x.get_type() != t.ConceptNode:
                continue
            # Now find Subset(Not x, concept)
            template_not = T('SubsetLink',
                T('NotLink', x),
                concept
            )
            not_candidates = match_wrapper(space, template_not, match_axiom)
            #print not_candidates
            assert len(not_candidates) < 2
            if len(not_candidates) == 0:
                print 'missing link',template_not
                continue

            (_, tv_not) = not_candidates[0]
            assoc_strength = tv.mean - tv_not.mean

            # TODO obviously we're fudging the confidence values here
            if assoc_strength > 0:
                mem_tr = T('MemberLink',
                    x,
                    assoc_node
                )
                mem_link = atom_from_tree(mem_tr, space)
                mem_link.tv = TruthValue(assoc_strength, confidence_to_count(1.0))
                print mem_link
                has_any_members = True

        #assert has_any_members

        return T(assoc_node)

    ASSOC_A = create_ASSOC(A)
    ASSOC_B = create_ASSOC(B)

    # IntInh A B = Subset ASSOC(A) ASSOC(B)
    subset_target = T('SubsetLink', ASSOC_A, ASSOC_B)
    # There should always be one result
    subset_result = match_subset(space, subset_target)
    [(_, tv)] = subset_result

    int_inh = T('IntensionalInheritanceLink', A, B)
    return [(int_inh, tv)]

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
        self.trace = Data_Trace()

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

