#import pyximport; pyximport.install()

#from IPython.Debugger import Tracer; debug_here = Tracer()

try:
    from opencog.atomspace import AtomSpace, types, Atom, TruthValue, get_type_name
except ImportError:
    from atomspace_remote import AtomSpace, types, Atom, TruthValue, get_type_name
from tree import *
from util import pp, OrderedSet, concat_lists, inplace_set_attributes
from pprint import pprint
#try:
#    from opencog.util import log
#except ImportError:
from util import log

from collections import defaultdict

import formulas

from sys import stdout
# You can use kcachegrind on cachegrind.out.profilestats
#from profilestats import profile

#import line_profiler
#profiler = line_profiler.LineProfiler()

from time import time
import exceptions

t = types

def format_log(*args):
    global _line    
    out = '['+str(_line) + '] ' + ' '.join(map(str, args))
#    if _line == 32:
#        import pdb; pdb.set_trace()
    _line+=1
    return out
_line = 1

class Chainer:

    # Convert Atoms into FakeAtoms for Pypy/Pickle/Multiprocessing compatibility
    _convert_atoms = False

    def __init__(self, space):
        self.deduction_types = ['SubsetLink', 'ImplicationLink', 'InheritanceLink']

        self.pd = dict()

        self.results = []

        self.space = space
        self.viz = PLNviz(space)
        self.viz.connect()
        self.setup_rules(space)

        self.bc_later = OrderedSet()
        self.bc_before = OrderedSet()

        self.fc_later = OrderedSet()
        self.fc_before = OrderedSet()
        
        global _line
        _line = 1
        
        #profiler.add_function(self.bc)

    #def do_planning(self):
    #    try:
    #        target_PredicateNodes = [x for x in self.space.get_atoms_by_type(t.PredicateNode) if "EnergyDemandGoal" in x.name]
    #
    #        for atom in target_PredicateNodes:
    #            # Here be DRAGONS!
    #            #target = Tree('EvaluationLink', atom, Tree('ListLink'))
    #            target = T('EvaluationLink', atom)
    #
    #        a = self.space
    #
    #        rl = T('ReferenceLink', a.add_node(t.ConceptNode, 'plan_selected_demand_goal'), target)
    #        atom_from_tree(rl, a)
    #        
    #        # hack
    #        print 'target'
    #        target_a = atom_from_tree(target, a)
    #        print target_a
    #        print target_a.tv
    #        target_a.tv = TruthValue(0,  0)
    #        print target_a
    #        # Reset the rules list so the EvaluationLink in it won't have a TV!
    #        self.setup_rules(a)
    #
    #        self.rules = [r for r in self.rules if r.name not in ['Deduction', 'Inversion']]
    #
    #        result_atoms = self.bc(target)
    #        
    #        print "planning result: ",  result_atoms
    #        
    #        if result_atoms:
    #            res_Handle = result_atoms[0]
    #            res = tree_from_atom(Atom(res_Handle, a))
    #            
    #            trail = self.trail(res)
    #            actions = self.extract_plan(trail)
    #            
    #            # set plan_success
    #            ps = T('EvaluationLink', a.add_node(t.PredicateNode, 'plan_success'), T('ListLink'))
    #            # set plan_action_list
    #            pal = T('ReferenceLink',
    #                        a.add_node(t.ConceptNode, 'plan_action_list'),
    #                        T('ListLink', actions))
    #            
    #            ps_a  = atom_from_tree(ps, a)
    #            pal_a = atom_from_tree(pal, a)
    #            
    #            print ps
    #            print pal
    #            ps_a.tv = TruthValue(1, 9001)
    #            
    #            print ps_a.tv
    #
    #    except Exception, e:
    #        print e


    #@profile
    def bc(self, target):
        #import prof3d; prof3d.profile_me()
        
        try:
            tvs = self.get_tvs(target)
            print "Existing target truth values:", map(str, tvs)
            
            log.info(format_log('bc', target))
            self.bc_later = OrderedSet([target])
            self.results = []

            self.target = target

            # viz - visualize the root
            self.viz.outputTarget(target, None, 0, 'TARGET')

            start = time()
            while self.bc_later and not self.results:
                children = self.bc_step()
                self.propogate_results_loop(children)

                msg = '%s goals expanded, %s remaining, %s Proof DAG Nodes' % (len(self.bc_before), len(self.bc_later), len(self.pd))
                log.info(format_log(msg))
                log.info(format_log('time taken', time() - start))
                #log.info(format_log('PD:'))
                #for pdn in self.pd:
                #    log.info(format_log(len(pdn.args),str(pdn)))
            
            # Always print it at the end, so you can easily check the (combinatorial) efficiency of all tests after a change
            print msg

            for res in self.results:
                print 'Inference trail:'
                trail = self.trail(res)
                self.print_tree(trail)
                print 'Action plan (if applicable):'
                print self.extract_plan(trail)
#            for res in self.results:
#                self.viz_proof_tree(self.trail(res))

            return self.results
            #return [atom_from_tree(result, self.space).h for result in self.results]
        except Exception, e:
            import traceback, pdb
            #pdb.set_trace()
            print traceback.format_exc(10)
            # Start the post-mortem debugger
            #pdb.pm()
            return []

    def fc(self):
        axioms = [r.head for r in self.rules if r.tv.count > 0]
        self.propogate_results_loop(axioms)
        
        while self.fc_later:
            next_premise = self.get_fittest() # Best-first search
            log.info(format_log('-FCQ', next_premise))

            apps = self.find_new_rule_applications_by_premise(next_premise)
            for app in apps:
                got_result = self.check_premises(app)
                if got_result:
                    self.compute_and_add_tv(app)
                    
                    if not self.contains_isomorphic_tree(app.head, self.fc_before) and not self.contains_isomorphic_tree(app.head, self.fc_later):
                        self.add_tree_to_index(app.head, self.fc_before)
                        self.add_tree_to_index(app.head, self.fc_later)
                        log.info(format_log('+FCQ', app.head, app.name))
                        stdout.flush()

    def propogate_results_loop(self, premises):
        assert not self.fc_later
        self.fc_later = OrderedSet(premises)
        # Any result which has been propogated before, may now be useful in new places.
        # So reset this list so they will be tried again.
        self.fc_before = OrderedSet()

        while self.fc_later and not self.results:
            self.propogate_results_step()

    def bc_step(self):
        assert self.bc_later
        #print 'bcq', map(str, self.bc_later)
        next_target = self.bc_later.pop_first() # Breadth-first search
        #next_target = self.bc_later.pop_last() # Depth-first search
        #next_target = self.get_fittest(self.bc_later) # Best-first search

        next_target = standardize_apart(next_target)

        log.info(format_log('-BCQ', next_target))
        self.add_tree_to_index(next_target, self.bc_before)

        ret = []
        apps = self.find_rule_applications(next_target)

        for a in apps:
            a = a.standardize_apart()
            #if not a.goals: print format_log('generator', repr(a))
            if not a.goals and a.tv:
                # Makes sure it goes in the list of apps, but just propogate results from the head (i.e. the actual Atom)
                # and not the goals (an empty list!)
                ret.append(a.head)                
                self.add_queries(a)
                #viz
                self.viz.declareResult(a.head)
            else:
                added_queries = self.add_queries(a)
                ret += added_queries

        return ret

    def propogate_results_step(self):
        next_premise = self.fc_later.pop_last() # Depth-first search
        #next_premise = self.get_fittest() # Best-first search

        next_premise = standardize_apart(next_premise)

        log.info(format_log('-FCQ', next_premise))

        # WARNING: the specialization process won't spec based on premises that only exist as axioms, or...

        # If the premises are already specific enough to be produced exactly by an existing chain of apps...
        # then you have a result! Apply the rule and see if it produces the target (or just an intermediary step).
        potential_results = self.find_existing_rule_applications_by_premise(next_premise)
        specialized = self.specialize_existing_rule_applications_by_premise(next_premise)
        # Make sure you don't create a specialization that already exists
        specialized = [r for r in specialized if 
                              not any(r2.isomorphic(r) for r2 in potential_results)]
        #print 'potential_results', potential_results


        # If B->C => C was checked by BC before, it will be in the bc_before set. But it now has a TV, so it should
        # be used again!

        # Ignore invalid rule applications (i.e. if add_queries returns nothing)
        specialized = [app for app in specialized if self.add_queries(app)]
        #print 'specialized', specialized

        real_results = []

        for app in potential_results:
            #print repr(a)

            got_result = self.check_premises(app)
            if got_result:
                #if app.head.op == 'TARGET':
                #    target = app.goals[0]
                #    log.info(format_log('Target produced!', target))
                #    self.results.append(target)
                #viz
                self.viz.declareResult(app.head)
                
                self.compute_and_add_tv(app)

                real_results.append(app)

                if app.head == self.target:
                    log.info(format_log('Target produced!', app.head, app.tv))
                    self.results.append(app.head)

        for app in specialized+real_results:
            # If there is a result, then you want to propogate it up. You should also propogate specializations,
            # so that their parents will be specialized as well.
            # The proof DAG does not explicitly avoid linking things up in a loop (as it has no explicit links)
            if not self.contains_isomorphic_tree(app.head, self.fc_before) and not self.contains_isomorphic_tree(app.head, self.fc_later):
            #if not self.contains_isomorphic_tree(app, self.fc_before) and not self.contains_isomorphic_tree(app.head, self.fc_later):
                self.add_tree_to_index(app.head, self.fc_before)
                self.add_tree_to_index(app.head, self.fc_later)
                log.info(format_log('+FCQ', app.head, app.name))
                stdout.flush()

    def contains_isomorphic_tree(self, tr, idx):        
        #return any(expr.isomorphic(tr) for expr in idx)
        canonical = tr.canonical()
        return canonical in idx

    def add_tree_to_index(self, tr, idx):
        canonical = tr.canonical()
        idx.append(canonical)


    def _app_is_stupid(self, goal):

        # You should probably skip the app entirely if it has any self-implying goals
        def self_implication(goal):
            return any(goal.get_type() == get_type(type_name) and len(goal.args) == 2 and goal.args[0].isomorphic(goal.args[1])
                        for type_name in self.deduction_types)

        # Nested ImplicationLinks
        # skip Implications between InheritanceLinks etc as well
        types = map(get_type, self.deduction_types)
        if (goal.get_type() in types and len(goal.args) == 2 and
                (goal.args[0].get_type() in types or
                 goal.args[1].get_type() in types) ):
            return True

        try:
            self._very_vague_link_templates
        except:
            self._very_vague_link_templates = [standardize_apart(T(type, 1, 2)) for type in self.deduction_types]

        # Should actually block this one if it occurs anywhere, not just at the root of the tree.
        #very_vague = any(goal.isomorphic(standardize_apart(T(type, 1, 2))) for type in self.deduction_types)
        very_vague = any(goal.isomorphic(template) for template in self._very_vague_link_templates)
        return (self_implication(goal) or
                     very_vague)

    def add_queries(self, app):
        def goal_is_stupid(goal):
            return goal.is_variable()

        if any(map(self._app_is_stupid, app.goals)) or self._app_is_stupid(app.head):
            return []

        # If the app is a cycle or already added, don't add it or any of its goals
        (status, app_pdn) = self.add_app_to_pd(app)
        if status == "CYCLE":
            #print "CYCLE", app.name, app.head, app.goals
            return []
        elif status == "EXISTING":
            # Already added this
            return []
        else:
            # Only visualize it if it is actually new
            # viz
            for (i, input) in enumerate(app.goals):
                self.viz.outputTarget(input.canonical(), app.head.canonical(), i, app.name)

        added_queries = []

        # It's useful to add the head if (and only if) it is actually more specific than anything currently in the BC tree.
        # This happens all the time when atoms are found.
        for goal in tuple(app.goals)+(app.head,):
            if     (not goal_is_stupid(goal) and
                    not self.contains_isomorphic_tree(goal, self.bc_before) and
                    not self.contains_isomorphic_tree(goal, self.bc_later) ):
                assert goal not in self.bc_before
                assert goal not in self.bc_later
                self.add_tree_to_index(goal, self.bc_later)
                added_queries.append(goal)
                log.info(format_log('+BCQ', goal, app.name))
                #stdout.flush()
        
        return added_queries

    def check_premises(self, app):
        '''Check whether the given app can produce a result. This will happen if all its premises are
        already proven. Or if it is one of the axioms given to PLN initially. It will only find premises
        that are exactly isomorphic to those in the app (i.e. no more specific or general). The chainer
        itself is responsible for finding specific enough apps.'''
        input_tvs = [self.get_tvs(input) for input in app.goals]
        return all(input_tvs)
    
    def compute_and_add_tv(self, app):
        # NOTE: assumes this is the real copy of the rule, not just a new one.
        #app.tv = True
        input_tvs = [self.get_tvs(g) for g in app.goals]
        if all(input_tvs):
            input_tvs = [tvs[0] for tvs in input_tvs]
            input_tvs = [(tv.mean, tv.count) for tv in input_tvs]
            tv_tuple = app.formula(input_tvs,  None)
            app.tv = TruthValue(tv_tuple[0], tv_tuple[1])
            #atom_from_tree(app.head, self.space).tv = app.tv            
            
            self.set_tv(app, app.tv)
            
            log.info (format_log(app.name, 'produced:', app.head, app.tv, 'using', zip(app.goals, input_tvs)))

    def find_rule_applications(self, target):
        '''The main 'meat' of the chainer. Finds all possible rule-applications matching your criteria.
        Chainers can be made by searching for certain apps and doing things with them.'''
        ret = []
        for r in self.rules:
            s = unify(r.head, target, {})
            if s != None:
                new_rule = r.subst(s)
                ret.append(new_rule)
        return ret

    def find_existing_rule_applications_by_premise(self, premise):
        premise_pdn = self.expr2pdn(premise.canonical())        
        return [app_pdn.op for app_pdn in premise_pdn.parents]

    def specialize_existing_rule_applications_by_premise(self, premise):
        ret = []
        for expr_pdn in self.pd.values():
            for app_pdn in expr_pdn.parents:
                #app = Rule(head=app_pdn.parents[0].op,goals=app_pdn.args,name=app_pdn)                    
                #Rule(head,goals,name,tv,formula)
                app = app_pdn.op

                # It's necessary to store the app separately, because the
                # variables in its arguments may (and may not) be shared
                # between the arguments, but each expression-node in the
                # proof DAG actually has standard variables from 0.
                for arg in app.goals:
                    s = unify(arg, premise, {})
                    if s != None and not arg.isomorphic(premise):
                        # For every app with this as a premise
                            new_a = app.subst(s)
                            ret.append(new_a)
        return ret

    def find_new_rule_applications_by_premise(self,premise):
        ret = []
        for r in self.rules:
            for g in r.goals:
                s = unify(g, premise, {})
                if s != None:
                    new_rule = r.subst(s)
                    ret.append(new_rule)
        return ret


    def get_tvs(self, expr):
        # Only want to find results for this exact target, not every possible specialized version of it.
        # The chaining mechanism itself will create different apps for different specialized versions.
        # If there are any variables in the target, and a TV is found, that means it has been proven
        # for all values of that variable.
        
        canonical = expr.canonical()
        expr_pdn = self.expr2pdn(canonical)
        app_pdns = expr_pdn.args
        #print 'get_tvs:', [repr(app_pdn.op) for app_pdn in app_pdns if app_pdn.tv.count > 0]
        return [app_pdn.tv for app_pdn in app_pdns if app_pdn.tv.count > 0]
    
    def expr2pdn(self, expr):
        pdn = DAG(expr,[])
        try:
            return self.pd[pdn]
        except KeyError:
            #print 'expr2pdn adding %s for the first time' % (pdn,)
            self.pd[pdn] = pdn
            return pdn

    def set_tv(self,app,tv):
        # Find/add the app
        a = self.app2pdn(app)
        a.tv = tv

    def add_app_to_pd(self,app):
        head_pdn = self.expr2pdn(app.head.canonical())

        def canonical_app_goals(goals):
            return map(Tree.canonical, goals)

        goals_canonical = canonical_app_goals(app.goals)
        # Don't allow loops. Does this need to be a separate test? It should probably
        # check whether the new target is more specific, not just equal?
        goal_pdns = [self.expr2pdn(g) for g in goals_canonical]
        if head_pdn.any_path_up_contains(goal_pdns):
            return ('CYCLE',None)
        
        # Check if this application is in the Proof DAG already.
        # NOTE: You must use the app's goals rather than the the app PDN's arguments,
        # because the app's goals may share variables.
        existing = [apn for apn in head_pdn.args if canonical_app_goals(apn.op.goals) == goals_canonical]
        assert len(existing) < 2
        if len(existing) == 1:
            return ('EXISTING',existing[0])
        else:
            # Otherwise add it to the Proof DAG
            app_pdn = DAG(app,[])
            app_pdn.tv = app.tv
            
            #print 'add_app_to_pd:',repr(app)
            
            for goal_pdn in goal_pdns:
                app_pdn.append(goal_pdn)
            head_pdn.append(app_pdn)
        
            #print 'add_app_to_pd adding %s for the first time' % (app_pdn,)
        
            return ('NEW',app_pdn)

    def app2pdn(self,app):
        (status,app_pdn) = self.add_app_to_pd(app)
        return app_pdn
    
    def add_rule(self, rule):
        self.rules.append(rule)
        
        # This is necessary so get_tvs will work
        # Only relevant to generators or axioms
        if rule.tv.confidence > 0:
            self.app2pdn(rule)

    def extract_plan(self, trail):
#        def is_action(proofnode):
#            target = proofnode.op
#            return target.op == 'ExecutionLink'
#        # Extract all the targets in best-first order
#        proofnodes = trail.flatten()
#        proofnodes.reverse()
#        actions = [pn.op for pn in proofnodes if is_action(pn)]
#        return actions
        # The list of actions in an ImplicationLink. Sometimes there are none,
        # sometimes one; if there is a SequentialAndLink then it can be more than one.
        def actions(proofnode):            
            target = proofnode.op
            if isinstance(target, Rule):
                return []
            if target.op in ['ExecutionLink',  'SequentialAndLink']:
                return [pn.op]
            else:
                return []
        # Extract all the targets in best-first order
        proofnodes = trail.flatten()
        proofnodes.reverse()
        actions = concat_lists([actions(pn) for pn in proofnodes])
        return actions

    def trail(self, target):
        #def filter_with_tv(tr):
        #    args = []
        #    for child in tr.args:
        #        if len(self.get_tvs(child.op)) > 0:
        #            args.append(child)
        #    return Tree(tr.op, args)
        #
        #bit = self.traverse_tree(target, set())
        #
        #return filter_with_tv(bit)

        def rule_found_result(rule_pdn):
            return rule_pdn.tv.count > 0
            #try:
            #    rule_pdn.tv
            #    return True
            #except AttributeError:
            #    return False

            #return hasattr(rule_pdn, 'tv')

        def recurse(rule_pdn):
            print repr(rule_pdn.op),' PDN args', rule_pdn.args
            exprs = map(filter_expr,rule_pdn.args)
            return DAG(rule_pdn.op, exprs)

        def filter_expr(expr_pdn):
            print expr_pdn.op, expr_pdn.args
            #successful_rules = [recurse(rpdn) for rpdn in expr_pdn.args if rule_found_result(rpdn)]
            successful_rules = [recurse(rpdn) for rpdn in expr_pdn.args if rule_found_result(rpdn)]
            return DAG(expr_pdn.op, successful_rules)
        
        root = self.expr2pdn(target)

        return filter_expr(root)
    
    def print_tree(self, tr, level = 1):
        try:
            (tr.depth, tr.best_conf_above)
            print ' '*(level-1)*3, tr.op, tr.depth, tr.best_conf_above
        except AttributeError:
            print ' '*(level-1)*3, tr.op
        
        for child in tr.args:
            self.print_tree(child, level+1)

    #def viz_proof_tree(self, pt):
    #    self.viz.connect()
    #
    #    target = pt.op
    #    self.viz.outputTarget(target, None, 0, repr(target))
    #    
    #    for arg in pt.args:
    #        self.viz_proof_tree_(arg)
    #    
    #def viz_proof_tree_(self, pt):
    #    target = pt.op
    #    
    #    self.viz.declareResult(target)
    #    
    #    for (i, input) in enumerate(pt.args):
    #        self.viz.outputTarget(input, target, i, repr(target))
    #
    #    for arg in pt.args:
    #        self.viz_proof_tree_(arg)
    #
    #def add_depths(self, bitnode, level = 1):
    #    #args = [self.add_depths(child, level+1) for child in tr.args]
    #    #return Tree((level, tr.op), args)
    #    
    #    args = [self.add_depths(child, level+1) for child in bitnode.args]
    #    return inplace_set_attributes(bitnode, depth=level)
    #
    #def add_best_conf_above(self, bitnode, best_above=0.0):
    #    bitnode.best_conf_above = best_above
    #
    #    if best_above > 0:
    #        print '-------', str(bitnode), best_above
    #
    #    confs_this_target = [tv.confidence for tv in self.get_tvs(bitnode.op)]
    #    best_above = max([best_above] + confs_this_target)
    #    
    #    for child in bitnode.args:
    #        self.add_best_conf_above(child, best_above) 
    #    return bitnode
    #
    #def get_fittest(self, queue):
    #    def num_vars(target):
    #        return len([vertex for vertex in target.flatten() if vertex.is_variable()])
    #
    #    competition_weight = - 10000
    #    depth_weight = -100
    #    solution_space_weight = -0.01
    #
    #    bit = self.traverse_tree(self.target, set())
    #    self.add_depths(bit)
    #    self.add_best_conf_above(bit)
    #    
    #    #self.print_tree(bit)
    #    
    #    flat = bit.flatten()
    #    in_queue = [bitnode for bitnode in flat if self.contains_isomorphic_tree(bitnode.op, queue)]
    #    if not in_queue:
    #        import pdb; pdb.set_trace()
    #    assert in_queue
    #    scores = [ bitnode.best_conf_above * competition_weight
    #                    +bitnode.depth * depth_weight
    #                    +num_vars(bitnode.op) * solution_space_weight
    #                    for bitnode in in_queue]
    #    ranked_bitnodes = zip(scores, in_queue)
    #    #ranked.sort(key=lambda (score, tr): -score)
    #    #print format_log(ranked)
    #    best = max(ranked_bitnodes, key=lambda (score, tr): score) [1] . op
    #    length = len(queue)
    #    queue.remove(next(existing for existing in queue if existing.isomorphic(best)))
    #    assert len(queue) == length - 1
    #    #print best
    #    return best

    def setup_rules(self, a):
        self.rules = []

        # All existing Atoms
        for obj in a.get_atoms_by_type(t.Atom):
            # POLICY: Ignore all false things. This means you can never disprove something! But much more useful for planning!
            if obj.tv.count > 0 and obj.tv.mean > 0:
                tr = tree_from_atom(obj)
                # A variable with a TV could just prove anything; that's evil!
                if not tr.is_variable():
                    
                    if 'CHUNK' in str(tr):
                        continue
                    
                    r = Rule(tr, [], '[axiom]')
                    r.tv = obj.tv
                    self.add_rule(r)

        ## Deduction
        #for type in self.deduction_types:
        #    self.add_rule(Rule(T(type, 1,3), 
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
        #    self.add_rule(Rule( T(type, 2, 1), 
        #                                 [T(type, 1, 2),
        #                                  Var(1),
        #                                  Var(2)], 
        #                                 name='Inversion', 
        #                                 formula = formulas.inversionFormula))

        # ModusPonens
        for type in ['ImplicationLink']:
            self.add_rule(Rule(Var(2), 
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
#                self.add_rule(Rule(Var(2), 
#                                             [T(type, andlink, 2),
#                                              andlink ], 
#                                              name='TheoremRule'))
        
       # ModusPonens for EvaluationLinks only
#        for type in ['ImplicationLink']:
#            conc = T('EvaluationLink', new_var(), new_var())
#            prem = T('EvaluationLink', new_var(), new_var())
#            imp = T('ImplicationLink', prem, conc)
#            
#            self.add_rule(Rule(conc, 
#                                         [imp, prem], 
#                                          name='ModusPonens_Eval'))

#        for type in ['ImplicationLink']:
#            conc = T('EvaluationLink', a.add_node(t.PredicateNode, 'B'))
#            prem = T('EvaluationLink', a.add_node(t.PredicateNode, 'A'))
#            imp = T('ImplicationLink', prem, conc)
#            
#            self.add_rule(Rule(conc, 
#                                         [imp, prem], 
#                                          name='ModusPonens_AB'))

        # AND/OR
        type = 'AndLink'
        for size in xrange(5):                
            args = [new_var() for i in xrange(size+1)]
            self.add_rule(Rule(T(type, args),
                               args,
                               type[:-4], 
                               formula = formulas.andSymmetricFormula))

        type = 'OrLink'
        for size in xrange(2):
            args = [new_var() for i in xrange(size+1)]
            self.add_rule(Rule(T(type, args),
                               args,
                               type[:-4], 
                               formula = formulas.orFormula))

        # Adding a NOT
        self.add_rule(Rule(T('NotLink', 1),
                           [ Var(1) ],
                           name = 'Not', 
                           formula = formulas.notFormula))

        # Link conversion
        self.add_rule(Rule(T('InheritanceLink', 1, 2),
                           [ T('SubsetLink', 1, 2) ],
                           name = 'SubsetLink=>InheritanceLink', 
                           formula = formulas.ext2InhFormula))

        # In planning, assume that an ExecutionLink (action) will be performed
        self.add_rule(Rule(T('ExecutionLink', 1, 2),
                           [],
                           name = 'PerformAction',
                           formula = formulas.ext2InhFormula))

#        # Producing ForAll/Bind/AverageLinks?
#        for type in ['ForAllLink', 'BindLink', 'AverageLink']:
#            self.add_rule(Rule(T(type, 1, 2),
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
#        self.add_rule(r)

        for atom in a.get_atoms_by_type(t.AverageLink):
            # out[0] is the ListLink of VariableNodes, out[1] is the expression
            tr = tree_from_atom(atom.out[1])
            r = Rule(tr, [], name='Average')
            r.tv = atom.tv
            self.add_rule(r)

        Chainer._convert_atoms = False

class Rule :
    def __init__ (self, head, goals, name, tv = TruthValue(0, 0), formula = None):
        if Chainer._convert_atoms:
            self.head = tree_with_fake_atoms(head)
            self.goals = map(tree_with_fake_atoms, goals)
        else:
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

from urllib2 import URLError
def check_connected(method):
    '''A nice decorator for use in visualization classes that stream graphs to Gephi. It catches exceptions raised
    when you aren't running Gephi.'''
    def wrapper(self, *args, **kwargs):
        if not self.connected:
            return

        try:
            method(self, *args, **kwargs)
        except URLError:
            self.connected = False

    return wrapper

from collections import defaultdict
import pygephi
class PLNviz:

    def __init__(self, space):
        self._as = space
        self.node_attributes = {'size':10, 'r':0.0, 'g':0.0, 'b':1.0}
        self.rule_attributes = {'size':10, 'r':0.0, 'g':1.0, 'b':1.0}
        self.root_attributes = {'size':20, 'r':1.0, 'g':1.0, 'b':1.0}
        self.result_attributes = {'r':1.0, 'b':0.0, 'g':0.0}

        self.connected = False
        
        self.parents = defaultdict(set)

    def connect(self):
        try:
            self.g = pygephi.JSONClient('http://localhost:8080/workspace0', autoflush=True)
            self.g.clean()
            self.connected = True
        except URLError:
            self.connected = False

    @check_connected
    def outputTarget(self, target, parent, index, rule=None):
        self.parents[target].add(parent)

        #target_id = str(hash(target))
        target_id = str(target)

        if parent == None:
            self.g.add_node(target_id, label=str(target), **self.root_attributes)

        if parent != None:
            self.g.add_node(target_id, label=str(target), **self.node_attributes)

            #parent_id = str(hash(parent))
            #link_id = str(hash(target_id+parent_id))
            parent_id = str(parent)
            #rule_app_id = 'rule '+repr(rule)+parent_id
            rule_app_id = 'rule '+str(rule)+parent_id
            target_to_rule_id = rule_app_id+target_id
            parent_to_rule_id = rule_app_id+' parent'

            self.g.add_node(rule_app_id, label=str(rule), **self.rule_attributes)

            self.g.add_node(parent_id, label=str(parent), **self.node_attributes)

            # Link parent to rule app
            self.g.add_edge(parent_to_rule_id, rule_app_id, parent_id, directed=True, label='')
            # Link rule app to target
            self.g.add_edge(target_to_rule_id, target_id, rule_app_id, directed=True, label=str(index+1))

    @check_connected
    def declareResult(self, target):
        target_id = str(target)
        self.g.change_node(target_id, **self.result_attributes)

        #self.g.add_node(target_id, label=str(target), **self.result_attributes)

    @check_connected
    # More suited for Fishgram
    def outputTreeNode(self, target, parent, index):
        #target_id = str(hash(target))
        target_id = str(target)

        if parent == None:
            self.g.add_node(target_id, label=str(target), **self.root_attributes)

        if parent != None:
            self.g.add_node(target_id, label=str(target), **self.node_attributes)

            #parent_id = str(hash(parent))
            #link_id = str(hash(target_id+parent_id))
            parent_id = str(parent)
            link_id = str(parent)+str(target)

            self.g.add_node(parent_id, label=str(parent), **self.node_attributes)
            self.g.add_edge(link_id, parent_id, target_id, directed=True, label=str(index))

if __name__ == '__main__':
    a = AtomSpace()
    log.use_stdout(True)

    atom_from_tree(T('EvaluationLink',a.add_node(t.PredicateNode,'A')), a).tv = TruthValue(1, 1)
    #atom_from_tree(T('EvaluationLink',1), a).tv = TruthValue(1, 1)

    c = Chainer(a)

    #search(T('EvaluationLink',a.add_node(t.PredicateNode,'B')))
    #fc(a)

    #c.bc(T('EvaluationLink',a.add_node(t.PredicateNode,'A')))

#    global rules
#    A = T('EvaluationLink',a.add_node(t.PredicateNode,'A'))
#    B = T('EvaluationLink',a.add_node(t.PredicateNode,'B'))
#    rules.append(Rule(B, 
#                                  [ A ]))

    #c.bc(T('EvaluationLink',a.add_node(t.PredicateNode,'A')))
    c.bc(T('EvaluationLink',-1))
