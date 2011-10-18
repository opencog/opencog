#import pyximport; pyximport.install()

from opencog.atomspace import AtomSpace, types, Atom, Handle, TruthValue
import opencog.cogserver
from tree import *
from adaptors import find_matching_conjunctions
from util import pp, OrderedSet, concat_lists
from opencog.util import log

from sys import stdout
from profilestats import profile
from time import time

t = types

def format_log(*args):
    global _line    
    out = str(_line) + ' ' + ' '.join(map(str, args))
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

        self.space = space
        self.viz = PLNviz(space)
        self.viz.connect()
        self.setup_rules(space)
        self.apps = []
        # Disturbingly, a separate set is still necessary for rule applications as they contain lists so tuples are stored instead.
        # Alternatively, a different hash and equality implementation could be used, or something
        self.apps_set = OrderedSet()
        self.bc_later = OrderedSet()
        self.bc_before = OrderedSet()

        self.fc_later = OrderedSet()
        self.fc_before = OrderedSet()

        global _line
        _line = 1

    @profile
    def bc(self, target):
        try:
            #save_trees([target], 'target')
            #save_trees([tree_from_atom(a) for a in self.space.get_atoms_by_type(t.Atom) if a.tv.count > 0], 'as')
            if Chainer._convert_atoms:
                target = tree_with_fake_atoms(target)

            log.info(format_log('bc', target))
            self.bc_later = OrderedSet([target])
            self.target = target
            self.results = []

            # viz - visualize the root
            self.viz.outputTarget(target, None, 0, 'GOAL')

            start = time()
            while self.bc_later and not self.results:
                log.info(format_log(time() - start))
#                if time() - start > 30:
#                    print 'TIMEOUT'
#                    break
                children = self.bc_step()
                self.propogate_results_loop(children)

                log.info('%s goals expanded, %s remaining, %s apps' % (len(self.bc_before), len(self.bc_later), len(self.apps)))
                #print ('%s goals expanded, %s remaining, %s apps' % (len(self.bc_before), len(self.bc_later), len(self.apps)))

            return [atom_from_tree(result, self.space).h for result in self.results]
        except Exception, e:
            import traceback, pdb

            print traceback.format_exc(10)
            # Start the post-mortem debugger
            #pdb.pm()
            return []

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
        #next_target = self.bc_later.pop_first() # Breadth-first search
        #next_target = self.bc_later.pop_last() # Depth-first search
        next_target = self.get_fittest(self.bc_later) # Best-first search
        log.info(format_log('-BCQ', next_target))
        self.bc_before.append(next_target)

        ret = []
        apps = self.find_rule_applications(next_target)

        for a in apps:
            a = a.standardize_apart()
            if not a.goals: print 'generator', repr(a)
            if not a.goals and a.tv:
                ret.append(a.head)
                #ret += self.add_queries(a)
                #viz
                self.viz.declareResult(a.head)
            else:
                added_queries = self.add_queries(a)
                ret += added_queries
#            added_queries = self.add_queries(a)
#            ret += added_queries

        return ret

    def propogate_results_step(self):
        #print 'fcq', map(str, self.fc_later)
        next_premise = self.fc_later.pop_last() # Depth-first search
        #self.fc_before.append(next_premise)
        #next_premise = self.get_fittest() # Best-first search

        log.info(format_log('-FCQ', next_premise))
#        import pdb; pdb.set_trace()

#        # Handle the case of actual Atoms, albeit in a somewhat messy way.
#        # NOTE: The backward chainer won't find rules with actual Atoms, because
#        # it's only interested in adding the GOALS of a rule into the queue.
#        # In addition, it will repeat results that were found via actual rules.
#        # It should be possible to do this via 1-step recursion, or something.
#        # This would also require the exact target!
#        if self.get_tvs(next_premise):
#            if next_premise.unifies(self.target):
#                self.results.append(next_premise)
#                #viz
#                self.viz.declareResult(next_premise)
#
#            if not self.contains_isomorphic_tree(next_premise, self.fc_before_idx) and not self.contains_isomorphic_tree(next_premise, self.fc_later_idx):
#                print (format_log('lookup:', next_premise))
#                stdout.flush()
#                self.fc_later.append(app.head)
#            
#            self.add_queries(app)

        # WARNING: the specialization process won't spec based on premises that only exist as axioms, or...

        # If the premises are already specific enough to be produced exactly by an existing chain of apps...
        # then you have a result! Apply the rule and see if it produces the target (or just an intermediary step).
        potential_results = self.find_existing_rule_applications_by_premise(next_premise)
        specialized = self.specialize_existing_rule_applications_by_premise(next_premise)
        # Make sure you don't create a specialization that already exists
        specialized = [r for r in specialized if 
                              not any(r2.isomorphic(r) for r2 in potential_results)]
        #print 'potential_results', potential_results

#        # Notice if this is actually one of the original axioms. This needs to be done separately. Because
#        # get_tv requires the app to already have found the exact atom, with no variables where there shouldn't be.
#        # So this system allows discovering more specific atoms, which then leads to specializing one or more apps.
#        direct_rules = [r for r in self.find_rule_applications(next_premise) if r.tv and not r.goals]
#        for atom_rule in direct_rules:
#            if atom_rule.head.isomorphic(next_premise):
#                print (format_log('lookup:', atom_rule.head))
#                potential_results.append(atom_rule)
#            else:
#                print (format_log('spec:', atom_rule.head))
#                specialized.append(atom_rule)

        # If B->C => C was checked by BC before, it will be in the bc_before set. But it now has a TV, so it should
        # be used again!

        #print [r for r in potential_results+specialized if str(r.head) == '(SubsetLink AlQaeda:ConceptNode Abu:ConceptNode)']

        # If more specific values for the (variables in the) goals have been found, then make a new
        # app, which is more specific. In particular, those variables will have been filled in within any other
        # goals too.
        #for app in specialized:
            # If new values for the variables have just been found, we want to use the query mechanism,
            # to find any other goals. They may also be more specific now (if they used the same variables),
            # so we want to find them again before propogating results any further.
            #self.add_queries(app)
        # Ignore invalid rule applications (i.e. if add_queries returns nothing)
        specialized = [app for app in specialized if self.add_queries(app)]
        #print 'specialized', specialized

        real_results = []

        for app in potential_results:
            #print repr(a)

            got_result = self.check_premises_and_add_result(app)
            if got_result:
                #viz
                self.viz.declareResult(app.head)

                if app.head.unifies(self.target):
                    self.results.append(app.head)

                print (format_log('produced:', app.head))

                real_results.append(app)

        for app in specialized+real_results:
            # If there is a result, then you want to propogate it up. You should also propogate specializations,
            # so that their parents will be specialized as well.
            # The proof DAG does not explicitly avoid linking things up in a loop (as it has no explicit links)
            if not self.contains_isomorphic_tree(app.head, self.fc_before) and not self.contains_isomorphic_tree(app.head, self.fc_later):
            #if not self.contains_isomorphic_tree(app, self.fc_before) and not self.contains_isomorphic_tree(app.head, self.fc_later):
                self.fc_before.append(app.head)
                self.fc_later.append(app.head)
                log.info(format_log('+FCQ', app.head, app.name))
                stdout.flush()

# Some hints about how to make an index that stores the canonical trees.
#    def index_fill(self, idx, goals):
#        for g in goals:
#            canon = tuple(canonical_trees((g, )))
#            idx.add(canon)
#        queue = goals

    def contains_isomorphic_tree(self, tr, idx):        
        return any(expr.isomorphic(tr) for expr in idx)

#    # NOTE: assumes that you want to add the item into the corresponding index
#    def contains_isomorphic_tree(self, tr, idx):
#        start = time()
#        #containing = any(tr.isomorphic(existing) for existing in collection)
#        canon = tuple(canonical_trees((tr, )))
#        #idx = self.indexes[collection]
#        if canon in idx:
#            #print 'contains_isomorphic_tree', time() - start
#            return True
#        else:
#            idx.add(canon)
#            #print 'contains_isomorphic_tree', time() - start
#            return False
#        #print 'contains_isomorphic_tree', time() - start
#        #return containing

    def add_queries(self, app):
        def goal_is_stupid(goal):
            return goal.is_variable()

        def app_is_stupid(goal):
            #nested_implication = standardize_apart(Tree('ImplicationLink', 1, Tree('ImplicationLink', 2, 3)))
            # Accidentally unifies with (ImplicationLink $blah some_target) !
            #nested_implication2 = Tree('ImplicationLink', Tree('ImplicationLink', 1, 2), 3)

            # Nested ImplicationLinks
            # skip Implications between InheritanceLinks etc as well
            for nested_type_name in self.deduction_types:
                nested_type = get_type(nested_type_name)
                if goal.get_type() == t.ImplicationLink and len(goal.args) == 2 and goal.args[1].get_type() == nested_type:
                    return True

                if goal.get_type() == t.ImplicationLink and len(goal.args) == 2 and goal.args[0].get_type() == nested_type:
                    return True

            # Should actually block this one if it occurs anywhere, not just at the root of the tree.
            very_vague = any(goal.isomorphic(standardize_apart(Tree(type, 1, 2))) for type in self.deduction_types)
            return (self_implication(goal) or
                         very_vague)

        # You should probably skip the app entirely if it has any self-implying goals
        def self_implication(goal):
            return any(goal.get_type() == get_type(type_name) and len(goal.args) == 2 and goal.args[0].isomorphic(goal.args[1])
                        for type_name in self.deduction_types)

        if any(map(app_is_stupid, app.goals)) or app_is_stupid(app.head):
            return []

        added_queries = []

        # It's useful to add the head if (and only if) it is actually more specific than anything currently in the BC tree.
        # This happens all the time when atoms are found.
        for goal in tuple(app.goals)+(app.head,):
            if     (not goal_is_stupid(goal) and
                    not self.contains_isomorphic_tree(goal, self.bc_before) and
                    not self.contains_isomorphic_tree(goal, self.bc_later) ):
                assert goal not in self.bc_before
                assert goal not in self.bc_later
                self.bc_later.append(goal)
                added_queries.append(goal)
                log.info(format_log('+BCQ', goal, app.name))
                stdout.flush()

        # This records the path of potential rule-apps found on the way down the search tree,
        # so that results can be propogated back up that path. If you just did normal forward
        # chaining on the results, it would take lots of other paths as well.

        canon = app.canonical_tuple()
        if canon not in self.apps_set:
            self.apps_set.add(canon)
            self.apps.append(app)
        #if not any(app.isomorphic(existing) for existing in self.apps):
        #    self.apps.append(app)

            # Only visualize it if it is actually new
            # viz
            for (i, input) in enumerate(app.goals):
                self.viz.outputTarget(input.canonical(), app.head.canonical(), i, repr(app))

        return added_queries

    def check_premises_and_add_result(self, app):
        '''Check whether the given app can produce a result. This will happen if all its premises are
        already proven. Or if it is one of the axioms given to PLN initially. It will only find premises
        that are exactly isomorphic to those in the app (i.e. no more specific or general). The chainer
        itself is responsible for finding specific enough apps.'''
        input_tvs = [self.get_tvs(input) for input in app.goals]
        if all(input_tvs):
            self.compute_and_add_tv(app)
            return True
        return False

    def compute_and_add_tv(self, app):
        # NOTE: assumes this is the real copy of the rule, not just a new one.
        app.tv = True

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
        ret = []
        for a in self.apps:
            if any(arg.isomorphic(premise) for arg in a.goals):
                ret.append(a)
        return ret

    def specialize_existing_rule_applications_by_premise(self, premise):
        ret = []
        for a in self.apps:
            for arg in a.goals:
                s = unify(arg, premise, {})
                # TODO This is redundant as it's also calculated in the other function. But whatever.
                if s != None and not arg.isomorphic(premise):
                    new_a = a.subst(s)
                    ret.append(new_a)
        return ret

#    def find_existing_rule(self, rule):
#        matches = [r for r in self.rules if r.isomorphic(rule)]
#        assert len(matches) < 2
#        return matches

    def get_tvs(self, expr):
        # NOTE: It may be easier to just do this by just storing the TVs for each target.
        #rs = self.find_rule_applications(expr)

        # Only want to find results for this exact target, not every possible specialized version of it.
        # The chaining mechanism itself will create different apps for different specialized versions.
        # If there are any variables in the target, and a TV is found, that means it has been proven
        # for all values of that variable.
        #rs = [r for r in rs if unify(expr, r.head, {}) != None]
        rs = [r for r in self.rules+self.apps if expr.isomorphic(r.head)]

        return [r.tv for r in rs if r.tv]

#    def is_true_weird(self, expr):
#        #import pdb; pdb.set_trace()
#        rs = self.find_rule_applications(expr)
#        if len([r.tv for r in rs if r.tv and not r.goals]) > 0:
#            print expr, repr(r)
#            import pdb; pdb.set_trace()
#        return len([r.tv for r in rs if r.tv and not r.goals]) > 0

    def add_rule(self, rule):        
        self.rules.append(rule)

    def traverse_tree(self, target, already):
        producers = [app for app in self.apps if app.head.unifies(target)]
        
        # Deliberately allows repetition of subgoals
        subgoals = concat_lists([list(app.goals) for app in producers])
        subgoals_ = []
        for goal in subgoals:
            canon = goal.canonical()
            if canon not in already:
                subgoals_.append(canon)
            already.add(canon)
        
        #return [target]+concat_lists([self.traverse_tree(g, already) for g in subgoals_])
        return Tree(target, [self.traverse_tree(g, already) for g in subgoals_])

    def follow_tree(self, tr, level = 1):
        print ' '*level, Tree(tr.op)
        
        for child in tr.args:
            self.follow_tree(child, level+1)

    def add_depths(self, tr, level = 1):
        args = [self.add_depths(child, level+1) for child in tr.args]
        return Tree((level, tr.op), args)

    def get_fittest(self, queue):
        def num_vars(target):
            return len([vertex for vertex in target.flatten() if vertex.is_variable()])

        depth_weight = -100
        solution_space_weight = -0.01

        bit = self.traverse_tree(self.target, set())
        deep = self.add_depths(bit)
        #print format_log(self.follow_tree(deep))
        flat = deep.flatten()
        in_queue = [depthed_tr for depthed_tr in flat if self.contains_isomorphic_tree(depthed_tr.op[1], queue)]
        if not in_queue:
            import pdb; pdb.set_trace()
        assert in_queue
        #print in_queue
        ranked = [(tr.op[0]*depth_weight
                        +num_vars(tr.op[1])*solution_space_weight
                        , tr.op[1])
                       for tr in in_queue]
        #ranked.sort(key=lambda (score, tr): -score)
        #print format_log(ranked)
        best = max(ranked, key=lambda (score, tr): score) [1]
        length = len(queue)
        queue.remove(next(existing for existing in queue if existing.isomorphic(best)))
        assert len(queue) == length - 1
        #print best
        return best

    def setup_rules(self, a):
        self.rules = []

        # All existing Atoms
        for obj in a.get_atoms_by_type(t.Atom):
            if obj.tv.count > 0:
                tr = tree_from_atom(obj)
                # A variable with a TV could just prove anything; that's evil!
                if not tr.is_variable():
                    r = Rule(tr, [], '[axiom]')
                    r.tv = True
                    self.add_rule(r)

        # Deduction
        for type in self.deduction_types:
            self.add_rule(Rule(Tree(type, 1,3), 
                                         [Tree(type, 1, 2),
                                          Tree(type, 2, 3) ], 
                                          name='Deduction'))

        # Inversion
        for type in self.deduction_types:
            self.add_rule(Rule( Tree(type, 1, 2), 
                                         [Tree(type, 2, 1)], 
                                         name='Inversion'))

        # ModusPonens
        for type in ['ImplicationLink']:
            self.add_rule(Rule(Tree(2), 
                                         [Tree(type, 1, 2),
                                          Tree(1) ], 
                                          name='ModusPonens'))

#       # MP for AndLink as a premise
#        for type in ['ImplicationLink']:
#            for size in xrange(5):
#                args = [new_var() for i in xrange(size+1)]
#                andlink = Tree('AndLink', args)
#
#                self.add_rule(Rule(Tree(2), 
#                                             [Tree(type, andlink, 2),
#                                              andlink ], 
#                                              name='TheoremRule'))
        
       # ModusPonens for EvaluationLinks only
#        for type in ['ImplicationLink']:
#            conc = Tree('EvaluationLink', new_var(), new_var())
#            prem = Tree('EvaluationLink', new_var(), new_var())
#            imp = Tree('ImplicationLink', prem, conc)
#            
#            self.add_rule(Rule(conc, 
#                                         [imp, prem], 
#                                          name='ModusPonens_Eval'))

#        for type in ['ImplicationLink']:
#            conc = Tree('EvaluationLink', a.add_node(t.PredicateNode, 'B'))
#            prem = Tree('EvaluationLink', a.add_node(t.PredicateNode, 'A'))
#            imp = Tree('ImplicationLink', prem, conc)
#            
#            self.add_rule(Rule(conc, 
#                                         [imp, prem], 
#                                          name='ModusPonens_AB'))

        # AND/OR
        for type in ['AndLink', 'OrLink']:
            for size in xrange(5):
                args = [new_var() for i in xrange(size+1)]
                self.add_rule(Rule(Tree(type, args),
                                   args,
                                   type[:-4]))

        # Adding a NOT
        self.add_rule(Rule(Tree('NotLink', 1),
                           [ Tree(1) ],
                           name = 'Not'))

        # Link conversion
        self.add_rule(Rule(Tree('InheritanceLink', 1, 2),
                           [ Tree('SubsetLink', 1, 2) ],
                           name = 'Link2Link'))

        # This may cause weirdness with things matching too eagerly...
#       # Both of these rely on the policy that tree_from_atom replaces VariableNodes in the AtomSpace with the variables the tree class uses.
#        fact = new_var()
#        list_link = new_var()
#        r = Rule(
#                        fact,
#                        [Tree('ForAllLink', list_link, fact )], 
#                        name = 'ForAll'     
#                    )
#        r.tv = True
#        self.add_rule(r)

        for atom in a.get_atoms_by_type(t.ForAllLink):
            # out[0] is the ListLink of VariableNodes, out[1] is the expression
            tr = tree_from_atom(atom.out[1])
            r = Rule(tr, [], name='ForAll')
            r.tv = True
            self.add_rule(r)

        Chainer._convert_atoms = False

class Rule :
    def __init__ (self, head, goals, name, tv = False):
        if Chainer._convert_atoms:
            self.head = tree_with_fake_atoms(head)
            self.goals = map(tree_with_fake_atoms, goals)
        else:
            self.head = head
            self.goals = goals

        self.name = name
        self.tv = tv

        self.bc_depth = 0

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
        rep = self.name + '\n'
        rep += ' '*self.bc_depth*3
        rep += str(self.head)
        rep += '\n'
        for goal in self.goals :
            rep += ' '*(self.bc_depth*3+3)
            rep += str(goal) + '\n'
        return rep

    def standardize_apart(self):
        head_goals = (self.head,)+tuple(self.goals)
        tmp = standardize_apart(head_goals)
        new_version = Rule(tmp[0], tmp[1:], name=self.name, tv = self.tv)

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
        new_rule = Rule(new_head, new_goals, name=self.name, tv = self.tv)
        return new_rule

    def category(self):
        '''Returns the category of this rule. It can be either an axiom, a PLN Rule (e.g. Modus Ponens), or an
        application. An application is a PLN Rule applied to specific arguments.'''
        if self.name == '[axiom]':
            return 'axiom'
        elif self.name.startswith('[application]'):
            return 'application'
        else:
            return 'rule'

def test(a):
    c = Chainer(a)

    #search(Tree('EvaluationLink',a.add_node(t.PredicateNode,'B')))
    #fc(a)

    #c.bc(Tree('EvaluationLink',a.add_node(t.PredicateNode,'A')))

#    global rules
#    A = Tree('EvaluationLink',a.add_node(t.PredicateNode,'A'))
#    B = Tree('EvaluationLink',a.add_node(t.PredicateNode,'B'))
#    rules.append(Rule(B, 
#                                  [ A ]))

    c.bc(Tree('EvaluationLink',a.add_node(t.PredicateNode,'B')))


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
