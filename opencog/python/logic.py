from opencog.atomspace import AtomSpace, types, Atom, Handle, TruthValue
import opencog.cogserver
from tree import *
from adaptors import find_matching_conjunctions
from util import pp

from profilestats import profile

t = types

class Chainer:
    def __init__(self, space):
        self.space = space
        self.viz = PLNviz(space)
        self.viz.connect()
        self.setup_rules(space)
        self.apps = []
        self.bc_later = []
        self.bc_before = []

    @profile
    def bc(self, target):
        try:
            from time import time
            
            print '='*40+'\nbc', pp(target), '\n','='*40, '\n'
            self.bc_later = [target]
            self.target = target
            self.results = []

            start = time()
            while self.bc_later and not self.results:
                
                self.bc_step()
                
                print time() - start
                if time() - start > 10:
                    print 'TIMEOUT'
                    assert 0
                    break

            return [atom_from_tree(result, self.space).h for result in self.results]
        except Exception, e:
            import traceback, pdb
            
            print traceback.format_exc(10)
            # Start the post-mortem debugger
            #pdb.pm()
            return []
    
    def bc_step(self):
        next_target = self.bc_later.pop(0) # Breadth-first search
        self.bc_before.append(next_target)
        #next_target = self.bc_pool.pop() # Depth-first search
        
        for a in self.find_rule_applications(next_target):
            print repr(a)
            # PROPOGATE THIS RESULT UP THE BACKWARD CHAINING TREE
            
            got_result = self.check_premises_and_add_result(a)
            if got_result and a.head.isomorphic(self.target):
                self.results.append(a.head)
            
            self.add_queries_for_goals(a)
    
    # MUST ALSO ADD APPS AS QUERIES
    
    def add_queries_for_goals(self, app):
        def contains_isomorphic_tree(tr, collection):
            return any(tr.isomorphic(existing) for existing in collection)
        
        for goal in app.goals:
            if     (not contains_isomorphic_tree(goal, self.bc_before) and
                    not contains_isomorphic_tree(goal, self.bc_later) ):
                self.bc_later.append(goal)
    
    def check_premises_and_add_result(self, app):
        '''Check whether the given app can produce a result. This will happen if all its premises are
        already proven. Or if it is one of the axioms given to PLN initially.'''
        input_tvs = [self.get_tvs(input) for input in app.goals]
        if all(input_tvs):
            print 'coz that\'s what gets results'
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

    def find_existing_rule(self, rule):
        matches = [r for r in self.rules if r.isomorphic(rule)]
        assert len(matches) < 2
        return matches

    def get_tvs(self, expr):
        # NOTE: It may be easier to just do this by just storing the TVs for each target.
        rs = self.find_rule_applications(expr)
        
        rs = [r for r in rs if expr.isomorphic(r.head)]
        
        return [r.tv for r in rs if r.tv]

    def add_rule(self, rule):        
        self.rules.append(rule)

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
        for type in ['SubsetLink', 'ImplicationLink', 'AssociativeLink', 'InheritanceLink']:
            self.add_rule(Rule(tree(type, 1,3), 
                                         [tree(type, 1, 2),
                                          tree(type, 2, 3) ], 
                                          name='Deduction'))

        # Inversion
        for type in ['SubsetLink', 'ImplicationLink', 'InheritanceLink']:
            self.add_rule(Rule( tree(type, 1, 2), 
                                         [tree(type, 2, 1)], 
                                         name='Inversion'))

        # Try using a more specialized MP as well

        # ModusPonens
        for type in ['ImplicationLink']:
            self.add_rule(Rule(tree(2), 
                                         [tree(type, 1, 2),
                                          tree(1) ], 
                                          name='ModusPonens'))
#        # ModusPonens for EvaluationLinks only
#        for type in ['ImplicationLink']:
#            conc = tree('EvaluationLink', new_var(), new_var())
#            prem = tree('EvaluationLink', new_var(), new_var())
#            imp = tree('ImplicationLink', prem, conc)
#            
#            self.add_rule(Rule(conc, 
#                                         [imp, prem], 
#                                          name='ModusPonens_Eval'))
        
        # AND/OR
        for type in ['AndLink', 'OrLink']:
            for size in xrange(5):
                args = [new_var() for i in xrange(size+1)]
                self.add_rule(Rule(tree(type, args),
                                   args,
                                   type[:-4]))
        
        # Both of these rely on the policy that tree_from_atom replaces VariableNodes in the AtomSpace with the variables the tree class uses.
    #    fact = new_var()
    #    list_link = new_var()
    #    self.add_rule(Rule(
    #                            tree(fact),
    #                            [tree('ForAllLink', list_link, fact )]
    #                        ))
        
        for atom in a.get_atoms_by_type(t.ForAllLink):
            # out[0] is the ListLink of VariableNodes, out[1] is the expression
            tr = tree_from_atom(atom.out[1])
            r = Rule(tr, [], name='ForAll')
            r.tv = True
            self.add_rule(r)


class Rule :
    def __init__ (self, head, goals, name, tv = False):
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
    
    #search(tree('EvaluationLink',a.add_node(t.PredicateNode,'B')))
    #fc(a)

    #c.bc(tree('EvaluationLink',a.add_node(t.PredicateNode,'A')))

#    global rules
#    A = tree('EvaluationLink',a.add_node(t.PredicateNode,'A'))
#    B = tree('EvaluationLink',a.add_node(t.PredicateNode,'B'))
#    rules.append(Rule(B, 
#                                  [ A ]))

    c.bc(tree('EvaluationLink',a.add_node(t.PredicateNode,'B')))


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

import pygephi
class PLNviz:

    def __init__(self, space):
        self._as = space
        self.node_attributes = {'size':10, 'r':0.0, 'g':0.0, 'b':1.0}
        self.rule_attributes = {'size':10, 'r':0.0, 'g':1.0, 'b':1.0}
        self.root_attributes = {'size':20, 'r':1.0, 'g':1.0, 'b':1.0}
        self.result_attributes = {'r':1.0, 'b':0.0, 'g':0.0}
        
        self.connected = False

    def connect(self):
        try:
            self.g = pygephi.JSONClient('http://localhost:8080/workspace0', autoflush=True)
            self.g.clean()
            self.connected = True
        except URLError:
            self.connected = False

    @check_connected
    def outputTarget(self, target, parent, index, rule=None):
        if not self.connected:
            return
        
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
        if not self.connected:
            return
        
        target_id = str(target)
        self.g.change_node(target_id, **self.result_attributes)
        
        #self.g.add_node(target_id, label=str(target), **self.result_attributes)

    @check_connected
    # More suited for Fishgram
    def outputTreeNode(self, target, parent, index):
        if not self.connected:
            return
        
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
