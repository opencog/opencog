from opencog.atomspace import AtomSpace, types, Atom, Handle, TruthValue
import opencog.cogserver
from tree import *
from adaptors import find_matching_conjunctions
from util import pp

t = types

class Chainer:
    def __init__(self, space):
        self.space = space
        self.viz = PLNviz(space)
        self.viz.connect()
        self.setup_rules(space)
        self.proven = []
    
    # Either need to allow a rule-instance to be filled by two things, or reapply old premises,
    # or check for premises on each new rule-instance.
    def fc(self):
        self.proven = [r for r in self.rules if not len(r.goals)]
            
#        while self.proven:
#            r = self.proven.pop()
        for r in self.proven:
            r = r.standardize_apart()
            # add it into any premises
            partly_satisfied_rules = self.apply_rules_with_matching_premises(r.head)
            # check the "filled" rules and add their result to proven
            for possibly_satisfied in partly_satisfied_rules:
                # viz
                for i, input in enumerate(possibly_satisfied.goals):
                    self.viz.outputTarget(input, possibly_satisfied.head, i, possibly_satisfied)
                self.viz.declareResult(possibly_satisfied.head)
                
                if all(map(self.known, possibly_satisfied.goals)):
                    possibly_satisfied.tv = True
                    self.add_rule(possibly_satisfied)
                    self.proven.append(possibly_satisfied)

    # Note: It would be simpler to just skip the goal pool and use a rule pool instead.
    # Should also apply rules once they are found (and only up through existing specialized
    # rules, not through general rules)
    def bc(self, target):
        goal = Rule(None, target, name="goal")
        self.bc_pool = [goal]
        
        while self.bc_pool:
            next_target = self.bc_pool.pop()
            self.query_rules_with_matching_conclusion(next_target)
        
        return []
    
    def query_rules_with_matching_conclusion(self, target):
        for r in self.rules:
            r = r.standardize_apart()
            s = unify(r.head, target, {})
            if s != None:
                r2 = r.subst(s)
                # Add the specialized rule and then add the subgoals to the backchaining pool
                if self.add_rule(r2):
                    for new_goal in r2.goals:
                        existing = [g for g in self.bc_pool if unify(new_goal, g, {}, True) != None]
                        if not existing:
                            self.bc_pool.append(new_goal)

    def apply_rules_with_matching_premises(self, premise):
        conclusions = []
        for r in self.rules:
            for i, pr in enumerate(r.goals):
                s = unify(pr, premise, {})
                if s != None:
                    filled_in = r.subst(s)
                    if self.add_rule(filled_in):
                        conclusions.append(filled_in)
        return conclusions
    
    def add_rule(self, rule):
        '''Adds a rule, checking for redundancy (i.e. if it is exactly isomorphic to another rule)'''
        matches = [r for r in self.rules if rule.isomorphic(r)]
        
        if not matches:
            self.rules.append(rule)
            print 'added %s rule %s %s' % (rule.tv, rule, repr(rule))
            return True
        else:
            #print 'isomorphic; matches: ', len(matches)
            if rule.tv:
                print '***proved rule', repr(rule)
                return False

    def known(self, expr):
        '''Check whether a TruthValue has been found for expr'''
        matches = [r for r in self.rules if unify(expr, r.head, {}, True) != None]
        
        return any(m.tv for m in matches)

    def setup_rules(self, a):
        self.rules = []

        # All existing Atoms
        for obj in a.get_atoms_by_type(t.Atom):
            if obj.tv.count > 0:
                tr = tree_from_atom(obj)
                self.add_rule(Rule(tr, [], '[axiom]'))
        
#        # Deduction
#        for type in ['SubsetLink', 'ImplicationLink', 'AssociativeLink']:
#            self.add_rule(Rule(tree(type, 1,3), 
#                                         [tree(type, 1, 2),
#                                          tree(type, 2, 3) ], 
#                                          name='Deduction'))
#
#        # Inversion
#        for type in ['SubsetLink', 'ImplicationLink']:
#            self.add_rule(Rule( tree(type, 1, 2), 
#                                         [tree(type, 2, 1)], 
#                                         name='Inversion'))

        # ModusPonens
        for type in ['ImplicationLink']:
            self.add_rule(Rule(tree(2), 
                                         [tree(type, 1, 2),
                                          tree(1) ], 
                                          name='ModusPonens'))
        
#        # AND/OR
#        for type in ['AndLink', 'OrLink']:
#            for size in xrange(5):
#                args = [new_var() for i in xrange(size+1)]
#                self.add_rule(Rule(tree(type, args),
#                                   args,
#                                   type[:-4]))
#        
#        # Both of these rely on the policy that tree_from_atom replaces VariableNodes in the AtomSpace with the variables the tree class uses.
#    #    fact = new_var()
#    #    list_link = new_var()
#    #    self.add_rule(Rule(
#    #                            tree(fact),
#    #                            [tree('ForAllLink', list_link, fact )]
#    #                        ))
#        
        for atom in a.get_atoms_by_type(t.ForAllLink):
            # out[0] is the ListLink of VariableNodes, out[1] is the expression
            tr = tree_from_atom(atom.out[1])
            self.add_rule(Rule(tr, [], name='ForAll'))

        proven = [r for r in self.rules if not len(r.goals)]
        # Only done here until we introduce TVs
        for r in proven:
            r.tv = True
            self.add_rule(r)


class Rule :
    def __init__ (self, head, goals, name):
        self.head = head
        self.goals = goals
        self.name = name
        self.tv = False

    def __str__(self):
        return self.name

    def __repr__ (self) :
        rep = repr(self.head)
        sep = " :- "
        for goal in self.goals :
            rep += sep + repr(goal)
            sep = ","
        return rep
    
    def standardize_apart(self):
        head_goals = (self.head,)+tuple(self.goals)
        tmp = standardize_apart(head_goals)
        new_version = Rule(tmp[0], tmp[1:], name=self.name)
        
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
        new_rule = Rule(new_head, new_goals, name=self.name)
        return new_rule

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
