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
    
    def fc(self):
        #import pdb; pdb.set_trace()
        
        facts = {r.head for r in self.rules if not r.goals}
        real_rules = {r for r in self.rules if r.goals}

        # we have rules which are goal:-term,term,term,...
        # and include rules with no arguments.
        layer_facts = set()    
        
        for r in real_rules:
            print pp(r)
            
            # In case we prove something that still has a variable in it
            #r = Rule(standardize_apart(r.head), standardize_apart(tuple(r.goals)))
            s = {}
            
            matches = find_matching_conjunctions(tuple(r.goals), facts)
            
            for m in matches:
                result = subst(m.subst, r.head)
                #res = Rule(result, [])
                layer_facts.add(result)            
                print '%s: %s <- %s' % (pp(r), pp(result), pp(m.conj))
                
                atom = atom_from_tree(result, self.space)
                atom.tv = TruthValue(1, 1)
                
                for i, input in enumerate(m.conj):
                    self.viz.outputTarget(input, result , i, r)
                self.viz.declareResult(result)
        
        # Add the facts somewhere more permanent
        return layer_facts

    def bc(self, target):
        self.viz.outputTarget(target, None, 0)
        results = self.expand_target(target, [], depth=1)
        
        #results = [atom_from_tree(subst(s, target), self.space) for s in results]
        #print 'PythonPLN results: ', pp(results)
        
        #return results
        
        # Just return a single result for simplicity
        s = next(results)
        result1 = atom_from_tree(subst(s, target), self.space)
        print pp(result1)
        return [result1.h]

    def expand_target(self, target, stack, depth):
        #print ' '*depth+'expand_target', pp(target)
        # we have rules which are goal:-term,term,term,...
        # and include rules with no arguments.
        results = []

        # haxx to prevent searching for infinitely many nested ImplicationLinks
        # The unify-based version would match a single variable!
    #    bad = tree('ImplicationLink',
    #                     new_var(), 
    #                     tree('ImplicationLink', new_var(), new_var())
    #                    )
    #    if unify(bad, target, {}) != None:
        if target.get_type() == t.ImplicationLink and len(target.args) == 2 and target.args[1].get_type() == t.ImplicationLink:
            print ' '*depth+'nested implication'
            #return []
            return

        if depth > 6:
            print ' '*depth+'tacky maximum depth reached'
            #return []
            return

        for x in stack:
            if unify(target, x, {}, True) != None:
                #print ' '*depth+'loop'
                #return []
                return

        for r in self.rules:
            r = r.standardize_apart()
            
            s = unify(r.head, target, {})
            if s != None:
                child_results = self.apply_rule(target, r, 0, s, stack+[target], depth)
                #results+=child_results
                for cr in child_results:
                    yield cr
        #return results

    def apply_rule(self, target, rule, goals_index, s, stack, depth):
        if goals_index == len(rule.goals):
            self.viz.declareResult(target)
            return [s]
        
        goal = rule.goals[goals_index]
        goal = subst(s, goal)

        #print ' '*depth+'apply_rule //', target,'// rule =', rule, '// goal =',  goal, '// index =', goals_index

        results = []

        self.viz.outputTarget(goal, target, goals_index, rule)
        child_results = self.expand_target(goal, stack, depth+1)
        
        for child_s in child_results:
            child_s.update(s)
            
            input = subst(child_s, goal)
            self.viz.outputTarget(input, target, goals_index, rule)
            self.viz.declareResult(input)
            
            results+= self.apply_rule(target, rule, goals_index+1, child_s, stack, depth)
        
        return results

    def setup_rules(self, a):
        self.rules = []

        # All existing Atoms
        for obj in a.get_atoms_by_type(t.Atom):
            if obj.tv.count > 0:
                tr = tree_from_atom(obj)
                self.rules.append(Rule(tr, [], '[axiom]'))
        
        # Deduction
        for type in ['SubsetLink', 'ImplicationLink', 'AssociativeLink']:
            self.rules.append(Rule(tree(type, 1,3), 
                                         [tree(type, 1, 2),
                                          tree(type, 2, 3) ], 
                                          name='Deduction'))

        # Inversion
        for type in ['SubsetLink', 'ImplicationLink']:
            self.rules.append(Rule( tree(type, 1, 2), 
                                         [tree(type, 2, 1)], 
                                         name='Inversion'))

        # ModusPonens
        for type in ['ImplicationLink']:
            self.rules.append(Rule(tree(2), 
                                         [tree(type, 1, 2),
                                          tree(1) ], 
                                          name='ModusPonens'))
        
        # AND/OR
        for type in ['AndLink', 'OrLink']:
            for size in xrange(5):
                args = [new_var() for i in xrange(size+1)]
                self.rules.append(Rule(tree(type, args),
                                   args,
                                   type[:-4]))
        
        # Both of these rely on the policy that tree_from_atom replaces VariableNodes in the AtomSpace with the variables the tree class uses.
    #    fact = new_var()
    #    list_link = new_var()
    #    self.rules.append(Rule(
    #                            tree(fact),
    #                            [tree('ForAllLink', list_link, fact )]
    #                        ))
        
        for atom in a.get_atoms_by_type(t.ForAllLink):
            # out[0] is the ListLink of VariableNodes, out[1] is the expression
            tr = tree_from_atom(atom.out[1])
            self.rules.append(Rule(tr, [], name='ForAll'))


class Rule :
    def __init__ (self, head, goals, name):
        self.head = head
        self.goals = goals
        self.name = name

    def __str__(self):
        return self.name

    def __repr__ (self) :
        rep = str(self.head)
        sep = " :- "
        for goal in self.goals :
            rep += sep + str(goal)
            sep = ","
        return rep
    
    def standardize_apart(self):
        head_goals = (self.head,)+tuple(self.goals)
        tmp = standardize_apart(head_goals)
        new_version = Rule(tmp[0], tmp[1:], name=self.name)
        
        return new_version

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
