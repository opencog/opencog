from opencog.atomspace import AtomSpace, types, Atom, Handle, TruthValue
import opencog.cogserver
from tree import *
from adaptors import *
from util import pp

import sys, copy

rules    = []
trace    = 0
goalId   = 100

# hacking. for the lulz

#kb = set([1, 3])
#rules = {(1, 2),  (3, 4)}
#
#def fc():
#    layer_results = set()
#    for (premise,  conclusion) in rules:
#        if premise in kb:
#            layer_results.add(conclusion)
#        
#    kb.update(layer_results)
#    return layer_results
#
#print fc()
#print kb

#def fc(a):
#    layer_results = set()
#    
#    rules = find(imp_template, all_atoms(a))
#    facts = find(all_template, all_atoms(a))
#    
#    for r in rules:
#        (premise,  conclusion) = r.out
#        if premise in facts:
#            layer_results.add(conclusion)
#        
#    for atom in layer_results:
#        atom.tv = TruthValue(1, 1)
#        #atom_from_tree(t, a)
#    return layer_results
#
#def all_atoms(a):
#    return a.get_atoms_by_type(t.Atom)
#
#all_template = tree(1)
#eval_template = tree('EvaluationLink', 1, tree('ListLink', 2, 3))
#imp_template = tree('ImplicationLink', 1,  2)

class Chainer:
    def __init__(self, space):
        self.space = space
        self.viz = PLNviz(space)
        self.setup_rules(space)
    
    def fc(self):
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
        return results

    def expand_target(self, target, stack, depth):
        print ' '*depth+'expand_target', pp(target)
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
            return []

        if depth > 3:
            print ' '*depth+'tacky maximum depth reached'
            return []

        for x in stack:
            if unify(target, x, {}, True) != None:
                print ' '*depth+'loop'
                return []

        for r in self.rules:
            head_goals = (r.head,)+tuple(r.goals)
            tmp = standardize_apart(head_goals)
            r = Rule(tmp[0], tmp[1:])
            s = {}
            
            s = unify(r.head, target, {})
            if s != None:
                child_results = self.apply_rule(target, r, 0, s, stack+[target], depth)
                results+=child_results
        return results

    def apply_rule(self, target, rule, goals_index, s, stack, depth):
        if goals_index == len(rule.goals):
            self.viz.declareResult(target)
            return [s]
        
        goal = rule.goals[goals_index]
        goal = subst(s, goal)

        print ' '*depth+'apply_rule //', target,'// rule =', rule, '// goal =',  goal, '// index =', goals_index

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
                self.rules.append(Rule(tr))
        
        # Deduction
        for type in ['SubsetLink', 'ImplicationLink', 'AssociativeLink']:
            self.rules.append(Rule(tree(type, 1,3), 
                                         [tree(type, 1, 2),
                                          tree(type, 2, 3) ]))

        # Inversion
        for type in ['SubsetLink', 'ImplicationLink']:
            self.rules.append(Rule( tree(type, 1, 2), 
                                         [tree(type, 2, 1)]))

        # ModusPonens
        for type in ['ImplicationLink']:
            self.rules.append(Rule(tree(2), 
                                         [tree(type, 1, 2),
                                          tree(1) ]))
        
        # AND/OR
        for type in ['AndLink', 'OrLink']:
            for size in xrange(5):
                args = [new_var() for i in xrange(size+1)]
                self.rules.append(Rule(tree(type, args),
                                   args))
        
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
            self.rules.append(Rule(tr))


class Rule :
#    def __init__ (self, s) :   # expect "term:-term,term,..."
#        flds = s.split(":-")
#        self.head = Term(flds[0])
#        self.goals = []
#        if len(flds) == 2 :
#            flds = split(flds[1],",")
#            for fld in flds : self.goals.append(Term(fld))

    def __init__ (self, head, goals = []) :
        self.head = head
        self.goals = goals

    def __repr__ (self) :
        rep = str(self.head)
        sep = " :- "
        for goal in self.goals :
            rep += sep + str(goal)
            sep = ","
        return rep

def search (term) :
    print "Query: ", str(term)
    global trace, rules
    # pop will take item from end, insert(0,val) will push item onto queue
    goal = Goal(Rule("JUSTICE", []))      # Anything- just get a rule object
    goal.rule.goals = [term]                  # target is the single goal
    queue = [goal]                            # Start our search
    while queue :
        c = queue.pop()                       # Next goal to consider
        if trace : print "Deque", c
        if c.inx >= len(c.rule.goals) :       # Is this one finished?
            if c.parent == None :            # Yes. Our original goal?
#                if c.env : print pp(c.env)         # Yes. tell user we
#                else     : print "Yes"          # have a solution
                target = subst(c.env, c.rule.goals[0])
                print "Result:", pp(target)
                continue
            assert c.parent != None
#            parent = copy.deepcopy(c.parent)  # Otherwise resume parent goal
            parent = c.parent.clone()  # Otherwise resume parent goal
            parent.env = unify(c.rule.head, parent.rule.goals[parent.inx], c.env)
#            unify (c.rule.head,    c.env,
#                   parent.rule.goals[parent.inx],parent.env)
            parent.inx = parent.inx+1         # advance to next goal in body
            queue.insert(0,parent)            # let it wait its turn
            if trace : print "Queue parent", parent
            continue

        # No. more to do with this goal.
        term = c.rule.goals[c.inx]            # What we want to solve

        for rule in rules :                   # Walk rule database
            child = Goal(rule, c)               # A possible subgoal
#            ans = unify (term, c.env, rule.head, child.env)
            child.env = unify(term, rule.head, c.env)
            if child.env != None:                    # if unifies, queue it up
                queue.insert(0,child)
                if trace : print "Queue child", child


#class Goal :
#    def __init__ (self, rule, parent=None, env={}) :
#        global goalId
#        goalId += 1
#        self.id = goalId
#        self.rule = rule
#        self.parent = parent
#        self.env = copy.copy(env)
#        self.inx = 0      # start search with 1st subgoal
#
#    def clone(self):
#        return Goal(self.rule, self.parent, self.env)
#
#    def __repr__ (self) :
#        return "Goal %d rule=%s inx=%d env=%s" % (self.id,self.rule,self.inx,self.env)
#
## A Goal is a rule in at a certain point in its computation. 
## env contains definitions (so far), inx indexes the current term
## being satisfied, parent is another Goal which spawned this one
## and which we will unify back to when this Goal is complete.
##
#
#def search (term) :
#    print "Query: ", str(term)
#    global trace, rules
#    # pop will take item from end, insert(0,val) will push item onto queue
#    goal = Goal(Rule("JUSTICE", []))      # Anything- just get a rule object
#    goal.rule.goals = [term]                  # target is the single goal
#    queue = [goal]                            # Start our search
#    while queue :
#        c = queue.pop()                       # Next goal to consider
#        if trace : print "Deque", c
#        if c.inx >= len(c.rule.goals) :       # Is this one finished?
#            if c.parent == None :            # Yes. Our original goal?
##                if c.env : print pp(c.env)         # Yes. tell user we
##                else     : print "Yes"          # have a solution
#                target = subst(c.env, c.rule.goals[0])
#                print "Result:", pp(target)
#                continue
#            assert c.parent != None
##            parent = copy.deepcopy(c.parent)  # Otherwise resume parent goal
#            parent = c.parent.clone()  # Otherwise resume parent goal
#            parent.env = unify(c.rule.head, parent.rule.goals[parent.inx], c.env)
##            unify (c.rule.head,    c.env,
##                   parent.rule.goals[parent.inx],parent.env)
#            parent.inx = parent.inx+1         # advance to next goal in body
#            queue.insert(0,parent)            # let it wait its turn
#            if trace : print "Queue parent", parent
#            continue
#
#        # No. more to do with this goal.
#        term = c.rule.goals[c.inx]            # What we want to solve
#
#        for rule in rules :                   # Walk rule database
#            child = Goal(rule, c)               # A possible subgoal
##            ans = unify (term, c.env, rule.head, child.env)
#            child.env = unify(term, rule.head, c.env)
#            if child.env != None:                    # if unifies, queue it up
#                queue.insert(0,child)
#                if trace : print "Queue child", child

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

import pygephi
class NullPLNviz:
    '''Simply use the null-object pattern, rather than adding lots of if statements everywhere.'''
    def __init__(self, space):
        pass
    
    def outputTarget(self, target, parent, index, rule = None):
        pass
    
    def declareResult(self, target):
        pass

class PLNviz:

    def __init__(self, space):
        self._as = space
        self.g = pygephi.JSONClient('http://localhost:8080/workspace0', autoflush=True)
        self.g.clean()
        self.node_attributes = {'size':10, 'r':0.0, 'g':0.0, 'b':1.0}
        self.rule_attributes = {'size':10, 'r':0.0, 'g':1.0, 'b':1.0}
        self.root_attributes = {'size':20, 'r':1.0, 'g':1.0, 'b':1.0}
        self.result_attributes = {'r':1.0, 'b':0.0, 'g':0.0}

    def outputTarget(self, target, parent, index, rule=None):
        
        #target_id = str(hash(target))
        target_id = str(target)
        
        if parent == None:
            self.g.add_node(target_id, label=str(target), **self.root_attributes)

        if parent != None:
            self.g.add_node(target_id, label=str(target), **self.node_attributes)
            
            #parent_id = str(hash(parent))
            #link_id = str(hash(target_id+parent_id))
            parent_id = str(parent)
            rule_app_id = 'rule '+str(rule)+parent_id
            target_to_rule_id = rule_app_id+target_id
            parent_to_rule_id = rule_app_id+' parent'
            
            self.g.add_node(rule_app_id, label=str(rule), **self.rule_attributes)

            self.g.add_node(parent_id, label=str(parent), **self.node_attributes)
            
            # Link parent to rule app
            self.g.add_edge(parent_to_rule_id, rule_app_id, parent_id, directed=True, label='')
            # Link rule app to target
            self.g.add_edge(target_to_rule_id, target_id, rule_app_id, directed=True, label=str(index+1))

    def declareResult(self, target):
        
        target_id = str(target)
        self.g.change_node(target_id, **self.result_attributes)
        
        #self.g.add_node(target_id, label=str(target), **self.result_attributes)

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

print __name__
if __name__ == "__main__":
    #    a = AtomSpace()
    #    t=types
    #    bob = a.add_node(t.ConceptNode, "Bob")
    #    alice = a.add_node(t.ConceptNode, "Alice")
    #    link = a.add_link(t.ListLink, [bob, alice])
    #    link2 = a.add_link(t.ListLink, [alice, bob])
    #
    #    link3 = a.add_link(t.EvaluationLink, [a.add_node(t.PredicateNode, "likes"), link2])
    #
    #    obj1 = a.add_node(t.AccessoryNode, 'ball1')
    #    obj2 = a.add_node(t.StructureNode, 'tree1')
    #    next = a.add_link(t.EvaluationLink,
    #                   [a.add_node(t.PredicateNode, 'next'),
    #                    a.add_link(t.ListLink, [obj1, obj2])])
    #
    #    next2 = a.add_link(t.EvaluationLink,
    #                   [a.add_node(t.PredicateNode, 'next'),
    #                    a.add_link(t.ListLink, [obj2, obj1])])
    #
    #    next.tv = TruthValue(1, 1)
    #
    #    arity3 = a.add_link(t.AndLink, [bob, alice, obj1])
    #
    #    time = a.add_link(t.AtTimeLink, [a.add_node(t.TimeNode, "t-0"), a.add_node(t.ConceptNode, "blast-off")])
    #
    #    eval_arity1 = a.add_link(t.EvaluationLink, [a.add_node(t.PredicateNode, "is_edible"),
    #                    a.add_link(t.ListLink, [a.add_node(t.ConceptNode, "bowl123")])])
    #    eval_arity1.tv = TruthValue(1,  1)
    #
    #    #    f = FishgramFilter(a,SubdueTextOutput(a))
    #    #
    #    ##    d = DottyOutput(a)
    #    ##    g = GraphConverter(a,d)
    #    #
    #    ##    g = GraphConverter(a,SubdueTextOutput(a))
    #    #    g = GraphConverter(a, f)
    #    #
    #    #    g.output()
    #
    #    for obj in a.get_atoms_by_type(t.Atom):
    #        if obj.tv.count > 0:
    #            tr = tree_from_atom(obj)
    #            rules.append(Rule(tr))
    #
    #    #trace=1
    #    for  obj in a.get_atoms_by_type(t.Atom):
    #        if obj.tv.count > 0:
    #            tr = tree_from_atom(obj)
    #            search(tr)
    #
    #    r = Rule(tree(bob), [tree(alice)])
    #    rules.append(r)
    #    rules.append(Rule(tree(alice)))
    #    search(tree(alice))
    #
    #    for tr in [all_template, eval_template, imp_template]:
    #        search(tr)

    test(a)
