from opencog.atomspace import AtomSpace, types, Atom, Handle, TruthValue
import opencog.cogserver
from tree import *
from fishgram import *
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
        
class Goal :
    def __init__ (self, rule, parent=None, env={}) :
        global goalId
        goalId += 1
        self.id = goalId
        self.rule = rule
        self.parent = parent
        self.env = copy.copy(env)
        self.inx = 0      # start search with 1st subgoal

    def clone(self):
        return Goal(self.rule, self.parent, self.env)

    def __repr__ (self) :
        return "Goal %d rule=%s inx=%d env=%s" % (self.id,self.rule,self.inx,self.env)

# A Goal is a rule in at a certain point in its computation. 
# env contains definitions (so far), inx indexes the current term
# being satisfied, parent is another Goal which spawned this one
# and which we will unify back to when this Goal is complete.
#

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

def setup_rules(a):
    global rules

    # All existing Atoms
    for obj in a.get_atoms_by_type(t.Atom):
        if obj.tv.count > 0:
            tr = tree_from_atom(obj)
            rules.append(Rule(tr))
    
#    # Deduction
#    for type in ['SubsetLink', 'ImplicationLink']:
#        rules.append(Rule(tree(type, 1,3), 
#                                     [tree(type, 1, 2),
#                                      tree(type, 2, 3) ]))

    for type in ['ImplicationLink']:
        rules.append(Rule(tree(2), 
                                     [tree(type, 1, 2),
                                      tree(1) ]))

def test(a):
    setup_rules(a)
    search(tree('EvaluationLink',a.add_node(t.PredicateNode,'B')))

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

    setup_rules(a)
    logic.search(tree.tree('EvaluationLink',a.add_node(t.PredicateNode,'B')))
    
