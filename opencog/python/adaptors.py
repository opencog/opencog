from opencog.atomspace import AtomSpace, types, Atom, Handle, TruthValue
import opencog.cogserver
from tree import *
from util import *

# to debug within the cogserver, try these, inside the relevant function:
#import code; code.interact(local=locals())
#import ipdb; ipdb.set_trace()

t = types

class ForestExtractor:
    """Extracts a forest of trees, where each tree is a Link (and children) that are true in the AtomSpace.
    The trees may share some of the same Nodes."""
    def __init__(self, atomspace, writer):
        self.a = atomspace
        self.writer = writer
        
        # policy
        # Whether to create miner-friendly output, rather than human-friendly output.
        # Makes it output all object-nodes with the same label. May be more useful for visualisation anyway.
        self.miner_friendly = True
        # Only affects output
        self.compact_binary_links = True
        self.unwanted_atoms = set(["proximity", "near", 'next', "AGISIM_rotation", "AGISIM_position", "AGISIM_velocity", "SpaceMap", "inside_pet_fov", 'turn', 'walk',
                                    # These ones make it ignore physiological feelings; it'll only care about the corresponding DemandGoals
                                    'pee_urgency', 'poo_urgency', 'energy', 'fitness', 'thirst'])
        
        # state
        self.all_objects  = set()# all objects in the AtomSpace
        self.all_trees = []
        self.all_trees_atoms = []
        self.bindings = []
        # variable counter
        # NOTE: If you set it to 0 here, it will give unique variables to every tree. BUT it will then count every occurrence of
        # a tree as different (because of the different variables!)
        #self.i = 0
        
        # fishgram-specific experiments. Refactor later
        # map from unique tree to set of embeddings. An embedding is a set of bindings. Maybe store the corresponding link too.
        self.tree_embeddings = {} 
        
        # The incoming links (or rather trees/predicates) for each object.
        # For each object, for each predsize, for each slot, the list of preds. (Indexes into self.all_trees)
        self.incoming = {}

    class UnwantedAtomException(Exception):
        pass

    def extractTree(self,  atom, objects):
        if not self.include_atom(atom):
            raise self.UnwantedAtomException
        elif self.is_object(atom):
            objects.append(atom)
            self.i+=1
            return Tree(self.i-1)
        elif atom.is_node():
            return Tree(atom)
        else:
            args = [self.extractTree(x,  objects) for x in atom.out]
            return Tree(atom.type_name, args)

    def extractForest(self):

        # TODO >0.5 for a fuzzy link means it's true, but probabilistic links may work differently        
        for link in [x for x in self.a.get_atoms_by_type(t.Link) if x.tv.mean > 0.5 and x.tv.confidence > 0]:
            if not self.include_tree(link): continue
            
            objects = []            
            #print self.extractTree(link, objects),  objects, self.i
            self.i = 0
            try:
                tree = self.extractTree(link, objects)
            except(self.UnwantedAtomException):
                continue
            objects = tuple(objects)
            
            #print tree,  [str(o) for o in objects]
            
            # policy - throw out trees with no objects
            if len(objects):
                self.all_trees.append(tree)
                self.all_trees_atoms.append(link)
                self.bindings.append(objects)
                for obj in objects:
                    self.all_objects.add(obj)
                    
            # fishgram-specific
            if tree not in self.tree_embeddings:
                self.tree_embeddings[tree] = []
            substitution = subst_from_binding(objects)
            self.tree_embeddings[tree].append(substitution)
            
            size= len(objects)
            tree_id = len(self.all_trees) - 1
            for slot in xrange(size):
                obj = objects[slot]
                
                if obj not in self.incoming:
                    self.incoming[obj] = {}
                if size not in self.incoming[obj]:
                    self.incoming[obj][size] = {}
                if slot not in self.incoming[obj][size]:
                    self.incoming[obj][size][slot] = []
                self.incoming[obj][size][slot].append(tree_id)
        
        # Make all bound trees. Enables using lookup_embeddings
        self.all_bound_trees = [subst(subst_from_binding(b), tr) for tr, b in zip(self.all_trees, self.bindings)]        

    def output_tree(self, atom,  tree,  bindings):
        vertex_name = str(tree)
        
        # policy
        if self.compact_binary_links and len(bindings) == 2:
            self.writer.outputLinkEdge(atom,  label=vertex_name,  outgoing=bindings)
        else:
            self.writer.outputLinkVertex(atom,  label=vertex_name)
            self.writer.outputLinkArgumentEdges(atom, outgoing=bindings)

    def output(self):
        self.writer.start()
        self.extractForest()
        for obj in self.all_objects:
            # policy
            if self.miner_friendly and self.is_object(obj):
                self.writer.outputNodeVertex(obj, self.object_label(obj))
            else:
                self.writer.outputNodeVertex(obj)
        for i in xrange(len(self.all_trees)):
            self.output_tree(self.all_trees_atoms[i],  self.all_trees[i],  self.bindings[i])
        self.writer.stop()

    def is_object(self,  atom):
        return atom.is_a(t.ObjectNode) or atom.is_a(t.TimeNode) # haxx!
    
    def is_important_atom(self, atom):
        return atom.name in ['actionFailed', 'actionDone'] or "DemandGoal" in atom.name
    
    def object_label(self,  atom):
        return 'some_'+atom.type_name

    def include_atom(self,  atom):
        """Whether to include a given atom in the results. If it is not included, all trees containing it will be ignored as well."""
        if atom.is_node():
            if atom.name in self.unwanted_atoms:
                return False
        else:
            if (atom.is_a(t.SimultaneousEquivalenceLink) or atom.is_a(t.SimilarityLink) or atom.is_a(t.ImplicationLink) or atom.is_a(t.ReferenceLink) ):
                return False

        return True
    
    def include_tree(self,  link):
        """Whether to make a separate tree corresponding to this link. If you don't, links in its outgoing set can
        still get their own trees."""
#        if not link.is_a(t.SequentialAndLink):
#            return False

        # TODO check the TruthValue the same way as you would for other links.
        # work around hacks in other modules
        if any([i.is_a(t.AtTimeLink) for i in link.incoming]) or link.is_a(t.ExecutionLink):
            return False
        else:
            return True

    # tr = fish.forest.all_trees[0]
    # fish.forest.lookup_embeddings((tr,))
    def lookup_embeddings(self, conj):
        """Given a conjunction, do a naive search for all embeddings. Fishgram usually finds the embeddings as part of the search,
        which is probably more efficient. But this is simpler and guaranteed to be correct. So it is useful for testing and performance comparison.
        It could also be used to find (the embeddings for) extra conjunctions that fishgram has skipped
        (which would be useful for the calculations used when finding implication rules)."""

        # Find all bound trees
#        try:
#            self.all_bound_trees
#        except AttributeError:
#            self.all_bound_trees = [subst(subst_from_binding(b), t) for t, b in zip(self.all_trees, self.bindings)]
##        self.all_bound_trees = [subst(subst_from_binding(b), t) for t, b in zip(self.all_trees, self.bindings)]
        
        return self.lookup_embeddings_helper(conj, (), {}, self.all_bound_trees)

    def lookup_embeddings_helper(self, conj, bound_conj_so_far, s, all_bound_trees):

        if conj == ():            
            #return bound_conj_so_far
            return [s]

        # Find all compatible embeddings. Then commit to that tree
        tr = conj[0]

        ret = []
        substs = []
        matching_bound_trees = []
        s2 = ForestExtractor.magic_eval(self.a, tr, s)
        if s2 != None:
            substs.append(s2)
            bound_tr = subst(s2, tr)
            matching_bound_trees.append(bound_tr)
        else: # For efficiency, don't try looking it up it if it was magically evaluated
            for bound_tr in all_bound_trees:
                s2 = unify(tr, bound_tr, s)
                if s2 != None:
                    substs.append( s2 )
                    matching_bound_trees.append(bound_tr)

        for s2, bound_tr in zip(substs, matching_bound_trees):
            bc = bound_conj_so_far + ( bound_tr , )
            later = self.lookup_embeddings_helper(conj[1:], bc, s2, all_bound_trees)
            # Add the (complete) substitutions from lower down in the recursive search,
            # but only if they are not duplicates.
            # TODO I wonder why the duplication happens?
            for final_s in later:
                if final_s not in ret:
                    ret.append(final_s)
        
        return ret
    
    @staticmethod
    def magic_eval(atomspace, tr, s = {}):
        '''Evaluate special operators (as opposed to ordinary links in the atomspace).
        tr is a tree containing a link or operator. s is a substitution to use with tr.
        Currently the only operator is SequentialAndLink (the type). A SequentialAndLink
        between Atoms will be accepted if a) 1 or both of its arguments are only variables, or
        b) its arguments are TimeNodes within a certain interval (currently 30 seconds).
        If all possible valid links corresponding to this operator were explicitly recorded
        in the AtomSpace, you would be able to get the same effect by trying to unify tr
        against the trees for every Atom in the AtomSpace.
        
        NOTE: In a conjunction, the operator must be after any links that use the variables.'''
        # unit of timestamps is 0.01 second so multiply by 100
        interval = 100* 20
        
        # If this is called from lookup_embeddings_helper, s should contain specific TimeNodes
        tr = subst(s, tr)
        
        # use unify to extract a SequentialAndLink
        t1_var, t2_var = new_var(), new_var()
        template = Tree('SequentialAndLink', t1_var, t2_var)
        # template_s is just used to find the TimeNodes (to compare them with each other)
        template_s = unify(template, tr, s)
        if template_s == None:
            return None
        time1_tr = template_s[t1_var]
        time2_tr = template_s[t2_var]
        if time1_tr.is_variable() or time2_tr.is_variable():
            return s
        
        time1_atom = time1_tr.op
        time2_atom = time2_tr.op
        
        t1 = int(time1_atom.name)
        t2 = int(time2_atom.name)
        if 0 < t2 - t1 <= interval:
            return s
        else:
            return None

class GraphConverter:
    def __init__(self, a, writer):
        self.a = a
        self.writer = writer
        self.compact = True

    def addVertex(self,atom):
        if atom.is_node():
            self.writer.outputNodeVertex(atom)
        elif not self.compact or not self.is_compactable(atom):
            self.writer.outputLinkVertex(atom)

    def addEdges(self,atom):
        if atom.is_node():
            return
        elif self.compact and self.is_compactable(atom):
            self.writer.outputLinkEdge(atom)
        else:
            self.writer.outputLinkArgumentEdges(atom)

    def output(self):
        self.writer.start()
        # Use inline generators when you want to return all the results (or use a callback?)
        try:
            for atom in self.sorted_by_handle(self.a.get_atoms_by_type(t.Atom)):
                #print atom
                self.addVertex(atom)
            for atom in self.sorted_by_handle(self.a.get_atoms_by_type(t.Link)):
                self.addEdges(atom)
        except Exception, e:
            print e.__class__,  str(e)
            import pdb; pdb.set_trace()
        self.writer.stop()

    # This got a bit convoluted. The reason why it's necessary is to make sure children of an atom will be
    # output first (although there would be other ways to do that)
    def sorted_by_handle(self,atoms):
        handles = [atom.h for atom in atoms]
        handles.sort()
        return [Atom(h,self.a) for h in handles]
    
    def is_compactable(self,atom):
        return atom.arity == 2 and len(atom.incoming) == 0 and not FishgramFilter.is_application_link(atom) # TODO haxx?

import pygephi
class GephiOutput:

    def __init__(self, space):
        self._as = space
        self.g = pygephi.JSONClient('http://localhost:8080/workspace0', autoflush=True)
        self.g.clean()
        self.node_attributes = {'size':10, 'r':0.0, 'g':0.0, 'b':1.0, 'x':1}

    def start(self):
        pass

    def stop(self):
        pass

    def outputNodeVertex(self, a, label = None):
        assert a.is_node()
        if label==None:
            label = '%s:%s' % (a.name, a.type_name)

        self.g.add_node(str(a.h.value()), label=label,  **self.node_attributes)

    def outputLinkEdge(self, a, label=None,outgoing=None):
        
        assert a.is_link()
        assert len(a.out) == 2
        assert (label==None) == (outgoing==None)

        if label==None:
            label = a.type_name

        if outgoing==None:
            outgoing = a.out

        (out0, out1) = outgoing[0].h.value(), outgoing[1].h.value()       
     
        self.g.add_edge(str(a.h.value()), out0, out1, directed=True, label=label)
       
    def outputLinkVertex(self, a, label=None):
        #import code; code.interact(local=locals())
        #import ipdb; ipdb.set_trace()
        assert a.is_link()
       
        if label==None:
            label = a.type_name

        self.g.add_node(str(a.h.value()), label=label, **self.node_attributes)
   
    def outputLinkArgumentEdges(self,a, outgoing=None):
        #import code; code.interact(local=locals())
        #import ipdb; ipdb.set_trace()
        assert a.is_link()
        # assumes outgoing links/nodes have already been output

        if outgoing==None:
            outgoing = a.out

        for i in xrange(0, len(outgoing)):
            outi = outgoing[i]
            id = str(a.h.value())+'->'+str(outi.h.value())
            self.g.add_edge(id, a.h.value(), outi.h.value(), directed = True,  label=str(i))

class DottyOutput:
    def __init__(self,space):
        self._as = space

    def start(self):
        print "digraph OpenCog {"

    def stop(self):
        print "}"

    def outputNodeVertex(self,a,label=None):
        assert a.is_node()
        if label==None:
            label = '%s:%s' %(a.name, a.type_name)

        out = ""
        out+=str(a.h.value())+" "
        out+='[label="'+label+'"]'
        print out

    def outputLinkEdge(self,a, label=None,outgoing=None):
        assert a.is_link()
        assert len(a.out) == 2
        assert (label==None) == (outgoing==None)

        if label==None:
            label = a.type_name

        if outgoing==None:
            outgoing = a.out

        (out0, out1) = outgoing[0].h.value(), outgoing[1].h.value()

        out = ""
        out+= str(out0) + '->' + str(out1) + ' '
        out+= '[label="' + label + '"]'
        print out

    def outputLinkVertex(self,a, label=None):
        assert a.is_link()

        if label==None:
            label = a.type_name

        output = ""
        output+= str(a.h.value()) + " "
        output+= '[label="' + label + '" shape="diamond"]'
        print output

    def outputLinkArgumentEdges(self,a, outgoing=None):
        assert a.is_link()
        # assumes outgoing links/nodes have already been output

        if outgoing==None:
            outgoing = a.out

        output = ""
        for i in xrange(0, len(outgoing)):
            outi = outgoing[i]
            output+= str(a.h.value())+"->"+str(outi.h.value())+' '
            output+= '[label="'+str(i)+'"]'
            output+= '\n'
        print output,

class SubdueTextOutput:
    def __init__(self,space):
        self._as = space
        self.i = 1
        # Remember the Subdue vertex ID for each Handle - vertex IDs must be listed in exact order,
        # but Handles are usually missing some numbers, and in a different order
        self.handle2id = {}
        self.process = True

    def start(self):
        print "XP"

    def stop(self):
        pass

    def outputNodeVertex(self,a, label=None):
        assert a.is_node()
        if label==None:
            label = '%s:%s' %(a.name, a.type_name)

        self.handle2id[a.h.value()] = self.i

        out = 'v %s "%s"' % (str(self.i), label)

        self.i+=1
        print out

    def outputLinkEdge(self,a, label=None,outgoing=None):
        assert a.is_link()
        assert len(a.out) == 2
        assert (label==None) == (outgoing==None)

        if label==None:
            label = a.type_name

        if outgoing==None:
            outgoing = a.out

        (out0, out1) = outgoing[0].h.value(), outgoing[1].h.value()
        (out0, out1) = (self.handle2id[out0], self.handle2id[out1])

        if a.is_a(t.OrderedLink):
            out = 'd %s %s "%s"' %(str(out0), str(out1), label)
        else:
            out = 'u %s %s "%s"' %(str(out0), str(out1), label)
        print out

    def outputLinkVertex(self,a, label=None):
        assert a.is_link()

        if label==None:
            label = a.type_name

        self.handle2id[a.h.value()] = self.i

        output = 'v %s "%s"' % (str(self.i), label)
        self.i+=1
        print output

    def outputLinkArgumentEdges(self,a, outgoing=None):
        assert a.is_link()
        # assumes outgoing links/nodes have already been output

        if outgoing==None:
            outgoing = a.out

        try:
            a_id = self.handle2id[a.h.value()]        

            output = ''
            for i in xrange(0, len(outgoing)):
                #outi = outgoing[i]
                outi_id = self.handle2id[outgoing[i].h.value()]

                if a.is_a(t.OrderedLink):
                    output+= 'd %s %s "%s"\n' % (str(a_id), str(outi_id), str(i))
                else:
                    output+= 'u %s %s "%s"\n' % (str(a_id), str(outi_id), str(i))
        except KeyError, e:
            print "%% Processing", str(a), "!!! Error - did not previously output the vertex for this link:", str(Atom(Handle(e.args[0]),  self._as))

        print output,

# Does the following three transformations:
# * All ObjectNodes and TimeNodes are replaced with just a label. This means that a subgraph miner will find patterns
# where ANY ObjectNode has certain links; in other words objects are distinguished based on their features rather than
# treating every object differently. This enables generaliing patterns between different objects - very important!
# * Any time a PredicateNode or ConceptNode is used, make a separate copy. Otherwise every concept will have large numbers of links
# coming out of it (increasing connectivity without making it any more expressive). Greater connectivity is BAD for graph mining runtimes.
# * Every EvaluationLink, ExecutionLink etc is replaced with a new node. e.g [in OpenCog Scheme format].
# (ExecutionLink (PredicateNode "near")
#   (ListLink
#     (ObjectNode "id_123")
#     (ObjectNode "id_456")
# ))
# is translated into:
# a vertex called "ObjectNode" (with an id of say 1)
# another vertex called "ObjectNode" (with an id of 2)
# a vertex called "near" (with an id of 3)
# a link marked "0"  which goes from the near vertex to the first object vertex
# a link marked "1"  which goes from the near vertex to the second object vertex

# TODO: FishgramFilter doesn't treat unordered links any differently (and DottyOutput couldn't handle it anyway?)
# TODO: Treat symmetric relations as unordered links. Have an option for unordered, and switch it off for dotty output
# (or make dotty ignore its effects)
# TODO: can convert Eval(near:Pred List(a b)) into near(a b), but won't make that an unordered link...
# Should have the SymmetricRelation ConceptNode.
class FishgramFilter:
    def __init__(self,space,writer):
        self._as = space
        self.writer = writer

    def start(self):
        self.writer.start()

    def stop(self):
        self.writer.stop()

    def outputNodeVertex(self,a):
        assert a.is_node()

        if self.ignore(a): return

        # TODO: keep the ID if it's a Pet/Avatar/Humanoid; include Node type?
        # Remember that there is always an InheritanceLink to (ConceptNode "PetNode"), etc.
        if a.is_a(t.ObjectNode) or a.is_a(t.TimeNode):
            self.writer.outputNodeVertex(a,label=a.type_name) #'_OBJECT_')
        else:
            self.writer.outputNodeVertex(a)

    def outputLinkEdge(self,a):
        assert a.is_link()
        assert len(a.out) == 2

        if self.ignore(a): return

        if not self.is_application_link(a):
            self.writer.outputLinkEdge(a)
        else:
            label = a.out[0].name #PredicateNode name
            outgoing = a.out[1].out # ListLink outgoing
            self.writer.outputLinkEdge(a,label,outgoing)

    def outputLinkVertex(self,a):
        assert a.is_link()

        if self.ignore(a): return

        if not self.is_application_link(a):
            self.writer.outputLinkVertex(a)
        else:
            # Send this predicate to the single-edge system if possible
            if self.is_compactableEvalLink(a):
                self.outputLinkEdge(a)
            else:
                label = a.out[0].name #PredicateNode name
                self.writer.outputLinkVertex(a,label)

    def outputLinkArgumentEdges(self,a):
        assert a.is_link()
        # assumes outgoing links/nodes have already been output

        if self.ignore(a): return

        #import ipdb; ipdb.set_trace()

        if not self.is_application_link(a):
            self.writer.outputLinkArgumentEdges(a)
        else:
            if self.is_compactableEvalLink(a):
                return #Already handled by outputLinkEdge
            else:
                label = a.out[0].name
                outgoing = a.out[1].out # ListLink outgoing -TODO assumes a ListLink even when the Predicate has 1 argument
                self.writer.outputLinkVertex(a, label)
                self.writer.outputLinkArgumentEdges(a,outgoing)

    def ignore(self,a):
        # allows WRLinks (WordReference) to make it more interesting.
        # e.g. there's a WRLink from WordNode:"soccer ball", but the most specific ConceptNode is "ball"
        #return not (a.is_a(t.Node) or a.is_a(t.InheritanceLink) or a.is_a(t.EvaluationLink))
        include = True # proably safe to allow any link now, except false EvaluationLinks
#        include = (a.is_a(t.Node) or a.is_a(t.InheritanceLink) or a.is_a(t.SimilarityLink) or
#                        self.is_application_link(a) or
#                        a.is_a(t.AtTimeLink) )
        # Can't assume that EvalLinks or similar can be converted into single Edges (they may have something else pointing to them)
        #if a.is_a(t.PredicateNode): include = False
        if a.is_a(t.LatestLink): include = False
        if self.is_application_link(a):
           # TODO hack to deal with actionDone predicates not having TVs
           # TODO Won't handle cases where an EvalLink with no TV is wrapped in something OTHER than an AtTimeLink
           times = [x for x in a.out if x.t == t.AtTimeLink]
           # Deal with AtTime(T, SimilarityLink( ExecutionOutputLink ...
           times += [x for x in times if x.t == t.AtTimeLink]
           if any(self.is_true_tv(o.tv) for o in [a]+times): include = True
           if a.out[0].name == "proximity": include = False
        
        # Allow nested ListLinks, but still replace ones that are inside an EvaluationLink or similar
        if a.is_a(t.ListLink):
            if len(a.incoming) == 0 or self.is_application_link(a.incoming[0]):
                include = False
            else:
                include = True

        return not include

    def is_true_tv(self, tv):
            return tv.mean >= 0.5 and tv.count > 0

    def simplify(self,a):
        """If it is an EvaluationLink, replace it with a simple Predicate. Otherwise return the same thing"""
        if a.is_a(t.EvaluationLink):
            return None

    @staticmethod
    def is_compactableEvalLink(a):
        # Check that the EvaluationLink can be replaced with a single edge
        # i.e. if the EvalLink has no incoming and _its ListLink_ has 2 arguments
        #label = a.out[0].name #PredicateNode name
        outgoing = a.out[1].out # ListLink outgoing
        return len(outgoing) == 2 and len(a.incoming) == 0

    @staticmethod
    def is_application_link(a):
        return a.is_a(t.EvaluationLink) or a.is_a(t.ExecutionOutputLink) or a.is_a(t.ExecutionLink)

# Hacks
class GephiMindAgent(opencog.cogserver.MindAgent):
    def __init__(self):
        self.cycles = 1

    def run(self,atomspace):
#        import pdb; pdb.set_trace()

        try:
            #import pdb; pdb.set_trace()
#            g = GraphConverter(atomspace,
#                FishgramFilter(atomspace,
#                SubdueTextOutput(atomspace)))
#            g.output()

            te = ForestExtractor(atomspace, GephiOutput(atomspace))
            te.output()
        except KeyError,  e:
            KeyError
        except Exception, e:
            import traceback; traceback.print_exc(file=sys.stdout)
        self.cycles+=1

print __name__
if __name__ == "__main__":
    a = AtomSpace()
    t=types
    bob = a.add_node(t.ConceptNode, "Bob")
    alice = a.add_node(t.ConceptNode, "Alice")
    link = a.add_link(t.ListLink, [bob, alice])
    link2 = a.add_link(t.ListLink, [alice, bob])

    link3 = a.add_link(t.EvaluationLink, [a.add_node(t.PredicateNode, "likes"), link2])

    obj1 = a.add_node(t.AccessoryNode, 'ball1')
    obj2 = a.add_node(t.StructureNode, 'tree1')
    next = a.add_link(t.EvaluationLink,
                   [a.add_node(t.PredicateNode, 'next'),
                    a.add_link(t.ListLink, [obj1, obj2])])

    next.tv = TruthValue(1, 1)

    arity3 = a.add_link(t.AndLink, [bob, alice, obj1])

    time = a.add_link(t.AtTimeLink, [a.add_node(t.TimeNode, "t-0"), a.add_node(t.ConceptNode, "blast-off")])

    eval_arity1 = a.add_link(t.EvaluationLink, [a.add_node(t.PredicateNode, "is_edible"),
                    a.add_link(t.ListLink, [a.add_node(t.ConceptNode, "bowl123")])])
    eval_arity1.tv = TruthValue(1,  1)

    #    f = FishgramFilter(a,SubdueTextOutput(a))
    #
    ##    d = DottyOutput(a)
    ##    g = GraphConverter(a,d)
    #
    ##    g = GraphConverter(a,SubdueTextOutput(a))
    #    g = GraphConverter(a, f)
    #
    #    g.output()

    te = ForestExtractor(a,  DottyOutput(a))
    te.output()
