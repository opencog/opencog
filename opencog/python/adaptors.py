from opencog.atomspace import AtomSpace, types, Atom, Handle, TruthValue
import opencog.cogserver
from tree import *
from fishgram import *

# to debug within the cogserver, try these, inside the relevant function:
#import code; code.interact(local=locals())
#import ipdb; ipdb.set_trace()

t = types

class ForestExtractor:
    """Extracts a forest of trees, where each tree is a Link (and children) that are true in the AtomSpace.
    The trees may share some of the same Nodes."""
    def __init__(self, a, writer):
        self.a = a
        self.writer = writer
        
        # policy
        # Whether to create miner-friendly output, rather than human-friendly output
        self.miner_friendly = False
        # Only affects output
        self.compact_binary_links = True
        
        # state
        self.all_objects  = set()# all objects in the AtomSpace
        self.unique_trees = set()
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
        self.unique_trees_at_each_size = {}
        
        # The incoming links (or rather trees/predicates) for each object.
        # For each object, for each predsize, for each slot, the list of preds. (Indexes into self.all_trees)
        self.incoming = {}

    def extractTree(self,  atom, objects):
        if self.is_object(atom):
            objects.append(atom)
            self.i+=1
            return tree(self.i-1)
        elif atom.is_node():
            return tree(atom)
        else:
            args = [self.extractTree(x,  objects) for x in atom.out]
            return tree(atom, args)

    def extractForest(self):
        # TODO >0.5 for a fuzzy link means it's true, but probabilistic links may work differently        
        for link in [x for x in self.a.get_atoms_by_type(t.Link) if x.tv.mean > 0 and x.tv.count > 0]:
            if not self.include_tree(link): continue
            
            objects = []            
            #print self.extractTree(link, objects),  objects, self.i
            self.i = 0
            tree = self.extractTree(link, objects)
            objects = tuple(objects)
            
            #print tree,  [str(o) for o in objects]
            
            # policy - throw out trees with no objects
            if len(objects):
                self.all_trees.append(tree)
                self.all_trees_atoms.append(link)
                self.unique_trees.add(tree)
                self.bindings.append(objects)
                for obj in objects:
                    self.all_objects.add(obj)
                    
            # fishgram-specific experiments. Refactor later
            size = len(objects)
            if size not in self.tree_embeddings:
                self.tree_embeddings[size] = {}
            if tree not in self.tree_embeddings[size]:
                self.tree_embeddings[size][tree] = set()
            self.tree_embeddings[size][tree].add(objects)
            # You can output it using something like this:
            # for (tree,embeddingset_list) in te.tree_embeddings[1].items(): print len(embeddingset_list)
            # len([tree for (tree,embeddingset_list) in te.tree_embeddings[1].items() if len(embeddingset_list) > 0])
            if size not in self.unique_trees_at_each_size:
                self.unique_trees_at_each_size[size] = set()
            self.unique_trees_at_each_size[size] .add(tree)
            
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

#    def tree_to_string(self,  tree):
#
#        def helper(treenode):            
#            if isinstance(treenode ,  tuple):
#                ret = str(treenode[0])+'('
#                ret+= ' '.join([helper(x) for x in treenode[1:]])
#                ret+= ')'
#                return ret
#            elif isinstance(treenode,  int):
#                return str(treenode)
#            else:
#                return treenode.name+':'+treenode.type_name
#        
#        ret = helper(tree)
#        # haxx - because SUBDUE allocates a buffer of size 256!
#        ret = ret[:200]
#        return ret

    def output_tree(self, atom,  tree,  bindings):
        vertex_name = str(tree)
        
        # policy
        if self.compact_binary_links and len(bindings) == 2:
            self.writer.outputLinkEdge(atom,  label=vertex_name,  outgoing=bindings)
        else:
            self.writer.outputLinkVertex(atom,  label=vertex_name)
            self.writer.outputLinkArgumentEdges(atom,  outgoing=bindings)

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
        return atom.is_a(t.ObjectNode) or atom.is_a(t.TimeNode) or atom.is_a(t.ConceptNode)
        
    def object_label(self,  atom):
        return 'some_'+atom.type_name

    # NOT USED YET
    def include_node(self,  node):
        """Whether to include a given node in the results. If it is not included, all trees containing it will be ignored as well."""
        include = True
        if (a.type == t.PredicateNode and a.name == "proximity"): include = False
        
    def include_tree(self,  link):
        """Whether to make a separate tree corresponding to this link. If you don't, links in its outgoing set can
        still get their own trees."""
        # TODO check the TruthValue the same way as you would for other links.
        if link.is_a(t.SimultaneousEquivalenceLink):
            return False
        elif any([i.is_a(t.AtTimeLink) for i in link.incoming]):
            return False
        else:
            return True
  
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
        return [Atom(h,self.a) for h in handles] # When I mistakenly said a instead of self.a, no Exception was reported!
    
    def is_compactable(self,atom):
        return len(atom.out) == 2 and len(atom.incoming) == 0 and not FishgramFilter.is_application_link(atom) # TODO haxx?

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
class FishgramMindAgent(opencog.cogserver.MindAgent):
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

            # add Links between TimeNodes to indicate time sequences
            times = atomspace.get_atoms_by_type(t.TimeNode)
            times = sorted([f for f in times if f.name != "0"]) # Related to a bug in the Psi Modulator system

            for i in xrange(len(times)-1):
                (time1,  time2) = (times[i],  times[i+1])
                # TODO SeqAndLink was not supposed to be used on TimeNodes directly.
                # But that's more useful for fishgram
                print atomspace.add_link(t.SequentialAndLink,  [time1,  time2])

            te = ForestExtractor(atomspace, DottyOutput(atomspace))
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
