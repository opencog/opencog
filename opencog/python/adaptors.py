try:
    from opencog.atomspace import AtomSpace, types, Atom, TruthValue
    import opencog.cogserver
except ImportError:
    from atomspace_remote import AtomSpace, types, Atom, TruthValue
from tree import *
from util import *

from pprint import pprint
from collections import defaultdict

# to debug within the cogserver, try these, inside the relevant function:
#import code; code.interact(local=locals())
#import ipdb; ipdb.set_trace()

t = types

class ForestExtractor:
    """Extracts a forest of trees, where each tree is a Link (and children) that are true in the AtomSpace.
    The trees may share some of the same Nodes. This is used as a preprocessor for Fishgram. It makes a huge
    difference to the efficiency. There are lots of filters that let you control exactly which atoms will be used."""
    def __init__(self, atomspace, writer):
        self.a = atomspace
        self.writer = writer
        
        #self.attentional_focus = True
        self.attentional_focus = False
        
        # policy
        # Whether to create miner-friendly output, rather than human-friendly output.
        # Makes it output all object-nodes with the same label. May be more useful for visualisation anyway.
        # Not relevant to the Fishgram algorithm, only to external miners (abandoned)
        self.miner_friendly = True
        # Whether to include events (i.e. anything in an AtTimeLink). Otherwise
        # it will only include static properties of objects.
        self.include_events = False
        # Only affects output
        self.compact_binary_links = True
        # Spatial relations are useful, but cause a big combinatorial explosion
        self.unwanted_atoms = set(['proximity', 'next', 'near',
            'beside', 'left_of', 'right_of', 'far', 'behind', 'in_front_of',
            'between', 'touching', 'outside', 'above', 'below', # 'inside',
            # Useless stuff. null means the object class isn't specified (something that was used in the
            # Multiverse world but not in the Unity world. Maybe it should be?
            'is_movable', 'is_noisy', 'null', 'id_null', 'Object',
            'exist', # 'decreased','increased',
            # Not useful e.g. because they contain numbers
            "AGISIM_rotation", "AGISIM_position", "AGISIM_velocity", "SpaceMap", "inside_pet_fov", 'turn', 'walk',
            'move:actor', 'is_moving', 
            # These ones make it ignore physiological feelings; it'll only care about the corresponding DemandGoals
            'pee_urgency', 'poo_urgency', 'energy', 'fitness', 'thirst',
            # These might be part of the old embodiment system or part of Psi, I'm not sure
            'happiness','sadness','fear','excitement','anger',
            'night',
            'actionFailed','decreased',
            'food_bowl', # redundant with is_edible
            #'foodState','egg','dish',
            # The can_do predicate is useful but should be in an AtTimeLink
            'can_do'])
        
        # state
        self.all_objects  = set()# all objects in the AtomSpace
        self.all_timestamps = set()
        self.all_trees = []
        self.all_trees_atoms = []
        self.bindings = []
        # variable counter
        # NOTE: If you set it to 0 here, it will give unique variables to every tree. BUT it will then count every occurrence of
        # a tree as different (because of the different variables!)
        #self.i = 0

        self.event_embeddings = defaultdict(list)
        
        # fishgram-specific experiments. Refactor later
        # map from unique tree to set of embeddings. An embedding is a set of bindings. Maybe store the corresponding link too.
        self.tree_embeddings = defaultdict(list)
        
        # The incoming links (or rather trees/predicates) for each object.
        # For each object, a mapping from rel -> every embedding involving that object
        self.incoming = defaultdict(lambda:defaultdict(list))

    class UnwantedAtomException(Exception):
        pass

    def extractTree(self,  atom, objects):
        if not self.include_atom(atom):
            raise self.UnwantedAtomException
        elif self.is_object(atom):
            objects.append(atom)
            self.i+=1
            return Var(self.i-1)
        elif self.is_action_instance(atom):
            #print 'is_action_instance', atom
            # this is moderatly tacky, but doing anything different would require lots of changes...
            return Tree('ListLink', [])
        elif atom.is_node():
            return Tree(atom)
        else:
            args = [self.extractTree(x,  objects) for x in atom.out]
            return Tree(atom.type_name, args)

    def extractForest(self):
        # TODO >0.5 for a fuzzy link means it's true, but probabilistic links may work differently        
        initial_links = [x for x in self.a.get_atoms_by_type(t.Link) if (x.tv.mean > 0.5 and x.tv.confidence > 0)]
        
        for link in initial_links:
            if link.tv is None:
                print link, 'has no TV!'
                assert False
                     #or x.type_name in ['EvaluationLink', 'InheritanceLink']]: # temporary hack
                     #or x.is_a(t.AndLink)]: # temporary hack
            if self.attentional_focus and link.av['sti'] <= -10:
                continue
            
            if not self.include_tree(link): continue
            #print link
            
            objects = []            
            #print self.extractTree(link, objects),  objects, self.i
            self.i = 0
            try:
                tree = self.extractTree(link, objects)
                #print tree
            except(self.UnwantedAtomException):
                #print 'UnwantedAtomException'
                continue
            # fishgram wants objects as trees for consistency, but
            # gephi output class wants atoms...
            objects = tuple(map(Tree,objects))
            
            #print tree,  [str(o) for o in objects]
            
            # policy - throw out trees with no objects
            if len(objects):
                self.all_trees.append(tree)
                self.all_trees_atoms.append(link)
                
                
                self.bindings.append(objects)
                
                for obj in objects:
                    if obj.get_type() != t.TimeNode:
                    #if obj.t != t.TimeNode:
                        self.all_objects.add(obj)
                    else:
                        self.all_timestamps.add(obj)
                    
                # fishgram-specific
                substitution = subst_from_binding(objects)
                if tree.op == 'AtTimeLink':
                    self.event_embeddings[tree].append(substitution)
                else:
                    self.tree_embeddings[tree].append(substitution)
                    for obj in objects:
                        # modifies self.incoming
                        tree_embeddings_for_obj = self.incoming[obj]
                        if substitution not in tree_embeddings_for_obj[tree]:
                            tree_embeddings_for_obj[tree].append(substitution)

                #size= len(objects)
                #tree_id = len(self.all_trees) - 1
                #for slot in xrange(size):
                #    obj = objects[slot]
                #    
                #    if obj not in self.incoming:
                #        self.incoming[obj] = {}
                #    if size not in self.incoming[obj]:
                #        self.incoming[obj][size] = {}
                #    if slot not in self.incoming[obj][size]:
                #        self.incoming[obj][size][slot] = []
                #    self.incoming[obj][size][slot].append(tree_id)
        
        # Make all bound trees. Enables using lookup_embeddings
        self.all_bound_trees = [subst(subst_from_binding(b), tr) for tr, b in zip(self.all_trees, self.bindings)]    
    
        pprint({tr:len(embs) for (tr, embs) in self.tree_embeddings.items()})
        
        print self.all_objects, self.all_timestamps

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

    def is_object(self, atom):
        # only useful for pathfinding visualization!
        #return atom.name.startswith('at ')
        return atom.is_a(t.StructureNode) or atom.is_a(t.BlockEntityNode) or atom.is_a(t.ObjectNode) or atom.is_a(t.SemeNode) or atom.is_a(t.TimeNode) or atom.name.startswith('at ') # or self.is_action_instance(atom)# or self.is_action_element(atom)
        
    def is_action_instance(self, atom):        
        return atom.t == t.ConceptNode and len(atom.name) and atom.name[-1].isdigit()
        
#    def is_action_element(self, atom):
#        return ':' in atom.name
    
    #def is_important_atom(self, atom):
    #    return atom.name in ['actionFailed', 'actionDone'] or "DemandGoal" in atom.name
    
    def object_label(self,  atom):
        return 'some_'+atom.type_name

    def include_atom(self,  atom):
        """Whether to include a given atom in the results. If it is not included, all trees containing it will be ignored as well."""
        if atom.is_node():
            if (atom.name in self.unwanted_atoms or atom.name.startswith('id_CHUNK') or
                atom.name.endswith('Stimulus') or atom.name.endswith('Modulator') or
                atom.is_a(t.VariableNode)):
                return False
        else:            
            if any([atom.is_a(ty) for ty in 
                    [t.SimultaneousEquivalenceLink, t.SimilarityLink, # t.ImplicationLink,
                     t.ReferenceLink,
                     t.ForAllLink, t.AverageLink, t.PredictiveImplicationLink] ]):
                return False

        return True
    
    def include_tree(self,  link):
        """Whether to make a separate tree corresponding to this link. If you don't, links in its outgoing set can
        still get their own trees."""
#        if not link.is_a(t.SequentialAndLink):
#            return False

        ## Policy: Only do objects not times
        #if link.is_a(t.AtTimeLink):
        #    return False

        if (not self.include_events) and link.is_a(t.AtTimeLink):
            return False

        # TODO check the TruthValue the same way as you would for other links.
        # work around hacks in other modules
        if any([i.is_a(t.AtTimeLink) for i in link.incoming]):
            return False
        if link.is_a(t.ExecutionLink) or link.is_a(t.ForAllLink) or link.is_a(t.AndLink):
            return False        
        if link.is_a(t.AtTimeLink) and self.is_action_instance(link.out[1]):
            return False

        return True

    # tr = fish.forest.all_trees[0]
    # fish.forest.lookup_embeddings((tr,))
    def lookup_embeddings(self, conj):
        """Given a conjunction, do a naive search for all embeddings. Fishgram usually finds the embeddings as part of the search,
        which is probably more efficient. But this is simpler and guaranteed to be correct. So it is useful for testing and performance comparison.
        It could also be used to find (the embeddings for) extra conjunctions that fishgram has skipped
        (which would be useful for the calculations used when finding implication rules)."""

        return self.lookup_embeddings_helper(conj, (), {}, self.all_bound_trees)

    def lookup_embeddings_helper(self, conj, bound_conj_so_far, s, all_bound_trees):

        if len(conj) == 0:
            return [s]

        # Find all compatible embeddings. Then commit to that tree
        tr = conj[0]

        ret = []
        substs = []
        matching_bound_trees = []

        for bound_tr in all_bound_trees:
            s2 = unify(tr, bound_tr, s)
            if s2 != None:
                #s2_notimes = { var:obj for (var,obj) in s2.items() if obj.get_type() != t.TimeNode }
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

    def outputNodeVertex(self, tr_a, label = None):
        a = atom_from_tree(tr_a,self._as)
        
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

        out0_tr = outgoing[0]
        out0 = atom_from_tree(out0_tr, self._as)
        out1_tr = outgoing[1]
        out1 = atom_from_tree(out1_tr, self._as)

        (out0, out1) = out0.h.value(), out1.h.value()
     
        self.g.add_edge(str(a.h.value()), out0, out1, directed=True, label=label)
       
    def outputLinkVertex(self, a, label=None):
        #import code; code.interact(local=locals())
        #import ipdb; ipdb.set_trace()
        assert a.is_link()
       
        if label==None:
            label = a.type_name

        self.g.add_node(str(a.h.value()), label=label, **self.node_attributes)
   
    def outputLinkArgumentEdges(self,a, outgoing=None):
        '''a is an Atom but outgoing is a list of Trees.'''
        #import code; code.interact(local=locals())
        #import ipdb; ipdb.set_trace()
        assert a.is_link()
        # assumes outgoing links/nodes have already been output

        if outgoing==None:
            outgoing = a.out

        for i in xrange(0, len(outgoing)):
            outi_tr = outgoing[i]
            outi = atom_from_tree(outi_tr, self._as)
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

# Hacks
class GephiMindAgent(opencog.cogserver.MindAgent):
    def __init__(self):
        self.cycles = 1

    def run(self,atomspace):

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
