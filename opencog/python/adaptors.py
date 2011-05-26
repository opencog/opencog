from opencog.atomspace import AtomSpace, types, Atom, Handle, TruthValue
import opencog.cogserver

# to debug within the cogserver, try these, inside the relevant function:
#import code; code.interact(local=locals())
#import ipdb; ipdb.set_trace()

t = types

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
            output+= 'label="'+str(i)+'"]'
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
        times = self._as.get_atoms_by_type(t.TimeNode)
        times = sorted([f for f in times if f.name != "0"]) # Related to a bug in the Psi Modulator system
        
        # Have links to represent which TimeNodes happen (shortly) after another.
        # So the graph miner will hopefully find common sequences of events
        for i in xrange(len(times)-1):
            (time1,  time2) = (times[i],  times[i+1])
            # TODO SeqAndLink was not supposed to be used on TimeNodes directly.
            # But that's more useful for fishgram
            print self._as.add_link(t.SequentialAndLink,  [time1,  time2])
        
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
            import pdb; pdb.set_trace()
            g = GraphConverter(atomspace,
                FishgramFilter(atomspace,
                SubdueTextOutput(atomspace)))
            g.output()
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

    f = FishgramFilter(a,SubdueTextOutput(a))

#    d = DottyOutput(a)
#    g = GraphConverter(a,d)

#    g = GraphConverter(a,SubdueTextOutput(a))
    g = GraphConverter(a, f)

    g.output()

