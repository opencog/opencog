from opencog.atomspace import AtomSpace, types, Atom, Handle, TruthValue
import opencog.cogserver

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
        [self.addVertex(atom) for atom in sorted(self.a.get_atoms_by_type(t.Atom))]
        [self.addEdges(atom) for atom in sorted(self.a.get_atoms_by_type(t.Link))]
        self.writer.stop()

    def is_compactable(self,atom):
        return len(atom.out) == 2 and len(atom.incoming) == 0 and not atom.is_a(t.EvaluationLink) # TODO haxx?

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

    def outputLinkEdge(self,a):
        assert a.is_link()
        assert len(a.out) == 2

        (out0, out1) = a.out[0].h.value(), a.out[1].h.value()

        out = ""
        out+= str(out0) + '->' + str(out1) + ' '
        out+= '[label="' + a.type_name + '"]'
        print out

    def outputLinkVertex(self,a):
        assert a.is_link()

        output = ""
        output+= str(a.h.value()) + " "
        output+= '[label="' + a.type_name + '" shape="diamond"]'
        print output

    def outputLinkArgumentEdges(self,a):
        assert a.is_link()
        # assumes outgoing links/nodes have already been output

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

        a_id = self.handle2id[a.h.value()]        

        output = ''
        for i in xrange(0, len(outgoing)):
            #outi = outgoing[i]
            outi_id = self.handle2id[outgoing[i].h.value()]

            if a.is_a(t.OrderedLink):
                output+= 'd %s %s "%s"\n' % (str(a_id), str(outi_id), str(i))
            else:
                output+= 'u %s %s "%s"\n' % (str(a_id), str(outi_id), str(i))

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
        self.writer.start()

    def stop(self):
        self.writer.stop()

    def outputNodeVertex(self,a):
        assert a.is_node()

        if self.ignore(a): return

        # TODO: keep the ID if it's a Pet/Avatar/Humanoid; include Node type?
        # Remember that there is always an InheritanceLink to (ConceptNode "PetNode"), etc.
        if a.is_a(t.ObjectNode):
            self.writer.outputNodeVertex(a,label=a.type_name) #'_OBJECT_')
        else:
            self.writer.outputNodeVertex(a)

    def outputLinkEdge(self,a):
        assert a.is_link()
        assert len(a.out) == 2

        if self.ignore(a): return

        if not a.is_a(t.EvaluationLink):
            self.writer.outputLinkEdge(a)
        else:
            label = a.out[0].name #PredicateNode name
            outgoing = a.out[1].out # ListLink outgoing
            self.writer.outputLinkEdge(a,label,outgoing)

    def outputLinkVertex(self,a):
        assert a.is_link()

        if self.ignore(a): return

        if not a.is_a(t.EvaluationLink):
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

        if not a.is_a(t.EvaluationLink):
            self.writer.outputLinkArgumentEdges(a)
        else:
            if self.is_compactableEvalLink(a):
                return #Already handled by outputLinkEdge
            else:
                outgoing = a.out[1].out # ListLink outgoing
                self.writer.outputLinkArgumentEdges(a,outgoing)

    def ignore(self,a):
        # allows WRLinks (WordReference) to make it more interesting.
        # e.g. there's a WRLink from WordNode:"soccer ball", but the most specific ConceptNode is "ball"
        #return not (a.is_a(t.Node) or a.is_a(t.InheritanceLink) or a.is_a(t.EvaluationLink))
        return (not (a.is_a(t.Node) or a.is_a(t.InheritanceLink) or a.is_a(t.EvaluationLink) )# or
                     #a.is_a(t.WRLink))
               or (a.is_a(t.EvaluationLink) and (a.tv.mean < 0.5 or a.tv.count == 0))
               or (a.is_a(t.EvaluationLink) and a.out[0].name == "proximity"))
#        return a.is_a(t.ListLink) or a.is_a(t.PredicateNode)

    def simplify(self,a):
        """If it is an EvaluationLink, replace it with a simple Predicate. Otherwise return the same thing"""
        if a.is_a(t.EvaluationLink):
            return None

    def is_compactableEvalLink(self,a):
        # Check that the EvaluationLink can be replaced with a single edge
        # i.e. if the EvalLink has no incoming and _its ListLink_ has 2 arguments
        #label = a.out[0].name #PredicateNode name
        outgoing = a.out[1].out # ListLink outgoing
        return len(outgoing) == 2 and len(a.incoming) == 0

# Hacks
class FishgramMindAgent(opencog.cogserver.MindAgent):
    def __init__(self):
        self.cycles = 1

    def run(self,atomspace):
        g = GraphConverter(atomspace,
            FishgramFilter(atomspace,
            SubdueTextOutput(atomspace)))
        g.output()
        self.cycles+=1

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

    f = FishgramFilter(a,SubdueTextOutput(a))

#    d = DottyOutput(a)
#    g = GraphConverter(a,d)

#    g = GraphConverter(a,SubdueTextOutput(a))
    g = GraphConverter(a, f)

    #g.output()

