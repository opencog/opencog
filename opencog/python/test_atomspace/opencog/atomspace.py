from collections import defaultdict

AtomType = str


class TruthValue(object):
    def __init__(self, mean=0.0, count=0.0):
        self.mean = float(mean)
        self.count = float(count)
        self.confidence = count_to_confidence(count)

    def __repr__(self):
        return '(%s %s)' % (self.mean, self.count)

    def __cmp__(self, other):
        if type(other) != type(self):
            return cmp(type(self), type(other))
        else:
            return cmp((self.mean, self.count), (other.mean, other.count))

# If you change the constant, make sure to replace it in SimpleTruthValue.cc
def confidence_to_count(conf):
    KKK = 800.0
    conf = min(conf, 0.9999999)
    return KKK * conf / (1.0 - conf)


def count_to_confidence(count):
    KKK = 800.0
    return count / (count + KKK)


DEFAULT_TV = TruthValue()
DEFAULT_AV = {'sti': 0.0, 'lti':0.0, 'vlti':0.0}


class Atom(object):
    # legacy Cython-style interface
    class Handle(object):
        def __init__(self, v):
            self.v = v

        def value(self):
            return self.v

    def __init__(self):
        self._tv = DEFAULT_TV
        self._av = DEFAULT_AV
        self._in = []

        # legacy Cython-style interface
        self.out = []
        self.name = ''
        self.h = Atom.Handle(id(self))

    def is_a(self, t):
        return is_a(self.type, t)

    #def getout(self):
    #    return []
    #out = property(getout)

    # Mutable fields, so they need to inform the AtomSpace if they change
    def gettv(self):
        return self._tv

    def settv(self,tv):
        assert not tv is None
        assert isinstance(tv,TruthValue)
        self._tv = tv

    def getav(self):
        return self._av

    def setav(self, av):
        self._av = av

    tv = property(gettv, settv)
    av = property(getav, setav)

    # Needs to be updated by the atomspace
    def getincoming(self):
        return list(self._in)

    incoming = property(getincoming)

    def __cmp__(self, other):
        if type(other) != type(self):
            return cmp(type(self), type(other))
        else:
            return cmp(id(self), id(other))

    def __hash__(self):
        return id(self)

    def __eq__(self, other):
        return self is other

    # legacy Cython-style interface
    def gettype(self):
        return self.type

    t = property(gettype)
    type_name = property(gettype)


class Node(Atom):
    def __init__(self, type, name):
        Atom.__init__(self)
        self.type = type
        self.name = name

        assert isinstance(name, str)
        assert isinstance(type, AtomType)

    def __repr__(self):
        return self.type + ' ' + self.name

    def _signature(self):
        return (self.type, self.name)

    # legacy Cython-style interface
    def is_node(self):
        return True


class Link(Atom):
    def __init__(self, type, outgoing):
        Atom.__init__(self)
        self.type = type
        self.out = tuple(outgoing)

        assert isinstance(type, AtomType)

        for o in outgoing:
            assert isinstance(o, Atom)
            if o is None:
                import pdb;

                pdb.set_trace()

    def __repr__(self):
        return '(' + self.type + ' ' + repr([repr(atom) for atom in self.out]) + ')'

    #def getout(self):
    #    pass

    def _signature(self):
        return (self.type, tuple(o._signature() for o in self.out))

    # legacy Cython-style interface
    def is_node(self):
        return False


class _AtomTypeDict(dict):
    def __getitem__(self, item):
        return item


class AtomTypeList(object):
    def __init__(self):
        self.__dict__ = _AtomTypeDict()

    def __getattr__(self, type):
        return type


types = t = AtomTypeList()
# probably parse atom_types.script files. what does the scheme version do?

def get_type_name(t):
    return t


def get_type(t):
    return t


_inheritsFrom = {}


def is_a(type1, type2):
    if type2 == t.Atom:
        return True
    elif type2 == t.Link:
        return type1.endswith('Link')
    elif type2 == t.Node:
        return type1.endswith('Node')
    else:
        # watch out - won't actually check for inheritance!
        return type1 == type2

#    else:
#        raise NotImplementedError('Atom type inheritance not implemented')

class AtomSpace(object):
    def __init__(self):
        self.atoms_by_id = set()

        self.atoms_by_signature = {}

        self.atoms_by_handle = {}

        self.type2atom = defaultdict(set)

    def __getitem__(self, key):
        return self.atoms_by_handle[key]

    # legacy Cython-style interface
    def set_av(self, h, sti=0, lti=0, vlti=None, av_dict=None):
        atom = self[h]
        atom.setav({'sti': sti, 'lti':lti})

    def get_av(self, h):
        atom = self[h]
        av = atom.getav()
        return {'sti': av.sti}

    # legacy Cython-style interface
    def add_node(self, type, name, tv=DEFAULT_TV, **kwargs):
        atom = self.maybe_add(Node(type, name))
        atom.tv = tv
        return atom

    # legacy Cython-style interface
    def add_link(self, type, out, tv=DEFAULT_TV):
        atom = self.maybe_add(Link(type, out))
        atom.tv = tv

        return atom

    # legacy Cython-style interface. Adds a node or link depending on arguments
    def add(self, type, name=None, out=None, tv=DEFAULT_TV):
        if is_a(type,types.Node):
            assert out is None, "Nodes can't have outgoing sets"
            atom = self.add_node(type, name, tv)
        else:
            assert name is None, "Links can't have names"
            atom = self.add_link(type, out, tv)
        return atom

    # legacy Cython-style interface
    def print_list(self):
        for atom in self:
            if len(atom._in) is 0:
                print atom

    def maybe_add(self, atom):
        '''If this atom hasn't been added, add it and return it. Otherwise, return the
        existing atom.'''
        assert isinstance(atom, Atom)

        try:
            return self.get(atom)
        except KeyError:
            self._add(atom)
            return atom

    def _add(self, atom):
        # Make sure it is not the base class Atom
        assert type(atom) != Atom
        # Give an error if the outgoing list is not already in the AtomSpace.
        # Especially if it contains unique Atom objects that match an existing
        # Atom (but are different unique objects).
        if isinstance(atom, Link):
            for o in atom.out:
                assert self._atom_object_is_in_atomspace(o)

                # Also add atom to the incoming set of o.
                # It may be there already, but only if 'o' is included in 'atom''s outgoing set
                # more than once. In that case, this loop will have added it already.
                if atom not in o._in:
                    o._in.append(atom)

        self.atoms_by_id.add(atom)
        self.atoms_by_signature[atom._signature()] = atom
        self.atoms_by_handle[atom.h] = atom
        self.type2atom[atom.type].add(atom)

    def remove(self, atom):
        a = self.get(atom)
        self._remove(atom)

    def _remove(self, atom):
        '''Removes an atom from all indexes. The object will still exist
        until it is garbage-collected. atom must be the unique object for this atom.'''

        self.atoms_by_id.remove(atom)
        del self.atoms_by_signature[atom._signature()]

        self.type2atom[atom.type].remove(atom)

        # Look through the outgoing Atoms of 'atom' and delete 'atom' from their incoming sets.
        if isinstance(atom, Link):
            for o in atom.out:
                assert atom in o._in
                o._in.remove(atom)

    def get_atoms_by_type(self, t, subclass=True):
        return list(self._get_atoms_by_type_generator(type, subclass))

    def _get_atoms_by_type_generator(self, type, subclass=True):
        if subclass:
            for (t, atoms) in self.type2atom.items():
                if is_a(t, type):
                    for a in atoms:
                        yield a
        else:
            for a in self.type2atom[type]:
                yield a

    def get(self, atom):
        return self.atoms_by_signature[atom._signature()]

    def _atom_object_is_in_atomspace(self, atom):
        return atom in self.atoms_by_id

    def get_atoms_in_attentional_focus(self, attention_bound=0.5):
        attentional_focus = []
        for signature in self.atoms_by_signature:
            atom = self.atoms_by_signature[signature]
            av = atom.getav()
            if 'sti' in av and av['sti'] > attention_bound:
                attentional_focus.append(atom)
        return attentional_focus

    def __iter__(self):
        return iter(self.get_atoms_by_type(t.Atom, True))


def test():
    a = AtomSpace()
    n = Node(t.ConceptNode, 'cat')
    a.add(n)
    l = Link(t.ListLink, [n])
    ret = a.add(l)
    assert l.out[0] is n

    n2 = Node(t.ConceptNode, 'cat')
    assert not n is n2

    print a.get_atoms_by_type(t.ConceptNode)
    print a.get_atoms_by_type(t.ListLink)

    a.remove(n)

    print a.get_atoms_by_type(t.ConceptNode)

    for atom in a:
        print atom


def test_fishgram():
    import sokoban
    import fishgram

    sokoban.main()
    atomspace = sokoban.ATOMSPACE

    #    for atom in atomspace:
    #        print atom

    import pdb;

    pdb.set_trace()

    fish = fishgram.Fishgram(atomspace)

    # Detect timestamps where a DemandGoal got satisfied or frustrated
    #fishgram.notice_changes(atomspace)

    fish.forest.extractForest()

    fish.run()


def test_scaling():
    a = AtomSpace()
    for i in xrange(1, 1000000):
        #a.add(Node(t.ConceptNode,str(i)))
        a.add_node(t.ConceptNode, str(i))

        #try:
        #    import opencog.atomspace as cy
        #
        #    class CythonAtomSpaceSynchronizer(object):
        #        def cython2python(self,pyas,cyas):
        #            for cyatom in cyas.get_atoms_by_type(cy.types.Atom):
        #                pytype = cyatom.type_name
        #                pyname = cyatom.name
        #                # Except really it will be handles
        #                pyout = cyatom.out
        #
        #        def python2cython(self,pyas, cyas):
        #
        #
        #except ImportError:
        #    pass