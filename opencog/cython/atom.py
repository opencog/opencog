import opencog 

# Atom wrapper object, we should really do something similar in the
# core OpenCog API.
class Atom(object):
    def __init__(self,h,a):
        self.handle = h
        self.atomspace = a
        # cache the results after first retrieval of
        # immutable properties
        self._atom_type = None
        self._name = None
        self._outgoing = None

    @property
    def name(self):
        if self._name is None:
            self._name = self.atomspace.get_name(self.handle)
        return self._name

    @property
    def tv(self):
        return self.atomspace.get_tv(self.handle)

    @tv.setter
    def tv(self,val):
        return self.atomspace.set_tv(self.handle,val)

    @property
    def av(self):
        return self.atomspace.get_av(self.handle)

    @av.setter
    def av(self,val):
        return self.atomspace.set_av(self.handle,av_dict=val)

    @property
    def out(self):
        if self._outgoing is None:
            self._outgoing = self.atomspace.get_outgoing(self.handle)
        return self._outgoing

    @property
    def incoming(self):
        return self.atomspace.get_incoming(self.handle)

    @property
    def type(self):
        if self._atom_type is None:
            self._atom_type = self.atomspace.get_type(self.handle)
        return self._atom_type

    @property
    def t(self):
        return self.type

    def is_source(self,h):
        return self.atomspace.is_source(h,self.handle)

    def is_node(self):
        opencog.is_a(self.t,opencog.types.Node)

    def is_link(self):
        opencog.is_a(self.t,opencog.types.Link)

    def is_a(self,t):
        opencog.is_a(self.t,t)

    def long_string(self):
        return self.atomspace.get_atom_string(self.handle,terse=False)

    def __str__(self):
        return self.atomspace.get_atom_string(self.handle,terse=True)

