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
    def outgoing(self):
        return self.atomspace.get_outgoing(self.handle)

    @property
    def type(self):
        if self._atom_type is None:
            self._atom_type = self.atomspace.get_type(self.handle)
        return self._atom_type

    def is_source(self,h):
        pass

    def long_string(self):
        return self.atomspace.get_atom_string(self.handle,terse=False)

    def __str__(self):
        return self.atomspace.get_atom_string(self.handle,terse=True)

