from libcpp cimport bool
from libcpp.vector cimport vector
from cython.operator cimport dereference as deref, preincrement as inc

from atomspace cimport *
# @todo use the guide here to separate out into a hierarchy
# http://wiki.cython.org/PackageHierarchy

cdef class Handle:
    def __cinit__(self, h):
        self.h=new cHandle(h)
    def __dealloc__(self):
        del self.h
    def value(self):
        return self.h.value()
    def __richcmp__(Handle h1, Handle h2, int op):
        if op == 2: # ==
            return deref(h1.h) == deref(h2.h)
        elif op == 3: # !=
            return deref(h1.h) != deref(h2.h)
        elif op == 4: # >
            return deref(h1.h) > deref(h2.h)
        elif op == 0: # <
            return deref(h1.h) < deref(h2.h)
        elif op == 1: # <=
            return deref(h1.h) <= deref(h2.h)
        elif op == 5: # >=
            return deref(h1.h) >= deref(h2.h)
    def __str__(self):
        return "<UUID:" + str(self.h.value()) + ">"
    def __repr__(self):
        return "Handle(" + str(self.h.value()) + ")"
    def is_undefined(self):
        if deref(self.h) == self.h.UNDEFINED: return True
        return False
    def __nonzero__(self):
        """ Allows boolean comparison, return false is handle == UNDEFINED """
        return not self.is_undefined()
    

cdef class TruthValue:
    """ The truth value represents the strength and confidence of
        a relationship or term. In OpenCog there are a number of TruthValue
        types, but as these involve additional complexity we focus primarily on
        the SimpleTruthValue type which allows strength and count

        @todo Support IndefiniteTruthValue, DistributionalTV, NullTV etc
    """
    # This stores a pointer to a smart pointer to the C++ TruthValue object
    # This indirection is unfortunately necessary because cython doesn't
    # allow C++ objects on the stack
    cdef tv_ptr *cobj

    def __cinit__(self, strength=0.0, count=0.0):
        # By default create a SimpleTruthValue
        self.cobj = new tv_ptr(new cSimpleTruthValue(strength,count))

    def __dealloc__(self):
        # This deletes the *smart pointer*, not the actual pointer
        del self.cobj

    property mean:
        def __get__(self): return self._mean()

    property confidence:
        def __get__(self): return self._confidence()

    property count:
        def __get__(self): return self._count()

    cdef _mean(self):
        return self._ptr().getMean()

    cdef _confidence(self):
        return self._ptr().getConfidence()

    cdef _count(self):
        return self._ptr().getCount()

    def __richcmp__(TruthValue h1, TruthValue h2, int op):
        " @todo support the rest of the comparison operators"
        if op == 2: # ==
            return deref(h1._ptr()) == deref(h2._ptr())
        
        raise ValueError, "TruthValue does not yet support most comparison operators"

    cdef cTruthValue* _ptr(self):
        return self.cobj.get()

    cdef tv_ptr* _tvptr(self):
        return self.cobj

    def __str__(self):
        return self._ptr().toString().c_str()

# If you change the constant, make sure to replace it in SimpleTruthValue.cc
def confidence_to_count(conf):
    KKK = 800.0
    conf = min(conf, 0.9999999)
    return KKK * conf / (1.0 - conf)

def count_to_confidence(count):
    KKK = 800.0
    return count / (count + KKK)

cdef class TimeServer:
    cdef cTimeServer *timeserver

    def __cinit__(self):
        #self.timeserver = &timeserver
        pass

    def __dealloc__(self):
        # Don't do anything because the AtomSpace takes care of cleaning up
        pass

# @todo this should be a generator using the yield statement
cdef convert_handle_seq_to_python_list(vector[cHandle] handles, AtomSpace atomspace):
    cdef vector[cHandle].iterator iter
    cdef cHandle i
    result = []
    iter = handles.begin()
    while iter != handles.end():
        i = deref(iter)
        temphandle = Handle(i.value())
        result.append(Atom(temphandle,atomspace))
        inc(iter)
    return result

cdef AtomSpace_factory(cAtomSpace *to_wrap):
    cdef AtomSpace instance = AtomSpace.__new__(AtomSpace)
    instance.atomspace = to_wrap
    return instance

cdef class AtomSpace:
    # these are defined in atomspace.pxd:
    #cdef cAtomSpace *atomspace
    #cdef cTimeServer *timeserver
    #cdef bint owns_atomspace

    # TODO how do we do a copy constructor that shares the AtomSpaceAsync?
    def __cinit__(self):
        self.owns_atomspace = False

    def __init__(self):
        self.atomspace = new cAtomSpace()
        self.owns_atomspace = True

    def __dealloc__(self):
        if self.owns_atomspace:
            del self.atomspace

    def add(self, Type t, name=None, out=None, TruthValue tv=None, prefixed=False):
        """ add method that determines exact method to call from type """
        if is_a(t,types.Node):
            assert out is None, "Nodes can't have outgoing sets"
            atom = self.add_node(t,name,tv,prefixed)
        else:
            assert name is None, "Links can't have names"
            atom = self.add_link(t,out,tv)
        return atom

    def add_node(self, Type t, atom_name, TruthValue tv=None, prefixed=False):
        """ Add Node to AtomSpace
        @todo support [0.5,0.5] format for TruthValue.
        @todo support type name for type.
        @returns handle referencing the newly created Atom
        """
        # convert to string
        py_byte_string = atom_name.encode('UTF-8')
        # create temporary cpp string
        cdef string *name = new string(py_byte_string)
        cdef cHandle result
        if prefixed:
            # prefixed nodes ALWAYS generate a new atom using atom_name
            # as the prefix
            if tv is None:
                # get handle
                result = self.atomspace.addPrefixedNode(t,deref(name))
            else:
                result = self.atomspace.addPrefixedNode(t,deref(name), deref(<tv_ptr*>(tv._tvptr())))
        else:
            if tv is None:
                # get handle
                result = self.atomspace.addNode(t,deref(name))
            else:
                result = self.atomspace.addNode(t,deref(name), deref(<tv_ptr*>(tv._tvptr())))
        # delete temporary string
        del name
        if result == result.UNDEFINED: return None
        return Atom(Handle(result.value()), self);

    def add_link(self,Type t,outgoing,TruthValue tv=None):
        """ Add Link to AtomSpace
        @todo support [0.5,0.5] format for TruthValue.
        @todo support type name for type.
        @returns handle referencing the newly created Atom
        """
        # create temporary cpp vector
        cdef vector[cHandle] o_vect
        for h in outgoing:
            if isinstance(h,Handle):
                o_vect.push_back(deref((<Handle>h).h))
            elif isinstance(h,Atom):
                o_vect.push_back(deref((<Handle>(h.h)).h))
        cdef cHandle result
        if tv is None:
            # get handle
            result = self.atomspace.addLink(t, o_vect)
        else:
            result = self.atomspace.addLink(t, o_vect, deref(<tv_ptr*>(tv._tvptr())))
        if result == result.UNDEFINED: return None
        #return Handle(result.value());
        return Atom(Handle(result.value()), self);

    def is_valid(self,h):
        """ Check whether the passed handle refers to an actual handle
        """
        try:
            assert isinstance(h,Handle)
        except AssertionError:
            # Try to convert to a Handle object
            try:
                uuid = int(h)
                h = Handle(uuid)
            except ValueError, TypeError:
                raise TypeError("Need UUID or Handle object")
        if self.atomspace.isValidHandle(deref((<Handle>h).h)):
            return True
        return False

    def remove(self,atom,recursive=True):
        """ Removes an atom from the atomspace
        atom --  The Atom of the atom to be removed.
        recursive -- Recursive-removal flag; if set, then all links
            that contain this atom will be removed. If not set, the
            incoming set of this atom must be empty, as otherwise
            the atom cannot be removed.
         
        Returns True if the Atom for the given Handle was successfully
        removed. False, otherwise.

        """
        cdef bint recurse = recursive
        return self.atomspace.removeAtom(deref((<Handle>(atom.h)).h),recurse)

    def clear(self):
        """ Remove all atoms from the AtomSpace """
        self.atomspace.clear()

    # Methods to make the atomspace act more like a standard Python container
    def __contains__(self,o):
        """ Custom checker to see if object is in AtomSpace """
        if isinstance(o,Handle):
            return self.is_valid(o)
        elif isinstance(o,Atom):
            return self.is_valid((<Atom>o).handle)

    def __len__(self):
        """ Return the number of atoms in the AtomSpace """
        return self.size()

    def __getitem__(self,key):
        """ Support self[handle] lookup to get an atom """
        if not isinstance(key,Handle):
            raise KeyError("Lookup only supported by opencog.atomspace.Handle")
        if key in self:
            return Atom(key,self)
        raise IndexError("No Atom with handle %s" % str(key))

    def size(self):
        """ Return the number of atoms in the AtomSpace """
        return self.atomspace.getSize()

    def get_tv(self,Handle h):
        """ Return the TruthValue of an Atom in the AtomSpace """
        cdef tv_ptr tv
        tv = self.atomspace.getTV(deref(h.h))
        if (not tv.get() or tv.get().isNullTv()):
            pytv = TruthValue()
            pytv.cobj = new tv_ptr(tv) # make copy of smart pointer
            return pytv
        return TruthValue(tv.get().getMean(),tv.get().getCount())

    def set_tv(self,Handle h, TruthValue tv):
        """ Set the TruthValue of an Atom in the AtomSpace """
        self.atomspace.setTV(deref(h.h), deref(<tv_ptr*>(tv._tvptr())))

    def get_type(self,Handle h):
        """ Get the Type of an Atom in the AtomSpace """
        return self.atomspace.getType(deref(h.h))

    def get_name(self,Handle h):
        """ Get the Name of a Node in the AtomSpace """
        cdef string name
        name = self.atomspace.getName(deref(h.h))
        return name.c_str()[:name.size()].decode('UTF-8')

    def get_outgoing(self,Handle handle):
        """ Get the outgoing set for a Link in the AtomSpace """
        cdef vector[cHandle] o_vect
        o_vect = self.atomspace.getOutgoing(deref(handle.h))
        return convert_handle_seq_to_python_list(o_vect,self)

    def get_incoming(self,Handle handle):
        """ Get the incoming set for an Atom in the AtomSpace """
        cdef vector[cHandle] o_vect
        o_vect = self.atomspace.getIncoming(deref(handle.h))
        return convert_handle_seq_to_python_list(o_vect,self)

    def is_source(self, Handle source, Handle h):
        # This logic could probably easily be implemented client side, but best to
        # keep it all in the C++ code for now
        return self.atomspace.isSource(deref(source.h),deref(h.h))

    def get_av(self,Handle h):
        # @todo this is the slow way. quicker way is to support the
        # AttentionValue object and get all values with one atomspace call
        sti = self.atomspace.getSTI(deref(h.h))
        lti = self.atomspace.getLTI(deref(h.h))
        vlti = self.atomspace.getVLTI(deref(h.h))
        return { "sti": sti, "lti": lti, "vlti": vlti }

    def set_av(self,Handle h,sti=None,lti=None,vlti=None,av_dict=None):
        # @todo this is the slow way. quicker way is to support the
        # AttentionValue object and get all values with one atomspace call
        if av_dict:
            if "sti" in av_dict: sti = av_dict["sti"]
            if "lti" in av_dict: lti = av_dict["lti"]
            if "vlti" in av_dict: vlti = av_dict["vlti"]
        if sti: self.atomspace.setSTI(deref(h.h),sti)
        if lti: self.atomspace.setLTI(deref(h.h),lti)
        if vlti: self.atomspace.incVLTI(deref(h.h))

    def get_atom_string(self, Handle h, terse=False):
        return self.atomspace.atomAsString(deref(h.h),terse).c_str()

    # query methods
    # @todo it would be better if we got AtomSpaceAsync request objects directly
    # to avoid excessive copying
    def get_atoms_by_type(self, Type t, subtype = True):
        cdef vector[cHandle] o_vect
        cdef bint subt = subtype
        self.atomspace.getHandleSet(back_inserter(o_vect),t,subt)
        return convert_handle_seq_to_python_list(o_vect,self)

    def get_atoms_by_name(self, Type t, name, subtype = True):
        cdef vector[cHandle] o_vect
        # create temporary cpp string
        py_byte_string = name.encode('UTF-8')
        cdef string *cname = new string(py_byte_string)
        cdef bint subt = subtype
        self.atomspace.getHandleSet(back_inserter(o_vect),t,deref(cname),subt)
        del cname
        return convert_handle_seq_to_python_list(o_vect,self)

    def get_atoms_by_target_type(self, Type t, Type target_t, subtype = True, target_subtype = True):
        cdef vector[cHandle] o_vect
        cdef bint subt = subtype
        cdef bint target_subt = target_subtype
        self.atomspace.getHandleSet(back_inserter(o_vect),t,target_t,subt,target_subt)
        return convert_handle_seq_to_python_list(o_vect,self)

    def get_atoms_by_target_atom(self, Type t, Atom target_atom, subtype = True):
        cdef vector[cHandle] o_vect
        cdef bint subt = subtype
        cdef Handle target_h = target_atom.h
        self.atomspace.getHandleSet(back_inserter(o_vect),deref(target_h.h),t,subt)
        return convert_handle_seq_to_python_list(o_vect,self)

    def print_list(self):
        self.atomspace.print_list()

    def next_new_atom(self):
        '''Function to get atoms that have been added to the AtomSpace.
        There's a list of newly added ones, and this method will pop the
        first atom from the list. You should use it to update a separate
        list for each Python MindAgent that wants to use it. It returns
        either a Cython Atom or None (if there are none left).'''
        cdef cHandle ch
        cdef Handle h
        cdef Atom atom
        if deref(self.atomspace).addAtomSignalQueue.size() > 0:
            ch = deref(self.atomspace).addAtomSignalQueue.front()
            deref(self.atomspace).addAtomSignalQueue.pop_front()
            h = Handle(ch.value())
            atom = Atom(h,self)
            return atom
        else:
            return None

cdef class SpaceServer:
    cdef cSpaceServer *spaceserver

    def __init__(self):
        #self.spaceserver = &spaceserver
        pass

    def __dealloc__(self):
        # Don't do anything because the CogServer takes care of cleaning up
        pass


