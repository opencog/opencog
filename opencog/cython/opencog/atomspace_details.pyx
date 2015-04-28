from libcpp cimport bool
from libcpp.vector cimport vector
from cython.operator cimport dereference as deref, preincrement as inc

from atomspace cimport *

# @todo use the guide here to separate out into a hierarchy
# http://wiki.cython.org/PackageHierarchy
    

cdef convert_handle_seq_to_python_list(vector[cHandle] handles, AtomSpace atomspace):
    cdef vector[cHandle].iterator iter
    cdef cHandle i
    result = []
    iter = handles.begin()
    while iter != handles.end():
        i = deref(iter)
        temphandle = Handle(i.value())
        result.append(Atom(temphandle, atomspace))
        inc(iter)
    return result

cdef AtomSpace_factory(cAtomSpace *to_wrap):
    cdef AtomSpace instance = AtomSpace.__new__(AtomSpace)
    instance.atomspace = to_wrap
    instance.owns_atomspace = False
    return instance

cdef class AtomSpace:
    # these are defined in atomspace.pxd:
    #cdef cAtomSpace *atomspace
    #cdef cTimeServer *timeserver
    #cdef bint owns_atomspace

    # TODO how do we do a copy constructor that shares the AtomSpaceImpl?
    def __cinit__(self):
        self.owns_atomspace = False

    # A tacky hack to pass in a pointer to an atomspace from C++-land.
    # basically, pass an int, and cast it to the C++ pointer.  This
    # works, but is not very safe, and has a certain feeling of "ick"
    # about it.  But I can't find any better way.
    def __init__(self, long addr = 0):
        if (addr == 0) :
            self.atomspace = new cAtomSpace()
            self.owns_atomspace = True
        else :
            self.atomspace = <cAtomSpace*> PyLong_AsVoidPtr(addr)
            self.owns_atomspace = False

    def __dealloc__(self):
        if self.owns_atomspace:
            del self.atomspace

    def __richcmp__(as_1, as_2, int op):
        if not isinstance(as_1, AtomSpace) or not isinstance(as_2, AtomSpace):
            return NotImplemented
        cdef AtomSpace atomspace_1 = <AtomSpace>as_1
        cdef AtomSpace atomspace_2 = <AtomSpace>as_1

        cdef cAtomSpace* c_atomspace_1 = atomspace_1.atomspace
        cdef cAtomSpace* c_atomspace_2 = atomspace_2.atomspace
        
        is_equal = True
        if c_atomspace_1 != c_atomspace_2:
            is_equal = False
        if op == 2: # ==
            return is_equal
        elif op == 3: # !=
            return not is_equal

    def add(self, Type t, name=None, out=None, TruthValue tv=None, prefixed=False):
        """ add method that determines exact method to call from type """
        if is_a(t,types.Node):
            assert out is None, "Nodes can't have outgoing sets"
            atom = self.add_node(t, name, tv, prefixed)
        else:
            assert name is None, "Links can't have names"
            atom = self.add_link(t, out, tv)
        return atom

    def add_node(self, Type t, atom_name, TruthValue tv=None, prefixed=False):
        """ Add Node to AtomSpace
        @todo support [0.5,0.5] format for TruthValue.
        @todo support type name for type.
        @returns the newly created Atom
        """
        cdef string name = atom_name.encode('UTF-8')
        cdef cHandle result
# XXX fix this prefixed thing -- the code should be clling the atomspace
# utility for this.  How does one call a utility function here ??
        prefixed = False
        if prefixed:
            # prefixed nodes ALWAYS generate a new atom using atom_name
            # as the prefix
            # Need to call the utility that does prefixing, here.
            # result = self.addPrefixedNode(atromspace, t, name)
            pass
        else:
            result = self.atomspace.addNode(t, name)

        if result == result.UNDEFINED: return None
        atom = Atom(Handle(result.value()), self);
        if tv :
            self.set_tv(atom.h, tv)
        return atom

    def add_link(self,Type t,outgoing,TruthValue tv=None):
        """ Add Link to AtomSpace
        @todo support [0.5,0.5] format for TruthValue.
        @todo support type name for type.
        @returns handle referencing the newly created Atom
        """
        # create temporary cpp vector
        cdef vector[cHandle] handle_vector
        for h in outgoing:
            if isinstance(h, Handle):
                handle_vector.push_back(deref((<Handle>h).h))
            elif isinstance(h, Atom):
                handle_vector.push_back(deref((<Handle>(h.h)).h))
        cdef cHandle result
        result = self.atomspace.addLink(t, handle_vector)
        if result == result.UNDEFINED: return None
        #return Handle(result.value());
        atom = Atom(Handle(result.value()), self);
        if tv :
            self.set_tv(atom.h, tv)
        return atom

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
        if isinstance(o, Handle):
            return self.is_valid(o)
        elif isinstance(o, Atom):
            return self.is_valid((<Atom>o).handle)

    def __len__(self):
        """ Return the number of atoms in the AtomSpace """
        return self.size()

    def __getitem__(self, key):
        """ Support self[handle] lookup to get an atom """
        if not isinstance(key, Handle):
            raise KeyError("Lookup only supported by opencog.atomspace.Handle")
        if key in self:
            return Atom(key,self)
        raise IndexError("No Atom with handle %s" % str(key))

    def __iter__(self):
        """ Support iterating across all atoms in the atomspace """
        return iter(self.get_atoms_by_type(0))

    def size(self):
        """ Return the number of atoms in the AtomSpace """
        return self.atomspace.getSize()

    def get_tv(self, Handle h):
        """ Return the TruthValue of an Atom in the AtomSpace """
        cdef tv_ptr tv
        tv = self.atomspace.getTV(deref(h.h))
        if (not tv.get() or tv.get().isNullTv()):
            pytv = TruthValue()
            pytv.cobj = new tv_ptr(tv) # make copy of smart pointer
            return pytv
        return TruthValue(tv.get().getMean(),tv.get().getCount())

    def set_tv(self, Handle h, TruthValue tv):
        """ Set the TruthValue of an Atom in the AtomSpace """
        self.atomspace.setTV(deref(h.h), deref(<tv_ptr*>(tv._tvptr())))

    def get_type(self, Handle h):
        """ Get the Type of an Atom in the AtomSpace """
        return self.atomspace.getType(deref(h.h))

    def get_name(self, Handle h):
        """ Get the Name of a Node in the AtomSpace """
        cdef string name
        name = self.atomspace.getName(deref(h.h))
        return name.c_str()[:name.size()].decode('UTF-8')

    def get_outgoing(self, Handle handle):
        """ Get the outgoing set for a Link in the AtomSpace """
        cdef vector[cHandle] handle_vector
        handle_vector = self.atomspace.getOutgoing(deref(handle.h))
        return convert_handle_seq_to_python_list(handle_vector,self)

    def xget_outgoing(self, Handle handle):
        """ Get the outgoing set for a Link in the AtomSpace """
        cdef vector[cHandle] handle_vector
        handle_vector = self.atomspace.getOutgoing(deref(handle.h))

        # This code is the same for all the x iterators but there is no
        # way in Cython to yield out of a cdef function and no way to pass a 
        # vector into a Python def function, so we have to repeat code. ARGGG!
        cdef vector[cHandle].iterator c_handle_iter
        cdef cHandle current_c_handle
        c_handle_iter = handle_vector.begin()
        while c_handle_iter != handle_vector.end():
            current_c_handle = deref(c_handle_iter)
            temp_handle = Handle(current_c_handle.value())
            yield Atom(temp_handle,self)
            inc(c_handle_iter)

    def get_incoming(self, Handle handle):
        """ Get the incoming set for an Atom in the AtomSpace """
        cdef vector[cHandle] handle_vector
        handle_vector = self.atomspace.getIncoming(deref(handle.h))
        return convert_handle_seq_to_python_list(handle_vector,self)

    def xget_incoming(self, Handle handle):
        """ Get the incoming set for an Atom in the AtomSpace """
        cdef vector[cHandle] handle_vector
        handle_vector = self.atomspace.getIncoming(deref(handle.h))

        # This code is the same for all the x iterators but there is no
        # way in Cython to yield out of a cdef function and no way to pass a 
        # vector into a Python def function, so we have to repeat code. ARGGG!
        cdef vector[cHandle].iterator c_handle_iter
        cdef cHandle current_c_handle
        c_handle_iter = handle_vector.begin()
        while c_handle_iter != handle_vector.end():
            current_c_handle = deref(c_handle_iter)
            temp_handle = Handle(current_c_handle.value())
            yield Atom(temp_handle,self)
            inc(c_handle_iter)

    def is_source(self, Handle source, Handle h):
        # This logic could probably easily be implemented client side, but best to
        # keep it all in the C++ code for now
        return self.atomspace.isSource(deref(source.h),deref(h.h))

    def get_av(self, Handle h):
        # @todo this is the slow way. quicker way is to support the
        # AttentionValue object and get all values with one atomspace call
        sti = self.atomspace.getSTI(deref(h.h))
        lti = self.atomspace.getLTI(deref(h.h))
        vlti = self.atomspace.getVLTI(deref(h.h))
        return { "sti": sti, "lti": lti, "vlti": vlti }

    def set_av(self, Handle h,sti=None,lti=None,vlti=None,av_dict=None):
        # @todo this is the slow way. quicker way is to support the
        # AttentionValue object and get all values with one atomspace call
        if av_dict:
            if "sti" in av_dict: sti = av_dict["sti"]
            if "lti" in av_dict: lti = av_dict["lti"]
            if "vlti" in av_dict: vlti = av_dict["vlti"]
        if sti: self.atomspace.setSTI(deref(h.h),sti)
        if lti: self.atomspace.setLTI(deref(h.h),lti)
        if vlti != None:
            if vlti >= 1: 
                self.atomspace.incVLTI(deref(h.h))
            if vlti < 1:
                self.atomspace.decVLTI(deref(h.h))

    def get_atom_string(self, Handle h, terse=False):
        return self.atomspace.atomAsString(deref(h.h),terse).c_str()

    # query methods
    def get_atoms_by_type(self, Type t, subtype = True):
        cdef vector[cHandle] handle_vector
        cdef bint subt = subtype
        self.atomspace.getHandlesByType(back_inserter(handle_vector),t,subt)
        return convert_handle_seq_to_python_list(handle_vector,self)

    def xget_atoms_by_type(self, Type t, subtype = True):
        cdef vector[cHandle] handle_vector
        cdef bint subt = subtype
        self.atomspace.getHandlesByType(back_inserter(handle_vector),t,subt)

        # This code is the same for all the x iterators but there is no
        # way in Cython to yield out of a cdef function and no way to pass a 
        # vector into a Python def function, so we have to repeat code. ARGGG!
        cdef vector[cHandle].iterator c_handle_iter
        cdef cHandle current_c_handle
        c_handle_iter = handle_vector.begin()
        while c_handle_iter != handle_vector.end():
            current_c_handle = deref(c_handle_iter)
            temp_handle = Handle(current_c_handle.value())
            yield Atom(temp_handle,self)
            inc(c_handle_iter)

    def get_atoms_by_name(self, Type t, name, subtype = True):
        cdef vector[cHandle] handle_vector
        cdef string cname = name.encode('UTF-8')
        cdef bint subt = subtype
        self.atomspace.getHandlesByName(back_inserter(handle_vector), cname, t, subt)
        return convert_handle_seq_to_python_list(handle_vector,self)

    def xget_atoms_by_name(self, Type t, name, subtype = True):
        cdef vector[cHandle] handle_vector
        cdef string cname = name.encode('UTF-8')
        cdef bint subt = subtype
        self.atomspace.getHandlesByName(back_inserter(handle_vector), cname, t, subt)

        # This code is the same for all the x iterators but there is no
        # way in Cython to yield out of a cdef function and no way to pass a 
        # vector into a Python def function, so we have to repeat code. ARGGG!
        cdef vector[cHandle].iterator c_handle_iter
        cdef cHandle current_c_handle
        c_handle_iter = handle_vector.begin()
        while c_handle_iter != handle_vector.end():
            current_c_handle = deref(c_handle_iter)
            temp_handle = Handle(current_c_handle.value())
            yield Atom(temp_handle,self)
            inc(c_handle_iter)

    def get_atoms_by_av(self, lower_bound, upper_bound=None):
        cdef vector[cHandle] handle_vector
        if upper_bound is not None:
            self.atomspace.getHandlesByAV(back_inserter(handle_vector), lower_bound, upper_bound)
        else:
            self.atomspace.getHandlesByAV(back_inserter(handle_vector), lower_bound)
        return convert_handle_seq_to_python_list(handle_vector, self)

    def xget_atoms_by_av(self, lower_bound, upper_bound=None):
        cdef vector[cHandle] handle_vector
        if upper_bound is not None:
            self.atomspace.getHandlesByAV(back_inserter(handle_vector), lower_bound, upper_bound)
        else:
            self.atomspace.getHandlesByAV(back_inserter(handle_vector), lower_bound)

        # This code is the same for all the x iterators but there is no
        # way in Cython to yield out of a cdef function and no way to pass a 
        # vector into a Python def function, so we have to repeat code. ARGGG!
        cdef vector[cHandle].iterator c_handle_iter
        cdef cHandle current_c_handle
        c_handle_iter = handle_vector.begin()
        while c_handle_iter != handle_vector.end():
            current_c_handle = deref(c_handle_iter)
            temp_handle = Handle(current_c_handle.value())
            yield Atom(temp_handle,self)
            inc(c_handle_iter)

    def get_atoms_in_attentional_focus(self):
        cdef vector[cHandle] handle_vector
        self.atomspace.getHandleSetInAttentionalFocus(back_inserter(handle_vector))
        return convert_handle_seq_to_python_list(handle_vector, self)

    def xget_atoms_in_attentional_focus(self):
        cdef vector[cHandle] handle_vector
        self.atomspace.getHandleSetInAttentionalFocus(back_inserter(handle_vector))

        # This code is the same for all the x iterators but there is no
        # way in Cython to yield out of a cdef function and no way to pass a 
        # vector into a Python def function, so we have to repeat code. ARGGG!
        cdef vector[cHandle].iterator c_handle_iter
        cdef cHandle current_c_handle
        c_handle_iter = handle_vector.begin()
        while c_handle_iter != handle_vector.end():
            current_c_handle = deref(c_handle_iter)
            temp_handle = Handle(current_c_handle.value())
            yield Atom(temp_handle,self)
            inc(c_handle_iter)

    def get_atoms_by_target_atom(self, Type t, Atom target_atom, subtype = True):
        cdef vector[cHandle] handle_vector
        cdef bint subt = subtype
        cdef Handle target_h = target_atom.h
        self.atomspace.getIncomingSetByType(back_inserter(handle_vector),deref(target_h.h),t,subt)
        return convert_handle_seq_to_python_list(handle_vector,self)

    def xget_atoms_by_target_atom(self, Type t, Atom target_atom, subtype = True):
        cdef vector[cHandle] handle_vector
        cdef bint subt = subtype
        cdef Handle target_h = target_atom.h
        self.atomspace.getIncomingSetByType(back_inserter(handle_vector),deref(target_h.h),t,subt)

        # This code is the same for all the x iterators but there is no
        # way in Cython to yield out of a cdef function and no way to pass a 
        # vector into a Python def function, so we have to repeat code. ARGGG!
        cdef vector[cHandle].iterator c_handle_iter
        cdef cHandle current_c_handle
        c_handle_iter = handle_vector.begin()
        while c_handle_iter != handle_vector.end():
            current_c_handle = deref(c_handle_iter)
            temp_handle = Handle(current_c_handle.value())
            yield Atom(temp_handle,self)
            inc(c_handle_iter)

    def get_predicates(self, Atom target, Type predicate_type = types.PredicateNode, subclasses=True):
        cdef vector[cHandle] handle_vector
        cdef bint want_subclasses = subclasses
        cdef Handle target_h = target.h
        handle_vector = c_get_predicates(deref(target_h.h), predicate_type, want_subclasses)
        return convert_handle_seq_to_python_list(handle_vector,self)

    def xget_predicates(self, Atom target, Type predicate_type = types.PredicateNode, subclasses=True):
        cdef vector[cHandle] handle_vector
        cdef bint want_subclasses = subclasses
        cdef Handle target_h = target.h
        handle_vector = c_get_predicates(deref(target_h.h), predicate_type, want_subclasses)
        
        # This code is the same for all the x iterators but there is no
        # way in Cython to yield out of a cdef function and no way to pass a 
        # vector into a Python def function, so we have to repeat code. ARGGG!
        cdef vector[cHandle].iterator c_handle_iter
        cdef cHandle current_c_handle
        c_handle_iter = handle_vector.begin()
        while c_handle_iter != handle_vector.end():
            current_c_handle = deref(c_handle_iter)
            temp_handle = Handle(current_c_handle.value())
            yield Atom(temp_handle,self)
            inc(c_handle_iter)

    def get_predicates_for(self, Atom target, Atom predicate):
        cdef vector[cHandle] handle_vector
        cdef Handle target_h = target.h
        cdef Handle predicate_h = predicate.h
        handle_vector = c_get_predicates_for(deref(target_h.h), deref(predicate_h.h))
        return convert_handle_seq_to_python_list(handle_vector,self)

    def xget_predicates_for(self, Atom target, Atom predicate):
        cdef vector[cHandle] handle_vector
        cdef Handle target_h = target.h
        cdef Handle predicate_h = predicate.h
        handle_vector = c_get_predicates_for(deref(target_h.h), deref(predicate_h.h))
        
        # This code is the same for all the x iterators but there is no
        # way in Cython to yield out of a cdef function and no way to pass a 
        # vector into a Python def function, so we have to repeat code. ARGGG!
        cdef vector[cHandle].iterator c_handle_iter
        cdef cHandle current_c_handle
        c_handle_iter = handle_vector.begin()
        while c_handle_iter != handle_vector.end():
            current_c_handle = deref(c_handle_iter)
            temp_handle = Handle(current_c_handle.value())
            yield Atom(temp_handle,self)
            inc(c_handle_iter)

    @classmethod
    def include_incoming(cls, atoms):
        """
        Returns the conjunction of a set of atoms and their incoming sets.

        Example:
        self.atomspace.include_incoming(self.atomspace.get_atoms_by_type(types.ConceptNode))
        """
        return list(set(atoms +
                [item for sublist in [atom.incoming for atom in atoms if len(atom.incoming) > 0] for item in sublist]))

    @classmethod
    def include_outgoing(cls, atoms):
        """
        Returns the conjunction of a set of atoms and their outgoing sets.
        Useful when used in combination with include_incoming.

        Example:
        self.atomspace.include_outgoing(
            self.atomspace.include_incoming(self.atomspace.get_atoms_by_type(types.ConceptNode)))
        """
        return list(set(atoms +
                [item for sublist in [atom.out for atom in atoms if len(atom.out) > 0] for item in sublist]))

    def print_list(self):
    #    self.atomspace.print_list()
         pass

cdef api object py_atomspace(cAtomSpace *c_atomspace) with gil:
    cdef AtomSpace atomspace = AtomSpace_factory(c_atomspace)
    return atomspace

cdef api object py_atom(UUID uuid, object atomspace):
    cdef Handle temphandle = Handle(uuid)
    cdef Atom atom = Atom(temphandle, atomspace)
    return atom

