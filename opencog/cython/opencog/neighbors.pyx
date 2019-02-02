
from atomspace cimport *

cdef class Neighbors:
    # Deprecated. Who uses this? Anyone? Is it useful for anyone?
    def get_predicates(self,
                       Atom target,
                       Type predicate_type = types.PredicateNode,
                       subclasses=True):
        if self.atomspace == NULL:
            return None
        cdef vector[cHandle] handle_vector
        cdef bint want_subclasses = subclasses
        handle_vector = c_get_predicates(deref(target.handle), predicate_type,
                                         want_subclasses)
        return convert_handle_seq_to_python_list(handle_vector, self)

    # Deprecated. Who uses this? Anyone? Is it useful for anyone?
    def get_predicates_for(self, Atom target, Atom predicate):
        if self.atomspace == NULL:
            return None
        cdef vector[cHandle] handle_vector
        handle_vector = c_get_predicates_for(deref(target.handle),
                                             deref(predicate.handle))
        return convert_handle_seq_to_python_list(handle_vector, self)
