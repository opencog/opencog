
from opencog.atomspace cimport cHandle
ctypedef short Type

cdef extern from "GetPredicates.h" namespace "opencog":
    # C++:
    #
    #   HandleSeq get_predicates(const Handle& target,
    #                     Type predicateType=PREDICATE_NODE,
    #                     bool subClasses=true)
    #   void finalize_opencog();
    #   void configuration_load(const char* configFile);
    #
    cdef vector[cHandle] c_get_predicates "get_predicates" (cHandle& target, Type t, bint subclass)
    cdef vector[cHandle] c_get_predicates_for "get_predicates_for" (cHandle& target, cHandle& predicate)
