from opencog.atomspace import get_refreshed_types
from opencog.utilities import add_node, add_link

cdef extern :
    void nlp_oc_types_init()

nlp_oc_types_init()
types = get_refreshed_types() 

include "opencog/nlp/oc-types/nlp_oc_types.pyx"
