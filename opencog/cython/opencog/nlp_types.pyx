from opencog.atomspace import get_refreshed_types
from opencog.utilities import add_node, add_link

cdef extern :
    void nlp_types_init()

nlp_types_init()
types = get_refreshed_types() 

include "opencog/nlp/types/nlp_types.pyx"
