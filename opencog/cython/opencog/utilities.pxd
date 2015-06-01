from opencog.atomspace cimport cAtomSpace

cdef extern from "opencog/cython/opencog/Utilities.h" namespace "opencog":
    # C++: 
    #   
    #   initialize_opencog(AtomSpace* atomSpace, const char* configFile = NULL);
    #   void finalize_opencog();
    #   void configuration_load(const char* configFile);
    #
    cdef void c_initialize_opencog "opencog::initialize_opencog" (cAtomSpace*, char*)
    cdef void c_finalize_opencog "opencog::finalize_opencog" ()
    cdef void c_configuration_load "opencog::configuration_load" (char*)
