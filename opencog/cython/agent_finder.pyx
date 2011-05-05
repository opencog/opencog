# Write code for finding and initialising agents in cython because it'll
# probably be simpler
from libcpp cimport bool
from libcpp.vector cimport vector
from cython.operator cimport dereference as deref, preincrement as inc

# basic wrapping for std::string conversion
cdef extern from "<string>" namespace "std":
    cdef cppclass string:
        string()
        string(char *)
        char * c_str()
        int size()

# We need to define an api for Cython modules though
cdef public api:
    # Load module and return a list of MindAgent names
    vector[string] load_module(string& module_name)
    # Initialise an agent of class "agent_name"
    object init_agent(string& agent_name, string& module_name)

# we need to store module names with modules objects, and which modules 
# agents are from
module_agents = {}

import imp
from opencog import agent
cdef vector[string] load_module(string& module_name):
    cdef bytes c_str = module_name.c_str()
    filep,pathname,desc = imp.find_module(c_str)
    try:
        the_module = imp.load_module(c_str,filep,pathname,desc)
    finally:
        # Since we may exit via an exception, close fp explicitly.
        if filep: filep.close()
    # now we need to scan the module for subclasses of MindAgent
    

    
