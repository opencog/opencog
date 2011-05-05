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

# we need to store module names with modules objects, and which modules 
# agents are from
module_agents = {}

import inspect
def find_subclasses(module, clazz):
    return [ cls for cls in inspect.getmembers(module) \
            if inspect.isclass(cls[1]) and issubclass(cls[1], clazz) ]    

import imp
import opencog

cdef public api vector[string] load_module(string& module_name) with gil:
    """ Load module and return a vector of MindAgent names """
    cdef bytes c_str = module_name.c_str()
    cdef vector[string] results
    filep,pathname,desc = imp.find_module(c_str)
    agent_classes = []
    try:
        the_module = imp.load_module(c_str,filep,pathname,desc)
        # now we need to scan the module for subclasses of MindAgent
        agent_classes = find_subclasses(the_module,opencog.MindAgent)
    finally:
        # Since we may exit via an exception, close fp explicitly.
        if filep: filep.close()
    # convert names into strings
    for a in agent_classes:
        results.push_back(string(a.__name__))
    return results

# Initialise an agent of class "agent_name"
cdef public api object init_agent(string& agent_name, string& module_name) with gil:
    print "load agent" + agent_name.c_str()

    
