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
import opencog.cogserver

cdef extern from "agent_finder_types.h" namespace "opencog":
    cdef struct requests_and_agents_t:
        vector[string] agents
        vector[string] requests

#import signal
#import sys
#def exit_catcher(sig,stack):
    #if sig == signal.SIGINT:
        #print("Caught interrupt signal")
    #sys.exit(sig)
#
#signal.signal(signal.SIGINT, exit_catcher)

cdef api requests_and_agents_t load_module(string& module_name) with gil:
    """ Load module and return a vector of MindAgent names """
    cdef bytes c_str = module_name.c_str()
    cdef requests_and_agents_t results
    try:
        filep,pathname,desc = imp.find_module(c_str)
    except:
        return results
    agent_classes = []
    try:
        the_module = imp.load_module(c_str,filep,pathname,desc)
        # now we need to scan the module for subclasses of MindAgent
        # each entry is a tuple with ("ClassName", <classobject>)
        agent_classes = find_subclasses(the_module,opencog.cogserver.MindAgent)
    finally:
        # Since we may exit via an exception, close fp explicitly.
        if filep: filep.close()
    # convert names into strings
    for a in agent_classes:
        results.agents.push_back(string(a[0]))
    return results

# Initialise an agent of class "agent_name"
cdef api object init_agent(string& agent_name, string& module_name) with gil:
    print "load agent" + agent_name.c_str()

    
