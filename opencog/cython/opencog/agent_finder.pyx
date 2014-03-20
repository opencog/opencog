# Write code for finding and initialising agents in cython because it'll
# probably be simpler
from libcpp cimport bool
from libcpp.vector cimport vector
from libcpp.list cimport list as cpplist
from cython.operator cimport dereference as deref, preincrement as inc

# basic wrapping for std::string conversion
cdef extern from "<string>" namespace "std":
    cdef cppclass string:
        string()
        string(char *)
        char * c_str()
        int size()

#cdef extern from "<list>" namespace "std":
    #cdef cppclass cpplist[T]:
        #list()
        #pop_front()

# we need to store module names with modules objects, and which modules 
# agents are from
module_agents = {}

import inspect
def find_subclasses(module, clazz):
    return [ cls for cls in inspect.getmembers(module) \
            if inspect.isclass(cls[1]) and issubclass(cls[1], clazz) ]    

import imp
import traceback
import opencog.cogserver
from opencog.atomspace cimport cAtomSpace, AtomSpace_factory, AtomSpace
from opencog.cogserver cimport cAgent, stim_t, cRequest

cdef extern from "agent_finder_types.h" namespace "opencog":
    cdef struct requests_and_agents_t:
        vector[string] agents
        vector[string] requests
        vector[string] req_summary
        vector[string] req_description
        vector[bool] req_is_shell
        string err_string 

cdef api string get_path_as_string() with gil:
    import sys
    cdef bytes c_str = str(sys.path)
    return string(c_str)

cdef api requests_and_agents_t load_req_agent_module(string& module_name) with gil:
    """ Load module and return a vector of MindAgent names """
    cdef bytes c_str = module_name.c_str()
    # for return results
    cdef requests_and_agents_t results
    try:
        filep, pathname, desc = imp.find_module(c_str)
    except Exception, e:
        s = "Exception while searching for module " + c_str + "\n"
        s = traceback.format_exc(10)
        results.err_string = string(s)
        return results
    agent_classes = []
    request_classes = []
    try:
        the_module = imp.load_module(c_str, filep, pathname, desc)
        # now we need to scan the module for subclasses of MindAgent
        # each entry is a tuple with ("ClassName", <classobject>)
        agent_classes = find_subclasses(the_module,opencog.cogserver.MindAgent)
        request_classes = find_subclasses(the_module,opencog.cogserver.Request)
    except Exception, e:
        s = "Exception while loading module " + c_str + "\n"
        s += traceback.format_exc(10)
        results.err_string = string(s)
    finally:
        # Since we may exit via an exception, close fp explicitly.
        if filep: filep.close()
    # convert names into strings
    for a in agent_classes:
        results.agents.push_back(string(a[0]))
    for r in request_classes:
        results.requests.push_back(string(r[0]))

        # Every request should have a summary and a description of what it does.
        summy = ""
        try :
            summy = r[1].summary
        except Exception, e:
            summy = "Command summary is missing in the python code!"
        results.req_summary.push_back(string(summy))

        desc = ""
        try :
            desc = r[1].description
        except Exception, e:
            desc = "Command description is missing in the python code!"
        results.req_description.push_back(string(desc))

        is_shell = False
        try :
            is_shell = r[1].is_shell
        except Exception, e:
            is_shell = False
        results.req_is_shell.push_back(is_shell)

    return results

from opencog.cogserver cimport MindAgent
# Initialise an agent of class "agent_name"
cdef api object instantiate_agent(string module_name, string agent_name, cAgent *cpp_agent) with gil:
    cdef bytes module_str = module_name.c_str()
    cdef bytes agent_str = agent_name.c_str()
    cdef requests_and_agents_t results
    cdef MindAgent agent = None
    try:
        # find and load the module
        filep,pathname,desc = imp.find_module(module_str)
        the_module = imp.load_module(module_str,filep,pathname,desc)
        # get the class object
        agentClass = getattr(the_module, agent_str) 
        # instantiate it
        if agentClass:
            agent = agentClass()
            agent.c_obj = cpp_agent
    except Exception, e:
        print traceback.format_exc(10)
    finally:
        # Since we may exit via an exception, close fp explicitly.
        if filep: filep.close()
    return agent

cdef api string run_agent(object o,cAtomSpace *c_atomspace) with gil:
    cdef string result
    a = AtomSpace_factory(c_atomspace)
    try:
        o.run(a)
    except Exception, e:
        s = traceback.format_exc(10)
        result = string(s)
    return result

from opencog.cogserver cimport Request
# Initialise a request of class "r_name"
cdef api object instantiate_request(string module_name, string r_name, cRequest *cpp_request) with gil:
    cdef bytes module_str = module_name.c_str()
    cdef bytes request_str = r_name.c_str()
    cdef requests_and_agents_t results
    cdef Request request = None
    try:
        # find and load the module
        filep, pathname, desc = imp.find_module(module_str)
        the_module = imp.load_module(module_str, filep, pathname, desc)
        # get the class object
        rClass = getattr(the_module, request_str) 
        # instantiate it
        if rClass:
            request = rClass()
            request.c_obj = cpp_request
    except Exception, e:
        print traceback.format_exc(10)
    finally:
        # Since we may exit via an exception, close fp explicitly.
        if filep: filep.close()
    return request

cdef api string run_request(object o, cpplist[string] args, cAtomSpace *c_atomspace) with gil:
    cdef string result
    cdef bytes tostring 
    cdef cpplist[string].iterator the_iter

    # wrap atomspace
    a = AtomSpace_factory(c_atomspace)

    # convert args to python list of strings
    args_as_python_str_list = []
    the_iter = args.begin()
    while the_iter != args.end():
        tostring = deref(the_iter).c_str()
        args_as_python_str_list.append(tostring)
        inc(the_iter)
    try:
        o.run(args=args_as_python_str_list,atomspace=a)
    except Exception, e:
        s = traceback.format_exc(10)
        result = string(s)
    return result
    
