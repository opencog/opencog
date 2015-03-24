from opencog.atomspace cimport AtomSpace

cdef extern from "Python.h":
    char *PyString_AsString(object)

def initialize_opencog(AtomSpace atomspace, object config = None):
    cdef char *configFileString
    if (config == None):
        configFileString = NULL
    else:
        configFileString = PyString_AsString(config)
    c_initialize_opencog(atomspace.atomspace, configFileString)

def finalize_opencog():
    c_finalize_opencog()

def configuration_load(object config):
    cdef char *configFileString = PyString_AsString(config)
    c_configuration_load(configFileString)
