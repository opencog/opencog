from opencog.atomspace cimport AtomSpace
from opencog.type_constructors import set_atomspace as set_type_atomspace

cdef extern from "Python.h":
    char *PyString_AsString(object)

def initialize_opencog(AtomSpace atomspace, object config = None):
    cdef char *configFileString
    if (config == None):
        configFileString = NULL
    else:
        configFileString = PyString_AsString(config)
    c_initialize_opencog(atomspace.atomspace, configFileString)
    set_type_atomspace(atomspace)

def finalize_opencog():
    c_finalize_opencog()
    set_type_atomspace(None)

def configuration_load(object config):
    cdef char *configFileString = PyString_AsString(config)
    c_configuration_load(configFileString)
