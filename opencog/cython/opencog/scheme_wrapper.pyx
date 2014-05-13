"""
Scheme wrapper for Python

Before using the scheme_wrapper, you will need to import any atom type
definitions that you require. For a detailed example of how to use this
functionality, see:

    tests/cython/guile/test_pattern.py

Also refer to the list of .scm type definition files in opencog.conf
"""

from cython.operator cimport dereference as deref
from opencog.atomspace cimport cAtomSpace, AtomSpace, Handle, cHandle

# basic wrapping for std::string conversion
cdef extern from "<string>" namespace "std":
    cdef cppclass string:
        string()
        string(char *)
        char * c_str()
        int size()

# This needs to be explicitly initialized...
cdef extern from "opencog/query/PatternSCM.h" namespace "opencog":
    cdef cppclass PatternSCM:
        PatternSCM()

def __init__(AtomSpace a):
    # Do something, anything, to force initialization
    eval_scheme(deref(a.atomspace), "(+ 2 2)")
    PatternSCM()


cdef extern from "opencog/cython/opencog/PyScheme.h" namespace "opencog":
    string eval_scheme(cAtomSpace& as, const string& s)

def scheme_eval(AtomSpace a, char* s):
    """
    Returns a string value
    """
    cdef string ret
    cdef string expr
    expr = string(s)
    ret = eval_scheme(deref(a.atomspace), expr)
    return ret.c_str()

cdef extern from "opencog/cython/opencog/PyScheme.h" namespace "opencog":
    cHandle eval_scheme_h(cAtomSpace& as, const string& s)

def scheme_eval_h(AtomSpace a, char* s):
    """
    Returns a Handle
    """
    cdef cHandle ret
    cdef string expr
    expr = string(s)
    ret = eval_scheme_h(deref(a.atomspace), expr)
    return Handle(ret.value())

cdef extern from "opencog/guile/load-file.h" namespace "opencog":
    int load_scm_file_relative (cAtomSpace& as, char* filename)

def load_scm(AtomSpace a, char* fname):
    status = load_scm_file_relative(deref(a.atomspace), fname)
    return status == 0

