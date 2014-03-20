from cython.operator cimport dereference as deref
from opencog.atomspace cimport cAtomSpace, AtomSpace

# basic wrapping for std::string conversion
cdef extern from "<string>" namespace "std":
    cdef cppclass string:
        string()
        string(char *)
        char * c_str()
        int size()

cdef extern from "opencog/cython/PyScheme.h" namespace "opencog":
    string eval_scheme(cAtomSpace& as, const string& s)

cdef extern from "opencog/guile/load-file.h" namespace "opencog":
    int load_scm_file (cAtomSpace& as, char* filename)

def scheme_eval(AtomSpace a, char* s):
    cdef string ret
    cdef string expr
    expr = string(s)
    ret = eval_scheme(deref(a.atomspace), expr)
    return ret.c_str()

def load_scm(AtomSpace a, char* fname):
    status = load_scm_file(deref(a.atomspace), fname)
    return status == 0
