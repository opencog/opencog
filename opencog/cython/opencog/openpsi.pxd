
from opencog.atomspace cimport cHandle, tv_ptr
from libcpp.vector cimport vector

cdef extern from "opencog/openpsi/OpenPsiSCM.h" namespace "opencog":
    cdef cppclass cOpenPsi "opencog::OpenPsiSCM":
        cOpenPsi() except +

        # C++:
        #
        #  Handle add_rule(const HandleSeq& context, const Handle& action,
        #    const Handle& goal, const TruthValuePtr stv, const Handle& category);
        cHandle c_add_rule "add_rule" (vector[cHandle] context, cHandle action, cHandle goal, tv_ptr stv, cHandle category)
