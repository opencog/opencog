
# Basic wrapping for std::string conversion
cdef extern from "<string>" namespace "std":
    cdef cppclass string:
        string()
        string(char *)
        char * c_str()
        int size()

# Handle

cdef extern from "opencog/atoms/base/Handle.h" namespace "opencog":
    cdef cppclass cHandle "opencog::Handle":
        cHandle()
        bint operator==(cHandle h)
        bint operator!=(cHandle h)
        bint operator<(cHandle h)
        bint operator>(cHandle h)
        bint operator<=(cHandle h)
        bint operator>=(cHandle h)
        cHandle UNDEFINED
# HandleSeq
    cdef cppclass cHandleSeq "opencog::HandleSeq"

# AtomSpaces
from opencog.atomspace cimport cAtomSpace
cdef extern from "opencog/cogserver/server/BaseServer.h" namespace "opencog":
    cAtomSpace* server_atomspace()

# ideally we'd import these typedefs instead of defining them here but I don't
# know how to do that with Cython
ctypedef short stim_t

cdef extern from "opencog/cogserver/server/Agent.h" namespace "opencog":
    cdef cppclass cAgent "opencog::Agent":
        int frequency()
        void setFrequency(int f)
        string to_string()
        # HandleSeqSeq getUtilizedHandleSets
        void resetUtilizedHandleSets()

        stim_t stimulateAtom(cHandle h, stim_t amount)
        stim_t stimulateAtom(cHandleSeq hs, stim_t amount)
        void removeAtomStimulus(cHandle h)
        stim_t resetStimulus()
        stim_t getTotalStimulus()
        stim_t getAtomStimulus(cHandle h)

cdef class MindAgent:
    cdef cAgent *c_obj

cdef extern from "opencog/cogserver/server/Request.h" namespace "opencog":
    cdef cppclass cRequest "opencog::Request":
        void send(string s)

cdef class Request:
    cdef cRequest *c_obj
