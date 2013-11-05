from opencog.atomspace cimport cHandle, cHandleSeq, cAtomSpace, AtomSpace

# basic wrapping for std::string conversion
cdef extern from "<string>" namespace "std":
    cdef cppclass string:
        string()
        string(char *)
        char * c_str()
        int size()

# ideally we'd import these typedefs instead of defining them here but I don't
# know how to do that with Cython
ctypedef short stim_t

cdef extern from "opencog/server/Agent.h" namespace "opencog":
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

cdef extern from "opencog/server/CogServer.h" namespace "opencog":
    cdef cppclass cCogServer "opencog::CogServer":
        cAtomSpace& getAtomSpace()

    cdef cCogServer& cogserver()

cpdef get_atomspace():
    c_server = cogserver()
    c_atomspace = c_server.getAtomSpace()

    atomspace = AtomSpace_factory(&c_atomspace)
    return atomspace


cdef class MindAgent:
    cdef cAgent *c_obj

cdef extern from "opencog/server/Request.h" namespace "opencog":
    cdef cppclass cRequest "opencog::Request":
        void send(string s)

cdef class Request:
    cdef cRequest *c_obj

