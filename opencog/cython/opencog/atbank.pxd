from libcpp.unordered_set cimport unordered_set

from opencog.atomspace cimport cAtomSpace, cHandle

ctypedef public double av_type

cdef extern from "opencog/attentionbank/bank/AVUtils.h" namespace "opencog":
    cdef av_type get_sti(const cHandle&)
    cdef av_type get_lti(const cHandle&)
    cdef av_type get_vlti(const cHandle&)

cdef extern from "opencog/attentionbank/bank/AttentionBank.h" namespace "opencog":
    cdef cppclass cAttentionBank "opencog::AttentionBank":
        void set_sti(const cHandle&, av_type stiValue)
        void set_lti(const cHandle&, av_type ltiValue)
        void inc_vlti(const cHandle&)
        void dec_vlti(const cHandle&)

        # get by STI range
        unordered_set[cHandle] c_get_handles_by_av "getHandlesByAV" (av_type sti_lower_bind, av_type sti_upper_bound)

    cdef cAttentionBank attentionbank(cAtomSpace*)


cdef extern from "opencog/attentionbank/bank/AFImplicator.h" namespace "opencog":
    # C++:
    #   Handle af_bindlink(AtomSpace*, const Handle&);
    cHandle c_af_bindlink "af_bindlink" (cAtomSpace*, const cHandle&)
