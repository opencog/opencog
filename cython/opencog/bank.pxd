from libcpp.vector cimport vector
from libcpp.list cimport list as cpplist
from libcpp.memory cimport shared_ptr

cdef extern from "opencog/attentionbank/AVUtils.h" namespace "opencog":
    cdef av_type get_sti(const cHandle&)
    cdef av_type get_lti(const cHandle&)
    cdef av_type get_vlti(const cHandle&)

cdef extern from "opencog/attentionbank/AttentionBank.h" namespace "opencog":
    cdef cppclass cAttentionBank "opencog::AttentionBank":
        void set_sti(const cHandle&, av_type stiValue)
        void set_lti(const cHandle&, av_type ltiValue)
        void inc_vlti(const cHandle&)
        void dec_vlti(const cHandle&)

        # get by STI range
        output_iterator get_handles_by_AV(output_iterator, short lowerBound, short upperBound)
        output_iterator get_handles_by_AV(output_iterator, short lowerBound)

        # get from AttentionalFocus
        output_iterator get_handle_set_in_attentional_focus(output_iterator)

    cdef cAttentionBank attentionbank(cAtomSpace*)
