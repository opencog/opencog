from libcpp.unordered_set cimport unordered_set

from opencog.atomspace cimport cAtomSpace, cHandle

ctypedef public double av_type

### Cython version 0.23.4 does not contain libcpp.iterator package ###
cdef extern from "<iterator>" namespace "std" nogil:
    cdef cppclass iterator[Category,T,Distance,Pointer,Reference]:
        pass
    cdef cppclass output_iterator_tag:
        pass
    cdef cppclass back_insert_iterator[T](iterator[output_iterator_tag,void,void,void,void]):
        pass
    back_insert_iterator[CONTAINER] back_inserter[CONTAINER](CONTAINER &)


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

        # template <typename OutputIterator> OutputIterator
        # get_handle_set_in_attentional_focus(OutputIterator result)
        output_iterator get_handle_set_in_attentional_focus[output_iterator](output_iterator)

        # get by STI range
        output_iterator get_handles_by_AV[output_iterator](output_iterator, av_type sti_lower_bind, av_type sti_upper_bound)
        output_iterator get_handles_by_AV[output_iterator](output_iterator, av_type sti_lower_bind)

    cdef cAttentionBank attentionbank(cAtomSpace*)


cdef extern from "opencog/attentionbank/bank/AFImplicator.h" namespace "opencog":
    # C++:
    #   Handle af_bindlink(AtomSpace*, const Handle&);
    cHandle c_af_bindlink "af_bindlink" (cAtomSpace*, const cHandle&)
