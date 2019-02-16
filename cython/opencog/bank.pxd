from libcpp.vector cimport vector
from libcpp.list cimport list as cpplist
from libcpp.memory cimport shared_ptr
from libcpp.string cimport string

ctypedef public short av_type

# This import doesn't work for some reason.
# from opencog.atomspace cimport cHandle

# --------------------------------------------------------
cdef extern from "opencog/atoms/proto/ProtoAtom.h" namespace "opencog":
    cdef cppclass cProtoAtom "opencog::ProtoAtom":
        string to_string()
    
    ctypedef shared_ptr[cProtoAtom] cProtoAtomPtr "opencog::ProtoAtomPtr"

cdef class ProtoAtom:
    cdef cProtoAtomPtr shared_ptr

cdef ProtoAtom createProtoAtom(cProtoAtomPtr shared_ptr)

cdef extern from "opencog/atoms/base/Atom.h" namespace "opencog":
    cdef cppclass cAtom "opencog::Atom" (cProtoAtom):
        cAtom()

cdef extern from "opencog/atoms/base/Handle.h" namespace "opencog":
    ctypedef shared_ptr[cAtom] cAtomPtr "opencog::AtomPtr"

    cdef cppclass cHandle "opencog::Handle" (cAtomPtr):
        cHandle()
        cHandle(const cHandle&)
        string to_string()
        cHandle UNDEFINED

cdef extern from "opencog/atomspace/AtomSpace.h" namespace "opencog":
    cdef cppclass cAtomSpace "opencog::AtomSpace":
        AtomSpace()
# --------------------------------------------------------

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
