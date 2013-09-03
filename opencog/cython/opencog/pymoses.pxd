cdef extern from "../../learning/moses/service/moses_service.h" namespace "opencog::moses":
    cdef cppclass moses_service:
        moses_service() except +
        void run()

