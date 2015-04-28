cdef extern from "moses/moses/main/moses_exec.h" namespace "opencog::moses":
    int moses_exec(int argc, char **argv) except +

