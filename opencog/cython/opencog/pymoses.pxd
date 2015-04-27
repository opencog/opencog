cdef extern from "moses/moses/main/moses_exec.h" namespace "moses3::moses":
    int moses_exec(int argc, char **argv) except +

