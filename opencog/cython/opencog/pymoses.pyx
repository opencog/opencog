__author__ = 'cosmo'

'''
Wrapper for MOSES. Uses the C++ moses_exec function to access MOSES functionality.

To use:
    import pymoses
    moses = pymoses.moses()

To view the available arguments:
    moses.run("")

Example usage:
    moses.run("--version")
    moses.run("-i input.txt -c 1 -o output.txt")
'''

from libc.stdlib cimport malloc, free
import shlex

cdef class moses:
    cdef moses_service *thisptr

    def __cinit__(self):
        self.thisptr = new moses_service()

    def __dealloc__(self):
        del self.thisptr

    def run(self, args):
        self.run_args_list(shlex.split(args))

    def run_args_list(self, args_list):
        args_list.insert(0, "moses")
        cdef char **c_argv
        args_list = [bytes(x) for x in args_list]
        c_argv = <char**>malloc(sizeof(char*) * len(args_list))
        for idx, s in enumerate(args_list):
            c_argv[idx] = s
        try:
            self.thisptr.run(len(args_list), c_argv)
        except RuntimeError, ex:
            print "Exception occurred when calling MOSES:\n" + ex.message
        finally:
            free(c_argv)