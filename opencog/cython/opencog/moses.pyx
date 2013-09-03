# distutils: language = c++
# distutils: extra_compile_args = ['--std=c++0x']
# distutils: sources = ["../../learning/moses/service/moses_service.cc"]

cdef class pymoses:
    cdef moses_service *thisptr
    def __cinit__(self):
        self.thisptr = new moses_service()
    def __dealloc__(self):
        del self.thisptr
    def run(self):
        self.thisptr.run()
