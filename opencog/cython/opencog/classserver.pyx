from atomspace cimport cClassServer, classserver, NOTYPE, string, Type

# dynamically construct a "types" module
# this should also listen to "addtype" signals in case new types are
# added dynamically
cdef c_get_type_name(Type t):
    #cdef cClassServer cs
    #cs=classserver()
    cdef string s
    s = classserver().getTypeName(t)

    if str("*** Unknown Type! ***") == str(s) :
        s = ""
    return s.c_str()

cdef c_get_type(char *type_name):
    return classserver().getType(string(type_name))

# type methods
def get_type_name(t):
    return c_get_type_name(t)

def get_type(name):
    return c_get_type(name)

def is_a(Type t1, Type t2):
    return classserver().isA(t1,t2)

# From Roger's suggestion:
#import sys
#mod = sys.modules[__name__]
#
#for name in ['A', 'B', 'C']:
#    class_ = type(name, (object, ), {})
#    setattr(mod, name, class_)

cdef generate_type_module():
    types = {}
    cdef string s
    for i in range(0,classserver().getNumberOfClasses()):
        s=classserver().getTypeName(i)
        assert s.size() > 0, "Got blank type name while generating types module"
        types[s.c_str()] = i
    types["NO_TYPE"] = NOTYPE
    return types

types = type('atom_types', (), generate_type_module())

