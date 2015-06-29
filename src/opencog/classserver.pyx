from atomspace cimport cClassServer, classserver, NOTYPE, string, Type
from classserver cimport strcmp


# Dynamically construct a "types" module.
# XXX FIXME This should also listen to "addtype" signals in case
# new types are added dynamically (which they are, when new cogserver
# modules are loaded.)

# Given a numeric type, look up the string name.
cdef c_get_type_name(Type t):
    # cdef cClassServer cs
    # cs = classserver()
    cdef string s
    s = classserver().getTypeName(t)

    # Attempt to do a string compare. Either of the two commented-out
    # lines below work for me, but break to build bot.  So I dunno.
    # if s.__eq__(string("*** Unknown Type! ***")) :
    # if str("*** Unknown Type! ***") == str(s) :
    if 0 == strcmp(s.c_str(), "*** Unknown Type! ***") :
        s = string("")
    return s.c_str()

# Given the string name, look up the numeric type.
cdef c_get_type(char *type_name):
    return classserver().getType(string(type_name))

# Atom type methods.
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
    for i in range(0, classserver().getNumberOfClasses()):
        s = classserver().getTypeName(i)
        assert s.size() > 0, "Got blank type name while generating types module"
        types[s.c_str()] = i
    types["NO_TYPE"] = NOTYPE
    return types
types = type('atom_types', (), generate_type_module())

#This function is for refreshing new types
#ex. When you import a cython module which include non-core atom types in C++ 
#And you can refresh these new types by this function

def get_refreshed_types():    
    return type('atom_types', (), generate_type_module())
