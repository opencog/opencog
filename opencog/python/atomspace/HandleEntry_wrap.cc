#include "HandleEntry_wrap.h"
#include <opencog/atomspace/HandleEntry.h>
#include <boost/python/class.hpp>

using namespace opencog;
using namespace boost::python;

void init_HandleEntry_py()
{
    class_<HandleEntry>("HandleEntry", no_init)
        .def(init<Handle>())
        .def(init<Handle, HandleEntry*>())
    ;
}
