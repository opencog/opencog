#include "AtomTable_wrap.h"
#include <opencog/atomspace/AtomTable.h>
#include <boost/python/class.hpp>

using namespace opencog;
using namespace boost::python;

void init_AtomTable_py()
{
    class_<AtomTable, boost::noncopyable>("AtomTable", no_init)
        .def(init<optional<bool> >())
        .def("getSize", &AtomTable::getSize)
    ;
}
