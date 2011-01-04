#include "SpaceServer_wrap.h"
#include <opencog/atomspace/SpaceServer.h>
#include <opencog/atomspace/AtomSpaceAsync.h>
#include <opencog/persist/file/SavableRepository.h>

#include <boost/python/class.hpp>

using namespace opencog;
using namespace boost::python;

void init_SpaceServer_py()
{
    class_<SpaceServer, bases<SavableRepository>, boost::noncopyable>("SpaceServer", no_init)
        .def(init<AtomSpaceAsync&>())
        //.def(init<SpaceServerContainer&>())
    ;
}
