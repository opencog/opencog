#include "Handle_wrap.h"
#include <boost/python/class.hpp>

using namespace opencog;
using namespace boost::python;

void init_Handle_py()
{
    class_<Handle>("Handle", no_init)
    ;
}
