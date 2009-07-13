#include "BaseServer_wrap.h"
#include "BaseServer.h"
#include <boost/python/class.hpp>

using namespace opencog;
using namespace boost::python;

void init_BaseServer_py()
{
    class_<BaseServer>("BaseServer")
    ;
}
