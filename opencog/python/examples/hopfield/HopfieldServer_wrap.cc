#include <boost/python/class.hpp>
#include "HopfieldServer_wrap.h"
#include <examples/hopfield/HopfieldServer.h>

using namespace boost::python;
using namespace opencog;

void init_HopfieldServer_py()
{
    class_<HopfieldServer>("HopfieldServer")
    ;
}
