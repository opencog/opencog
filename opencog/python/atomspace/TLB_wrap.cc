#include "TLB_wrap.h"
#include "TLB.h"
#include <boost/python/class.hpp>

using namespace opencog;
using namespace boost::python;

void init_TLB_py()
{
    class_<TLB>("TLB", no_init)
    ;
}
