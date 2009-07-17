#include "ForgettingAgent_wrap.h"
//#include <opencog/dynamics/attention/ForgettingAgent.h>
#include "ForgettingAgent.h"
#include <boost/python/class.hpp>

using namespace opencog;
using namespace boost::python;

void init_ForgettingAgent_py()
{
    class_<ForgettingAgent>("ForgettingAgent")
    ;
}
