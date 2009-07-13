#include "CogServer_wrap.h"
#include <opencog/server/BaseServer.h>

#include <boost/python/class.hpp>

//using namespace opencog;
using namespace boost::python;

void init_CogServer_py()
{
    class_<CogServerWrap, bases<BaseServer>, boost::noncopyable>("CogServer")
        .def("runAgent",
            &CogServer::runAgent,
            &CogServerWrap::default_runAgent)
    ;
}

void CogServerWrap::runAgent(Agent *agent)
{
    if (override o = this->get_override("runAgent"))
        return runAgent(agent);
    else
        return CogServer::runAgent(agent);
}
void CogServerWrap::default_runAgent(Agent *agent)
{
    return this->CogServer::runAgent(agent);
}
