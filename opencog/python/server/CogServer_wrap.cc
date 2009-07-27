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
        .def("registerAgent",
            &CogServer::registerAgent,
            &CogServerWrap::default_registerAgent)
    ;
}

// Non-pure virtual functions.

void CogServerWrap::runAgent(Agent *agent)
{
    if (override o = this->get_override("runAgent"))
        return runAgent(agent);

    return CogServer::runAgent(agent);
}
void CogServerWrap::default_runAgent(Agent *agent)
{
    return this->CogServer::runAgent(agent);
}

bool CogServerWrap::registerAgent(const std::string& id, 
AbstractFactory<Agent> const* factory)
{
    if (override o = this->get_override("registerAgent"))
        return registerAgent(id, factory);

    return CogServer::registerAgent(id, factory);
}
bool CogServerWrap::default_registerAgent(const std::string& id, 
AbstractFactory<Agent> const* factory)
{
    return this->CogServer::registerAgent(id, factory);
}
