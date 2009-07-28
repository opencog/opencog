#include "CogServer_wrap.h"
#include <opencog/server/BaseServer.h>
#include <opencog/server/CogServer.h>

#include <boost/python/class.hpp>
#include <boost/python/return_value_policy.hpp>
#include <boost/python/copy_const_reference.hpp>
#include <boost/python/manage_new_object.hpp>

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
        .def("unregisterAgent",
            &CogServer::unregisterAgent,
            &CogServerWrap::default_unregisterAgent)
        .def("registerRequest",
            &CogServer::registerRequest,
            &CogServerWrap::default_registerRequest)
        .def("unregisterRequest",
            &CogServer::unregisterRequest,
            &CogServerWrap::default_unregisterRequest)
        .def("requestIds",
            &CogServer::requestIds,
            &CogServerWrap::default_requestIds)
        .def("createRequest",
            &CogServer::createRequest,
            &CogServerWrap::default_createRequest,
            return_value_policy<manage_new_object>())
        .def("requestInfo",
            &CogServer::requestInfo,
            &CogServerWrap::default_requestInfo,
            return_value_policy<copy_const_reference>())
        .def("getRequestQueueSize",
            &CogServer::getRequestQueueSize,
            &CogServerWrap::default_getRequestQueueSize)
        .def("processRequests", &CogServer::processRequests)
    ;
}

// Non-pure virtual functions.

void CogServerWrap::runAgent(Agent *agent)
{
    if (override o = this->get_override("runAgent"))
        o(agent);

    CogServer::runAgent(agent);
}
void CogServerWrap::default_runAgent(Agent *agent)
{
    this->CogServer::runAgent(agent);
}

bool CogServerWrap::registerAgent(const std::string& id, 
AbstractFactory<Agent> const* factory)
{
    if (override o = this->get_override("registerAgent"))
        return o(id, factory);

    return CogServer::registerAgent(id, factory);
}
bool CogServerWrap::default_registerAgent(const std::string& id, 
AbstractFactory<Agent> const* factory)
{
    return this->CogServer::registerAgent(id, factory);
}

bool CogServerWrap::unregisterAgent(const std::string& id)
{
    if (override o = this->get_override("unregisterAgent"))
        return o(id);

    return CogServer::unregisterAgent(id);
}
bool CogServerWrap::default_unregisterAgent(const std::string& id)
{
    return this->CogServer::unregisterAgent(id);
}

bool CogServerWrap::registerRequest(const std::string& id, 
AbstractFactory<Request> const* factory)
{
    if (override o = this->get_override("registerRequest"))
        return o(id, factory);

    return CogServer::registerRequest(id, factory);
}
bool CogServerWrap::default_registerRequest(const std::string& id, 
AbstractFactory<Request> const* factory)
{
    return this->CogServer::registerRequest(id, factory);
}

bool CogServerWrap::unregisterRequest(const std::string& id)
{
    if (override o = this->get_override("unregisterRequest"))
        return o(id);

    return CogServer::unregisterRequest(id);
}
bool CogServerWrap::default_unregisterRequest(const std::string& id)
{
    return this->CogServer::unregisterRequest(id);
}

std::list<const char*> CogServerWrap::requestIds(void) const
{
    if (override o = this->get_override("requestIds"))
        return o();

    return CogServer::requestIds();
}
std::list<const char*> CogServerWrap::default_requestIds(void) const
{
    return this->CogServer::requestIds();
}

Request* CogServerWrap::createRequest(const std::string& id)
{
    if (override o = this->get_override("createRequest"))
        return o(id);

    return CogServer::createRequest(id);
}
Request* CogServerWrap::default_createRequest(const std::string& id)
{
    return this->CogServer::createRequest(id);
}

const RequestClassInfo& CogServerWrap::requestInfo(const std::string& id) const
{
    if (override o = this->get_override("requestInfo"))
        return o(id);

    return CogServer::requestInfo(id);
}
const RequestClassInfo& CogServerWrap::default_requestInfo(const std::string& id) const
{
    return this->CogServer::requestInfo(id);
}

int CogServerWrap::getRequestQueueSize(void)
{
    if (override o = this->get_override("getRequestQueueSize"))
        return o();

    return CogServer::getRequestQueueSize();
}
int CogServerWrap::default_getRequestQueueSize(void)
{
    return this->CogServer::getRequestQueueSize();
}
