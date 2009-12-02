#include "Request_wrap.h"

#include <boost/python/class.hpp>
#include <boost/python/pure_virtual.hpp>
#include <boost/python/return_value_policy.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <boost/python/manage_new_object.hpp>

using namespace opencog;
using namespace boost::python;

void init_Request_py()
{
    class_<RequestWrap, boost::noncopyable>("Request", no_init)
        .def("execute", pure_virtual(&Request::execute))
        .def("send",
            &Request::send,
            &RequestWrap::default_send)
        .def("setSocket",
            &Request::setSocket,
            &RequestWrap::default_setSocket)
        .def("getSocket",
            &Request::getSocket,
            &RequestWrap::default_getSocket,
            return_value_policy<manage_new_object>())
        .def("setParameters",
            &Request::setParameters,
            &RequestWrap::default_setParameters)
        .def("addParameter",
            &Request::addParameter,
            &RequestWrap::default_addParameter)
    ;
}

// For the pure virtual functions..

bool RequestWrap::execute(void)
{
    return this->get_override("execute")();
}

// For the non-pure virtual functions..

void RequestWrap::send(const std::string& msg) const
{
    if (override o = this->get_override("send"))
        o(msg);

    Request::send(msg);
}
void RequestWrap::default_send(const std::string& msg) const
{
    this->Request::send(msg);
}

void RequestWrap::setSocket(ConsoleSocket *h)
{
    if (override o = this->get_override("setSocket"))
        o(h);

    Request::setSocket(h);
}
void RequestWrap::default_setSocket(ConsoleSocket *h)
{
    this->Request::setSocket(h);
}

ConsoleSocket *RequestWrap::getSocket(void)
{
    if (override o = this->get_override("getSocket"))
        return o();

    return Request::getSocket();
}
ConsoleSocket *RequestWrap::default_getSocket()
{
    return this->Request::getSocket();
}

void RequestWrap::setParameters(const std::list<std::string>& params)
{
    if (override o = this->get_override("setParameters"))
        o(params);

    Request::setParameters(params);
}
void RequestWrap::default_setParameters(const std::list<std::string>& params)
{
    this->Request::setParameters(params);
}

void RequestWrap::addParameter(const std::string& param)
{
    if (override o = this->get_override("addParameter"))
        o(param);

    Request::addParameter(param);
}
void RequestWrap::default_addParameter(const std::string& param)
{
    this->Request::addParameter(param);
}
