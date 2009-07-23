#include "Agent_wrap.h"

#include <boost/python/class.hpp>
#include <boost/python/pure_virtual.hpp>
#include <boost/python/return_internal_reference.hpp>

using namespace opencog;
using namespace boost::python;

void init_Agent_py()
{
    class_<AgentWrap, bases<AttentionValue>, boost::noncopyable>("Agent",
        no_init)
        .def(init<optional<const unsigned int> >())
        .def("run", pure_virtual(&Agent::run))
        .def("classinfo", pure_virtual(&Agent::classinfo),
            return_internal_reference<>())
        .def("to_string", &Agent::to_string)
    ;
}

AgentWrap::AgentWrap():
    Agent()
{}
AgentWrap::AgentWrap(const unsigned int f):
    Agent(f)
{}

// For the pure virtual functions..

void AgentWrap::run(CogServer* server)
{
    this->get_override("run")(ptr(server));
}
const ClassInfo& AgentWrap::classinfo() const
{
    return this->get_override("classinfo")();
}
