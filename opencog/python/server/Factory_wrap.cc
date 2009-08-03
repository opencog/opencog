#include "Factory_wrap.h"
#include <boost/noncopyable.hpp>
#include <boost/python/class.hpp>
#include <boost/python/pure_virtual.hpp>
#include <boost/python/return_value_policy.hpp>
#include <boost/python/manage_new_object.hpp>
#include <boost/python/copy_const_reference.hpp>
#include <opencog/server/Agent.h>

using namespace boost::python;
using namespace opencog;

void init_Factory_py()
{
    class_<AbstractFactory_AgentWrap, boost::noncopyable>("AbstractFactory_Agent", no_init)
        .def("create", pure_virtual(&AbstractFactory_Agent::create),
            return_value_policy<manage_new_object>())
        .def("info", pure_virtual(&AbstractFactory_Agent::info),
            return_value_policy<copy_const_reference>())
    ;

    class_<Factory_ForgettingAgent, bases<AbstractFactory_Agent> >("Factory_ForgettingAgent", 
    no_init)
        .def(init<>())
    ;
}

// For the pure virtual member functions..

Agent* AbstractFactory_AgentWrap::create() const
{
    return this->get_override("create")();
}
const ClassInfo& AbstractFactory_AgentWrap::info() const
{
    return this->get_override("info")();
}
