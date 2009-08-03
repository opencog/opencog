#ifndef _OPENCOG_FACTORY_WRAP_H
#define _OPENCOG_FACTORY_WRAP_H

#include <opencog/server/Agent.h>
#include <opencog/server/Factory.h>
#include <opencog/dynamics/attention/ForgettingAgent.h>

#include <boost/python/wrapper.hpp>

using namespace boost::python;
using namespace opencog;

typedef AbstractFactory<Agent> AbstractFactory_Agent;
typedef Factory<ForgettingAgent, Agent> Factory_ForgettingAgent;

/** Exposes the Factory classes. */
void init_Factory_py();

/** A class wrapper of the Factory class.
 *
 * Dummy classes like these are necessary for wrapping the virtual functions 
 * of classes.
 */
struct AbstractFactory_AgentWrap : AbstractFactory_Agent, wrapper<AbstractFactory_Agent>
{
    // Pure virtual functions.

    Agent* create() const;
    const ClassInfo& info() const;
};

#endif // _OPENCOG_FACTORY_WRAP_H
