#ifndef _OPENCOG_AGENT_WRAP_H
#define _OPENCOG_AGENT_WRAP_H

#include <boost/python/wrapper.hpp>

#include <opencog/server/Agent.h>
#include <opencog/server/CogServer.h>

using namespace boost::python;
using namespace opencog;

/** Exposes the Agent class. */
void init_Agent_py();

/** A class wrapper of the Agent class.
 *
 * Dummy classes like these are necessary for wrapping the virtual functions 
 * of classes.
 */
struct AgentWrap : Agent, wrapper<Agent>
{
    AgentWrap();
    AgentWrap(const unsigned int);

    // Pure virtual functions.

    void run(CogServer* server);
    const ClassInfo& classinfo() const;

    // Non-pure virtual functions.

    /*void runAgent(Agent *agent);
    void default_runAgent(Agent *agent);*/
};

#endif // _OPENCOG_AGENT_WRAP_H
