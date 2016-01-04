/*
 * opencog/server/AgentRunnerBase.cc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <algorithm>
#include <chrono>
#include <opencog/cogserver/server/AgentRunnerBase.h>
#include <opencog/util/Logger.h>

using namespace std;
using namespace chrono;


namespace opencog
{

struct equal_to_id: public std::binary_function<AgentPtr, const std::string&, bool>
{
        bool operator()(AgentPtr a, const std::string& cid) const
        {
            return (a->classinfo().id != cid);
        }
};

AgentRunnerBase::AgentRunnerBase(std::string runner_name): name(runner_name),
        cycleCount(0)
{
}

AgentRunnerBase::~AgentRunnerBase()
{
}

void AgentRunnerBase::setName(std::string new_name)
{
    name = new_name;
}

const std::string& AgentRunnerBase::getName() const
{
    return name;
}

unsigned long AgentRunnerBase::getCycleCount() const
{
    return cycleCount;
}

void AgentRunnerBase::addAgent(AgentPtr a)
{
    agents.push_back(a);
}

void AgentRunnerBase::removeAgent(AgentPtr a)
{
    AgentSeq::iterator ai = std::find(agents.begin(), agents.end(), a);
    if (ai != agents.end()) {
        agents.erase(ai);
        a->stop();
        logger().debug("[CogServer::%s] stopped agent \"%s\"", name.c_str(),
            a->to_string().c_str());
    }
}

void AgentRunnerBase::removeAllAgents(const std::string& id)
{
    // place agents with classinfo().id == id at the end of the container
    AgentSeq::iterator last =
        std::partition(agents.begin(), agents.end(),
                       boost::bind(equal_to_id(), _1, id));

    std::for_each(last, agents.end(), [] (AgentPtr &a) { a->stop(); });

    // remove those agents from the main container
    agents.erase(last, agents.end());

    logger().debug("[CogServer::%s] stopped all agents of type \"%s\"",
        name.c_str(), id.c_str());
}

void AgentRunnerBase::removeAllAgents()
{
    for (auto &a: agents)
        a->stop();
    agents.clear();
    logger().debug("[CogServer::%s] stopped all agents", name.c_str());
}

void AgentRunnerBase::runAgent(AgentPtr a)
{
    auto timer_start = system_clock::now();
    logger().debug("[CogServer::%s] begin to run mind agent: %s, [cycle = %d]",
                   name.c_str(), a->classinfo().id.c_str(), cycleCount);

    a->resetUtilizedHandleSets();
    a->run();

    auto timer_end = system_clock::now();
    auto elapsed = duration_cast<duration<float>>(timer_end - timer_start).count();
    logger().debug("[CogServer::%s] running mind agent: %s, elapsed time (sec): %f "
            "[cycle = %d]", name.c_str(), a->classinfo().id.c_str(), elapsed,
            cycleCount);
}

void SimpleRunner::processAgents()
{
    AgentSeq::const_iterator it;
    for (it = agents.begin(); it != agents.end(); ++it) {
        AgentPtr agent = *it;
        if ((cycleCount % agent->frequency()) == 0)
            runAgent(agent);
    }
    ++cycleCount;
}

} /* namespace opencog */
