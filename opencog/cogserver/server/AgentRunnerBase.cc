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
#include <opencog/cogserver/server/CogServer.h>
#include <opencog/util/Logger.h>
#include <opencog/util/platform.h>

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
        cycle_count(1)
{
}

AgentRunnerBase::~AgentRunnerBase()
{
}

void AgentRunnerBase::set_name(std::string new_name)
{
    name = new_name;
}

const std::string& AgentRunnerBase::get_name() const
{
    return name;
}

unsigned long AgentRunnerBase::get_cycle_count() const
{
    return cycle_count;
}

void AgentRunnerBase::add_agent(AgentPtr a)
{
    agents.push_back(a);
}

void AgentRunnerBase::remove_agent(AgentPtr a)
{
    AgentSeq::iterator ai = std::find(agents.begin(), agents.end(), a);
    if (ai != agents.end()) {
        agents.erase(ai);
        cogserver().systemActivityTable().clearActivity(a);
        a->stop();
        logger().debug("[CogServer::%s] stopped agent \"%s\"", name.c_str(),
            a->to_string().c_str());
    }
}

void AgentRunnerBase::remove_all_agents(const std::string& id)
{
    // place agents with classinfo().id == id at the end of the container
    AgentSeq::iterator last =
        std::partition(agents.begin(), agents.end(),
                       std::bind(equal_to_id(), std::placeholders::_1, id));

    std::for_each(last, agents.end(), [] (AgentPtr &a) {
        cogserver().systemActivityTable().clearActivity(a);
        a->stop();
    });

    // remove those agents from the main container
    agents.erase(last, agents.end());

    logger().debug("[CogServer::%s] stopped all agents of type \"%s\"",
        name.c_str(), id.c_str());
}

void AgentRunnerBase::remove_all_agents()
{
    for (auto &a : agents) {
        cogserver().systemActivityTable().clearActivity(a);
        a->stop();
    }
    agents.clear();
    logger().debug("[CogServer::%s] stopped all agents", name.c_str());
}

void AgentRunnerBase::run_agent(AgentPtr a)
{
    size_t mem_start = getMemUsage();
    int atoms_start = atomspace().get_size();

    logger().debug("[CogServer::%s] begin to run mind agent: %s, [cycle = %d]",
                   name.c_str(), a->classinfo().id.c_str(), cycle_count);

    auto timer_start = system_clock::now();

    a->resetUtilizedHandleSets();
    a->run();

    auto timer_end = system_clock::now();

    size_t mem_end = getMemUsage();
    int atoms_end = atomspace().get_size();

    size_t mem_used;
    if (mem_start > mem_end)
        mem_used = 0;
    else
        mem_used = mem_end - mem_start;

    int atoms_used;
    if (atoms_start > atoms_end)
        atoms_used = 0;
    else
        atoms_used = atoms_end - atoms_start;

    auto elapsed = timer_end - timer_start;
    auto elapsed_secs = duration_cast<duration<float>>(elapsed).count();
    logger().debug("[CogServer::%s] running mind agent: %s, elapsed time (sec): "
            "%f, memory used: %d, atom used: %d [cycle = %d]", name.c_str(),
            a->classinfo().id.c_str(), elapsed_secs, mem_used, atoms_used,
            cycle_count);

    cogserver().systemActivityTable().logActivity(a, elapsed, mem_used,
        atoms_used);
}

void SimpleRunner::process_agents()
{
    AgentSeq::const_iterator it;
    for (it = agents.begin(); it != agents.end(); ++it) {
        AgentPtr agent = *it;
        if ((cycle_count % agent->frequency()) == 0)
            run_agent(agent);
    }
    ++cycle_count;
}

} /* namespace opencog */
