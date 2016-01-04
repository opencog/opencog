/*
 * opencog/server/AgentRunnerThread.cc
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

#include <opencog/cogserver/server/AgentRunnerThread.h>
#include <opencog/util/Logger.h>

using namespace std;


namespace opencog
{

AgentRunnerThread::AgentRunnerThread(const std::string &name) :
        AgentRunnerBase(name), runAgents(false), agentsModified(false),
        clearAll(false)
{
}

AgentRunnerThread::~AgentRunnerThread()
{
    removeAllAgents();
    joinRunThread();
}

void AgentRunnerThread::start()
{
    {
        // locking is required, to prevent dead-lock in condition_variable wait()
        lock_guard<mutex> lock(runAgentsMutex);
        runAgents = true;
    }
    runAgentsCond.notify_one();
}

void AgentRunnerThread::stop()
{
    runAgents = false;
}

void AgentRunnerThread::addAgent(AgentPtr a)
{
    if (!hasAgents()) {
        joinRunThread();
        /*
         * processAgentsThread() will exit immediately if no agents are available,
         * so we add the first one immediately
         */
        {
            lock_guard<mutex> lock(agentsMutex);
            AgentRunnerBase::addAgent(a);
        }
        runThread = thread(&AgentRunnerThread::processAgentsThread, this);
        // todo: set thread name when using pthreads
    }
    else {
        lock_guard<mutex> lock(agentsMutex);
        agentsAddQ.push_back(a);
        agentsModified = true;
    }
}

void AgentRunnerThread::removeAgent(AgentPtr a)
{
    lock_guard<mutex> lock(agentsMutex);
    agentsRemoveQ.push_back(a);
    agentsModified = true;
}

void AgentRunnerThread::removeAllAgents(const std::string& id)
{
    lock_guard<mutex> lock(agentsMutex);
    idsRemoveQ.push_back(id);
    agentsModified = true;
}

void AgentRunnerThread::removeAllAgents()
{
    lock_guard<mutex> lock(agentsMutex);
    clearAll = true;
    agentsModified = true;
}

const AgentSeq& AgentRunnerThread::getAgents() const
{
    lock_guard<mutex> lock(agentsMutex);
    return agents;
}

bool AgentRunnerThread::hasAgents() const
{
    lock_guard<mutex> lock(agentsMutex);
    return !agents.empty();
}

void AgentRunnerThread::processAgentsThread()
{
    /*
     * 'agents' is only modified in this function, so there is no need to
     * protect read accesses to this container in it.
     */
    logger().debug("[CogServer::%s] Agent thread started", name.c_str());
    while (!agents.empty()) {
        if (!runAgents) {
            unique_lock<mutex> lock(runAgentsMutex);
            runAgentsCond.wait(lock, [this] { return runAgents.load(); });
        }


        AgentSeq::const_iterator it;
        for (it = agents.begin(); it != agents.end(); ++it) {
            AgentPtr agent = *it;
            // todo: can frequency() be still useful?
//            if ((cycleCount % agent->frequency()) == 0)
            runAgent(agent);
        }

        if (agentsModified) {
            logger().debug("[CogServer::%s] Updating active agents",
                name.c_str());
            lock_guard<mutex> agentsLock(agentsMutex);
            agentsModified = false;
            if (clearAll) {
                clearAll = false;
                AgentRunnerBase::removeAllAgents();
            } else {
                for (const auto &a : agentsRemoveQ)
                    AgentRunnerBase::removeAgent(a);
                for (const auto &id : idsRemoveQ)
                    AgentRunnerBase::removeAllAgents(id);
            }
            agentsRemoveQ.clear();
            idsRemoveQ.clear();
            for (const auto &a : agentsAddQ)
                AgentRunnerBase::addAgent(a);
            agentsAddQ.clear();
        }
        ++cycleCount;
    }
    logger().debug("[CogServer::%s] Agent thread stopped", name.c_str());
}

inline void AgentRunnerThread::joinRunThread()
{
    if (runThread.joinable()) {
        start(); // unlock processAgentsThread() if stopped
        runThread.join();
    }
}

} /* namespace opencog */
