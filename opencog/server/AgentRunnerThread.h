/*
 * opencog/server/AgentRunnerThread.h
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

#ifndef OPENCOG_SERVER_AGENTRUNNERTHREAD_H_
#define OPENCOG_SERVER_AGENTRUNNERTHREAD_H_

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <opencog/server/AgentRunnerBase.h>


namespace opencog
{

/**
 *
 */
class AgentRunnerThread: public AgentRunnerBase
{
    public:
        AgentRunnerThread(const std::string &name = "agent_thread");
        /**
         * AgentRunnerThread is not movable anyway, because mutex &
         * condition_variable are not. We just make it explicit.
         */
        AgentRunnerThread(AgentRunnerThread &&tmp) = delete;
        ~AgentRunnerThread();

        /** Start runAgents agents */
        void start();

        /** Stop runAgents agents */
        void stop();

        /** Adds agent 'a' to the list of scheduled agents. */
        void addAgent(AgentPtr a);

        /** Removes agent 'a' from the list of scheduled agents. */
        void removeAgent(AgentPtr a);

        /** Removes all agents from class 'id' */
        void removeAllAgents(const std::string &id);

        void removeAllAgents();

        const AgentSeq &getAgents() const;

        bool hasAgents() const;

    private:
        std::atomic_bool runAgents;
        std::mutex runAgentsMutex;
        std::condition_variable runAgentsCond;

        std::thread runThread;
        mutable std::mutex agentsMutex;

    private:
        void processAgentsThread();
        void joinRunThread();
};

} /* namespace opencog */

#endif /* OPENCOG_SERVER_AGENTRUNNERTHREAD_H_ */
