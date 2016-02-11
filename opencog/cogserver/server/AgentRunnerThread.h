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
#include <vector>
#include <opencog/cogserver/server/AgentRunnerBase.h>


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

        /** Start running agents */
        void start();

        /** Stop running agents */
        void stop();

        /** Adds agent 'a' to the list of scheduled agents. */
        void add_agent(AgentPtr a);

        /** Removes agent 'a' from the list of scheduled agents. */
        void remove_agent(AgentPtr a);

        /** Removes all agents from class 'id' */
        void remove_all_agents(const std::string &id);

        void remove_all_agents();

        AgentSeq get_agents() const;

        bool has_agents() const;

    private:
        std::atomic_bool running;
        std::mutex running_cond_mutex;
        std::condition_variable running_cond;

        std::thread run_thread;

        std::atomic_bool agents_modified;
        mutable std::mutex agents_mutex;
        std::vector<AgentPtr> agents_add_q;
        std::vector<AgentPtr> agents_remove_q;
        std::vector<std::string> ids_remove_q;
        bool clear_all;

    private:
        void process_agents_thread();
        void join_run_thread();
};

} /* namespace opencog */

#endif /* OPENCOG_SERVER_AGENTRUNNERTHREAD_H_ */
