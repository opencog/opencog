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
        AgentRunnerBase(name), running(false), agents_modified(false),
        clear_all(false)
{
}

AgentRunnerThread::~AgentRunnerThread()
{
    remove_all_agents();
    join_run_thread();
}

void AgentRunnerThread::start()
{
    {
        // locking is required, to prevent dead-lock in condition_variable wait()
        lock_guard<mutex> lock(running_cond_mutex);
        running.store(true, memory_order_relaxed);
    }
    running_cond.notify_one();
}

void AgentRunnerThread::stop()
{
    running.store(false, memory_order_relaxed);
}

void AgentRunnerThread::add_agent(AgentPtr a)
{
    if (!has_agents()) {
        join_run_thread();
        /*
         * process_agents_thread() will exit immediately if no agents are
         * available, so we add the first one immediately
         */
        {
            lock_guard<mutex> lock(agents_mutex);
            AgentRunnerBase::add_agent(a);
        }
        run_thread = thread(&AgentRunnerThread::process_agents_thread, this);
        // todo: set thread name when using pthreads
    }
    else {
        lock_guard<mutex> lock(agents_mutex);
        agents_add_q.push_back(a);
        agents_modified.store(true, memory_order_relaxed);
    }
}

void AgentRunnerThread::remove_agent(AgentPtr a)
{
    lock_guard<mutex> lock(agents_mutex);
    agents_remove_q.push_back(a);
    agents_modified.store(true, memory_order_relaxed);
}

void AgentRunnerThread::remove_all_agents(const std::string& id)
{
    lock_guard<mutex> lock(agents_mutex);
    ids_remove_q.push_back(id);
    agents_modified.store(true, memory_order_relaxed);
}

void AgentRunnerThread::remove_all_agents()
{
    lock_guard<mutex> lock(agents_mutex);
    clear_all = true;
    agents_modified.store(true, memory_order_relaxed);
}

AgentSeq AgentRunnerThread::get_agents() const
{
    lock_guard<mutex> lock(agents_mutex);
    return agents;
}

bool AgentRunnerThread::has_agents() const
{
    lock_guard<mutex> lock(agents_mutex);
    return !agents.empty();
}

void AgentRunnerThread::process_agents_thread()
{
    /*
     * 'agents' is only modified in this function while this thread is active,
     * so there is no need to protect read accesses to it in this function.
     */
    logger().debug("[CogServer::%s] Agent thread started", name.c_str());

    while (!agents.empty()) {
        if (!running.load(memory_order_relaxed)) {
            unique_lock<mutex> lock(running_cond_mutex);
            running_cond.wait(lock,
                [this] {return running.load(memory_order_relaxed);});
        }

        AgentSeq::const_iterator it;
        for (it = agents.begin(); it != agents.end(); ++it) {
            AgentPtr agent = *it;
            // todo: can frequency() be still useful?
//            if ((cycle_count % agent->frequency()) == 0)
            run_agent(agent);
        }

        if (agents_modified.load(memory_order_relaxed)) {
            logger().debug("[CogServer::%s] Updating active agents",
                name.c_str());
            lock_guard<mutex> agentsLock(agents_mutex);
            agents_modified.store(false, memory_order_relaxed);
            if (clear_all) {
                clear_all = false;
                AgentRunnerBase::remove_all_agents();
            } else {
                for (const auto &a : agents_remove_q)
                    AgentRunnerBase::remove_agent(a);
                for (const auto &id : ids_remove_q)
                    AgentRunnerBase::remove_all_agents(id);
            }
            agents_remove_q.clear();
            ids_remove_q.clear();
            for (const auto &a : agents_add_q)
                AgentRunnerBase::add_agent(a);
            agents_add_q.clear();
        }
        ++cycle_count;
    }
    logger().debug("[CogServer::%s] Agent thread stopped", name.c_str());
}

inline void AgentRunnerThread::join_run_thread()
{
    if (run_thread.joinable()) {
        start(); // unlock process_agents_thread() if stopped
        run_thread.join();
    }
}

} /* namespace opencog */
