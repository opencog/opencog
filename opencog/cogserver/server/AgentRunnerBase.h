/*
 * opencog/server/AgentRunnerBase.h
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

#ifndef OPENCOG_SERVER_AGENTRUNNERBASE_H_
#define OPENCOG_SERVER_AGENTRUNNERBASE_H_

#include <string>
#include <vector>
#include <opencog/cogserver/server/Agent.h>
#include <opencog/cogserver/server/SystemActivityTable.h>

namespace opencog
{

typedef std::vector<AgentPtr> AgentSeq;

/**
 * This class provides the basic building blocks for classes who are responsible
 * for running agents ('Runner' classes).
 */
class AgentRunnerBase
{
    public:
        AgentRunnerBase(std::string runner_name = "unnamed");
        AgentRunnerBase(AgentRunnerBase &&tmp) = default;
        ~AgentRunnerBase();

        void set_name(std::string new_name);
        const std::string &get_name() const;

        unsigned long get_cycle_count() const;

    protected:
        /** The runner name; mainly used for logging purposes */
        std::string name;

        /** Pointer to SystemActivityTable, owned by CogServer */
        SystemActivityTable* sat;

        /** Current cycle number (will reset to 0 if reaches max possible value) */
        unsigned long cycle_count;

        /** Agents controlled by this runner */
        AgentSeq agents;

    protected:
        /** Adds agent 'a' to the list of scheduled agents. */
        void add_agent(AgentPtr a);

        /** Removes agent 'a' from the list of scheduled agents. */
        void remove_agent(AgentPtr a);

        /** Removes all agents from class 'id' */
        void remove_all_agents(const std::string &id);

        /** Removes all agents */
        void remove_all_agents();

        /** Run an Agent and log its activity. */
        void run_agent(AgentPtr a);

        void set_activity_table(SystemActivityTable* sat);
};


/**
 * This class calls Agent::run() for its agents each time process_agents() is
 * called. This provides the classical behavior of CogServer.
 */
class SimpleRunner: public AgentRunnerBase
{
    public:
        SimpleRunner(const std::string &name = "simple"): AgentRunnerBase(name)
        {}

        /** Adds agent 'a' to the list of scheduled agents. */
        void add_agent(AgentPtr a) { AgentRunnerBase::add_agent(a); }

        /** Removes agent 'a' from the list of scheduled agents. */
        void remove_agent(AgentPtr a) { AgentRunnerBase::remove_agent(a); }

        /** Removes all agents from class 'id' */
        void remove_all_agents(const std::string &id)
        { AgentRunnerBase::remove_all_agents(id); }

        const AgentSeq &get_agents() const { return agents; }

        /**
         * Runs a single step of contained agents and returns when they are
         * finished.
         *
         * Each call to process_agents() is a processing cycle, and it selects
         * agents to run in each cycle based on their \link Agent::_frequency
         * frequency \endlink property.
         */
        void process_agents();

        void set_activity_table(SystemActivityTable* sat) { this->sat = sat; };
};

} /* namespace opencog */

#endif /* OPENCOG_SERVER_AGENTRUNNERBASE_H_ */
