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
#include <opencog/server/Agent.h>

namespace opencog
{

typedef std::vector<AgentPtr> AgentSeq;

/**
 *
 */
class AgentRunnerBase
{
    public:
        AgentRunnerBase(std::string runner_name = "unnamed");
        AgentRunnerBase(AgentRunnerBase &&tmp) = default;
        ~AgentRunnerBase();

        void setName(std::string new_name);
        const std::string &getName() const;

        unsigned long getCycleCount() const;

    protected:
        std::string name;

        unsigned long cycleCount;

        AgentSeq agents;

    protected:
        /** Adds agent 'a' to the list of scheduled agents. */
        void addAgent(AgentPtr a);

        /** Removes agent 'a' from the list of scheduled agents. */
        void removeAgent(AgentPtr a);

        /** Removes all agents from class 'id' */
        void removeAllAgents(const std::string &id);

//        void processAgents();

        /** Run an Agent and log its activity. */
        void runAgent(AgentPtr a);
};


class SimpleRunner: public AgentRunnerBase
{
    public:
        SimpleRunner(const std::string &name = "simple"): AgentRunnerBase(name), running(false) {}

        /** Put runner into 'running' state, so that agents will be run when
         * processAgents() is called */
        void start() { running = true; }

        /** Put runner into 'stopped' state, so that nothing happens when
         * processAgents() is called */
        void stop() { running = false; }

        /** Adds agent 'a' to the list of scheduled agents. */
        void addAgent(AgentPtr a) { AgentRunnerBase::addAgent(a); }

        /** Removes agent 'a' from the list of scheduled agents. */
        void removeAgent(AgentPtr a) { AgentRunnerBase::removeAgent(a); }

        /** Removes all agents from class 'id' */
        void removeAllAgents(const std::string &id) { AgentRunnerBase::removeAllAgents(id); }

        const AgentSeq &getAgents() const { return agents; }

        void processAgents();

    private:
        bool running;
};

} /* namespace opencog */

#endif /* OPENCOG_SERVER_AGENTRUNNERBASE_H_ */
