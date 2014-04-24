/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PatternMinningAgent.h
 *
 * @author Shujing Ke <rainkekekeke@gmail.com>
 * @date   2014-04-11
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

#ifndef PATTERNMININGAGENT_H
#define PATTERNMININGAGENT_H

#include <opencog/server/Agent.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/learning/PatternMiner/PatternMiner.h>

using namespace opencog::PatternMining;

namespace opencog { namespace oac {

/**
 * @class
 *
 * @brief Agent of pattern mining
 */

class PatternMiningAgent  : public opencog::Agent
{

private:

    unsigned long cycleCount;

    bool bInitialized; 

    PatternMiner* patternMiner;

    AtomSpace* corpusAtomSpace;

    void init();

public:

    PatternMiningAgent(CogServer&);
    virtual ~PatternMiningAgent();

    virtual const ClassInfo& classinfo() const {
        return info();
    }

    static const ClassInfo& info() {
        static const ClassInfo _ci("OperationalAvatarController::PatternMinningAgent");
        return _ci;
    }

    // Entry of the Agent, CogServer will invoke this function during its cycle
    virtual void run();

    // After calling this function, the Agent will invoke its "init" method firstly
    // in "run" function during its next cycle
    void forceInitNextCycle() {
        this->bInitialized = false;}

}; // class

typedef std::shared_ptr<PatternMiningAgent> PatternMiningAgentPtr;

} } // namespace opencog::oac

#endif

