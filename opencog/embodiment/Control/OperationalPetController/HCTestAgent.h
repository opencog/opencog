/*
 * opencog/embodiment/Control/MessagingSystem/HCTestAgent.h
 *
 * Copyleft (C) 2007-2008 Nil Geisweiller
 * All Wrongs Reserved
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

#ifndef HCTESTAGENT_H
#define HCTESTAGENT_H

#include <opencog/server/Agent.h>
#include "MessageSender.h"

namespace OperationalPetController
{

using namespace opencog;

/**
 * That class is in charge of executing a scenario to test hillclimbing
 * hosted by LS
 */

class HCTestAgent : public Agent
{

    enum HCTestMode {
        HCT_IDLE,
        HCT_INIT,
        HCT_WAIT1,
        HCT_WAIT2,
        HCT_WAIT3,
        HCT_WAIT4
    };

private:
    HCTestMode mode;
    unsigned long cycle;
    std::string schemaName;
    std::vector<std::string> schemaArguments;
    std::string avatarId;
    std::string ownerId;
    AtomSpace* atomSpace;
    MessageSender* sender;

public:

    virtual const ClassInfo& classinfo() const {
        return info();
    }
    static const ClassInfo& info() {
        static const ClassInfo _ci("OperationalPetController::HCTestAgent");
        return _ci;
    }

    HCTestAgent();
    void init(std::string sn, std::vector<std::string> schemaArgs, std::string b, std::string a, AtomSpace* as, MessageSender* s);
    ~HCTestAgent();

    void run(opencog::CogServer* ne);

    void setWait2() {
        mode = HCT_WAIT2;
    }

    void setWait4() {
        mode = HCT_WAIT4;
    }

}; // class
}  // namespace

#endif
