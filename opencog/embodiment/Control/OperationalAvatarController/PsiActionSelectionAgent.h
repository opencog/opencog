/*
 * @file opencog/embodiment/Control/OperationalPetController/ActionSelectionAgent.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * 
 * @author Andre Senna, Zhenhua Cai <czheud@gmail.com>
 * @date 2010-11-15
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

#ifndef ACTIONSELECTIONAGENT_H
#define ACTIONSELECTIONAGENT_H

#include <opencog/server/Agent.h>

#include <time.h>

namespace OperationalPetController
{

using namespace std;

/**
 * @class
 *
 * @brief Action Selection Agent
 *
 * Ideally using PLN, looking for, or inferring cognitive schematics and choosing
 * the one according to their weights (calculating the weight can depend on the
 * Modulator Certainty).
 */
class ActionSelectionAgent : public opencog::Agent
{

private:

    time_t lastTickTime;

    int prepareForActionSelection(opencog::CogServer * server);
    void selectAction(opencog::CogServer * server);
    void executeAction(opencog::CogServer * server);

public:

    ~ActionSelectionAgent();
    ActionSelectionAgent();

    virtual const ClassInfo& classinfo() const {
        return info();
    }

    static const ClassInfo& info() {
        static const ClassInfo _ci(
                "OperationalPetController::ActionSelectionAgent");
        return _ci;
    }

    void run(opencog::CogServer * server);

}; // class

}  // namespace

#endif
