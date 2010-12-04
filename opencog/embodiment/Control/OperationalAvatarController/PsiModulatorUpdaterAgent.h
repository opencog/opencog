/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiModulatorUpdaterAgent.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2010-12-04
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


#ifndef MODULATORUPDATERAGENT_H
#define MODULATORUPDATERAGENT_H

#include <opencog/server/Agent.h>
#include <opencog/atomspace/AtomSpace.h>
#include <time.h>

namespace OperationalAvatarController
{

/**
 * @class
 *
 * @brief Agent for updating modulators
 *
 * Controls the dynamic of the emotion of the agent (Perhaps how angry an
 * agent is. Note that "angry" should either correspond to a modulator, or,
 * more likely a configuration of modulators, but this is just an example).
 */
class PsiModulatorUpdaterAgent : public opencog::Agent
{
private:

    // Helper class that stores meta data of a Modulator 
    class ModulatorMeta
    {
    public:

        std::string updaterName;   // The schema name that updates the Modulator
        Handle numberNode;         // Handle to NumberNode that stores the value of 
                                   // the Modulator
        double updatedValue;       // The updated value after calling the Modulator updater  
        bool bUpdated;             // Indicate if it has been updated

        void init (const std::string & updaterName, Handle numberNode) {
            this->updaterName = updaterName;
            this->numberNode = numberNode;
            this->updatedValue = 0;
            this->bUpdated = false;
        }
    }; // class

    time_t lastTickTime;

    std::map <std::string, ModulatorMeta> modulatorMetaMap;  // Modulator - Meta data map 

    // Initialize modulatorMetaMap etc.
    void init(opencog::CogServer * server);

    // Run updaters (combo scripts)
    void runUpdaters(opencog::CogServer * server);

    // Set updated values to AtomSpace (NumberNodes)
    void setUpdatedValues(opencog::CogServer * server);

    bool bInitialized; 

public:

    PsiModulatorUpdaterAgent();
    virtual ~PsiModulatorUpdaterAgent();

    virtual const ClassInfo& classinfo() const {
        return info();
    }

    static const ClassInfo& info() {
        static const ClassInfo _ci("OperationalAvatarController::PsiModulatorUpdaterAgent");
        return _ci;
    }

    // Entry of the Agent, CogServer will invoke this function during its cycle
    void run(opencog::CogServer * server);

    // After calling this function, the Agent will invoke its "init" method firstly 
    // in "run" function during its next cycle
    void forceInitNextCycle() {
        this->bInitialized = false;
    }

}; // class

}  // namespace

#endif
