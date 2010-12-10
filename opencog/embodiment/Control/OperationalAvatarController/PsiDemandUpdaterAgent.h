/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiDemandUpdaterAgent.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2010-12-09
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


#ifndef PSIDEMANDUPDATERAGENT_H
#define PSIDEMANDUPDATERAGENT_H

#include <opencog/server/Agent.h>
#include <opencog/atomspace/AtomSpace.h>

namespace OperationalAvatarController
{

/**
 * @class
 *
 * @brief Agent of updating Demands
 *
 * Updates the demands (for example, how hungry the agent is)
 * 
 * The format of DemandSchema in AtomSpace is
 *
 * SimilarityLink (stv 1.0 1.0)
 *     NumberNode: "demand_value"
 *     ExecutionOutputLink
 *         GroundedSchemaNode: xxxDemandUpdater
 *         ListLink
 *            PET_HANDLE
 *
 * DemandGoal is represented as:
 *
 * SimultaneousEquivalenceLink
 *     EvaluationLink
 *         PredicateNode: "demand_name_goal" 
 *                        (SimpleTruthValue indicates how well the demand is satisfied)
 *                        (ShortTermInportance indicates the urgency of the demand)
 *     EvaluationLink
 *         GroundedPredicateNode: "FuzzyWithin"
 *         ListLink
 *             NumberNode: "min_acceptable_value"
 *             NumberNode: "max_acceptable_value"
 *             SimilarityLink (stv 1.0 1.0)
 *                 NumberNode: "demand_value"
 *                 ExecutionOutputLink
 *                     GroundedSchemaNode: "demand_schema_name"
 *                     ListLink
 *                         PET_HANDLE
 *
*/
class PsiDemandUpdaterAgent : public opencog::Agent
{
private:

    // Helper class that stores meta data of a Demand 
    class DemandMeta
    {
    public:

        std::string updaterName; // The schema name that updates the Demand
        Handle similarityLink;   // Handle to SimilarityLink that holds the DemandSchema
        Handle simultaneousEquivalenceLink; // Handle to SimultaneousEquivalenceLink that
                                            // holds DemandGoal
        double updatedValue; // The updated value after calling the Demand updater  
        bool bUpdated;       // Indicate if the value of the DemandSchema has been updated

        void init ( const std::string & updaterName, 
                    Handle similarityLink,
                    Handle simultaneousEquivalenceLink) {
            this->updaterName = updaterName;
            this->similarityLink = similarityLink;
            this->simultaneousEquivalenceLink = simultaneousEquivalenceLink;
            this->updatedValue = 0;
            this->bUpdated = false;
        }
    }; // class

    unsigned long cycleCount;

    std::map <std::string, DemandMeta> demandMetaMap;  // Demand - Meta data map 

    // Initialize demandMetaMap etc.
    void init(opencog::CogServer * server);

    // Run updaters (combo scripts)
    void runUpdaters(opencog::CogServer * server);

    // Set updated values to AtomSpace (NumberNodes)
    void setUpdatedValues(opencog::CogServer * server);

    bool bInitialized; 

public:

    PsiDemandUpdaterAgent();
    virtual ~PsiDemandUpdaterAgent();

    virtual const ClassInfo& classinfo() const {
        return info();
    }

    static const ClassInfo& info() {
        static const ClassInfo _ci("OperationalAvatarController::PsiDemandUpdaterAgent");
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

