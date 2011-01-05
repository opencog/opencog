/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiRelationUpdaterAgent.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2010-12-23
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


#ifndef PSIRELATIONUPDATERAGENT_H
#define PSIRELATIONUPDATERAGENT_H

#include <opencog/server/Agent.h>
#include <opencog/atomspace/AtomSpace.h>

namespace OperationalAvatarController
{

/**
 * Original ideas from RuleEngine
 *
 * step1. Write Relation rules in pet_rules.lua
 * step2. RuleEngine::processRules if the Precondition meets, add the corresponding 
 *        Relation suggest Relations via suggestRelation method
 * step3. Apply Relation in RuleEngine::processNextAction through addRelation, 
 *        removeRelation removeOppositeRelation methods
 *
 */     

/**
 * @class
 *
 * @brief Agent of updating relations
 *
 * For instance, if the pet licks X it becomes Familiar with X, Familiar(self, X) relation is
 * then created
 * 
 * The format of RelationSchema in AtomSpace is
 *
 * SimilarityLink (stv 1.0 1.0)
 *     NumberNode: "demand_value"
 *     ExecutionOutputLink
 *         GroundedSchemaNode: xxxRelationUpdater
 *         ListLink
 *            PET_HANDLE
 *
 * RelationGoal is represented as:
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
class PsiRelationUpdaterAgent : public opencog::Agent
{
private:

    // Helper class that stores meta data of a Relation 
    class RelationMeta
    {
    public:

        std::string updaterName; // The schema name that updates the Relation
        Handle similarityLink;   // Handle to SimilarityLink that holds the RelationSchema
        Handle simultaneousEquivalenceLink; // Handle to SimultaneousEquivalenceLink that
                                            // holds RelationGoal
        double updatedValue; // The updated value after calling the Relation updater  
        bool bUpdated;       // Indicate if the value of the RelationSchema has been updated

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

    std::map <std::string, RelationMeta> demandMetaMap;  // Relation - Meta data map 

    // Initialize demandMetaMap etc.
    void init(opencog::CogServer * server);

    // Run updaters (combo scripts)
    void runUpdaters(opencog::CogServer * server);

    // Set updated values to AtomSpace (NumberNodes)
    void setUpdatedValues(opencog::CogServer * server);

    // Update PredicateNodes of corresponding RelationGoals
    void updateRelationGoals(opencog::CogServer * server);

    bool bInitialized; 

public:

    PsiRelationUpdaterAgent();
    virtual ~PsiRelationUpdaterAgent();

    virtual const ClassInfo& classinfo() const {
        return info();
    }

    static const ClassInfo& info() {
        static const ClassInfo _ci("OperationalAvatarController::PsiRelationUpdaterAgent");
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
