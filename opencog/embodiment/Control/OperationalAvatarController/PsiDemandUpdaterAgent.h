/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiDemandUpdaterAgent.h
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2011-03-14
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
 * AtTimeLink
 *     TimeNode "timestamp"
 *     SimilarityLink (stv 1.0 1.0)
 *         NumberNode: "demand_value"
 *         ExecutionOutputLink
 *             GroundedSchemaNode: xxxDemandUpdater
 *             ListLink (empty)
 *
 * The connection of Demand and Goal is represented as:
 *
 * SimultaneousEquivalenceLink
 *     EvaluationLink
 *         (SimpleTruthValue indicates how well the demand is satisfied)
 *         (ShortTermInportance indicates the urgency of the demand)
 *         PredicateNode: "demand_name_goal" 
 *         ListLink (empty)
 *     EvaluationLink
 *         GroundedPredicateNode: "FuzzyWithin"
 *         ListLink
 *             NumberNode: "min_acceptable_value"
 *             NumberNode: "max_acceptable_value"
 *             ExecutionOutputLink
 *                 GroundedSchemaNode: "demand_schema_name"
 *                 ListLink (empty)
 *
*/
class PsiDemandUpdaterAgent : public opencog::Agent
{
private:

    /**
     * Inner class for Demand
     */
    class Demand
    {
    public:

        Demand(const std::string & demandName, Handle hDemandGoal, Handle hFuzzyWithin) :
            demandName(demandName), hDemandGoal(hDemandGoal), hFuzzyWithin(hFuzzyWithin)
        {};

        inline const std::string & getDemandName() 
        {
            return this->demandName;
        }

        inline Handle getHandleDemandGoal()
        {
            return this->hDemandGoal;
        }
   
        inline Handle getHandleFuzzyWithin()
        {
            return this->hFuzzyWithin;
        }

        /**
         * Update the Demand Value
         *
         * @return return true if update successfully, otherwise false
         */
        bool runUpdater(const AtomSpace & atomSpace, 
                        Procedure::ProcedureInterpreter & procedureInterpreter, 
                        const Procedure::ProcedureRepository & procedureRepository);

        /**
         * Update the truth value of Demand Goal
         *
         * @return return true if update successfully, otherwise false
         *
         * TODO: Only current (latest) demand value is considered, also take previous demand values in future
         *
         * @note This function does two jobs as follows:
         *
         *       1. Create a new NumberNode and SimilarityLink to store the result, 
         *          and then time stamp the SimilarityLink.
         *
         *          Since OpenCog would forget (remove) those Nodes and Links gradually, 
         *          unless you create them to be permanent, don't worry about the overflow of memory. 
         *
         *          AtTimeLink
         *              TimeNode "timestamp"
         *              SimilarityLink (stv 1.0 1.0)
         *                  NumberNode: "demand_value"
         *                  ExecutionOutputLink
         *                      GroundedSchemaNode: xxxDemandUpdater
         *                      ListLink (empty)
         *
         *       2. Run the procedure named "FuzzyWithin", and then set the result to the truth value of 
         *         both EvaluationLink of Demand Goal and FuzzyWithin
         */
        bool updateDemandGoal(AtomSpace & atomSpace, 
                              Procedure::ProcedureInterpreter & procedureInterpreter,
                              const Procedure::ProcedureRepository & procedureRepository, 
                              const unsigned long timeStamp);

    private:        

        std::string demandName; // The name of the Demand

        Handle hDemandGoal;   // Handle to Demand Goal (EvaluationLink)
        Handle hFuzzyWithin;  // Handle to FuzzyWithin (EvaluationLink)

        double currentDemandValue; // Store current (latest) value of Demand, 

    };// class Demand

    unsigned long cycleCount;

    std::vector<Demand> demandList; // List of Demands

    // Initialize demandList etc.
    void init(opencog::CogServer * server);

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

