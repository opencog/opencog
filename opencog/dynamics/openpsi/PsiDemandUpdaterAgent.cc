/*
 * @file opencog/dynamics/openpsi/PsiDemandUpdaterAgent.cc
 *
 * @author Jinhua Chua <JinhuaChua@gmail.com>
 * @date   2011-11-22
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
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/spacetime/SpaceTime.h>
#include <opencog/guile/SchemeEval.h>
#include <opencog/util/Config.h>
#include <opencog/util/octime.h>

// TODO: the methods from AtomSpaceUtil to openpsi directory
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>

#include "PsiDemandUpdaterAgent.h"

using namespace opencog;


bool PsiDemandUpdaterAgent::Demand::runUpdater(AtomSpace & atomSpace)
{
    std::string demandUpdater = this->demandName + "DemandUpdater";

#if HAVE_GUILE

    // Initialize scheme evaluator
    SchemeEval evaluator1(&atomSpace);
    std::string scheme_expression, scheme_return_value;

    scheme_expression = "( " + demandUpdater + " )";

    // Run the Procedure that update Demands and get the updated value
    scheme_return_value = evaluator1.eval(scheme_expression);

    if ( evaluator1.eval_error() ) {
        logger().error("PsiDemandUpdaterAgent::Demand::%s - "
                       "Failed to execute '%s'",
                       __FUNCTION__, scheme_expression.c_str());
        return false;
    }

    // Store the updated demand value (result)
    this->currentDemandValue = atof( scheme_return_value.c_str() );

    logger().debug("PsiDemandUpdaterAgent::Demand::%s - "
                   "The level of demand '%s' will be set to '%f'",
                   __FUNCTION__, this->demandName.c_str(),
                   this->currentDemandValue);

#endif // HAVE_GUILE

    return true;
}

bool PsiDemandUpdaterAgent::Demand::updateDemandGoal (AtomSpace & atomSpace, const octime_t timeStamp)
{
    // Get the GroundedPredicateNode "fuzzy_within"
    Handle hGroundedPredicateNode = atomSpace.get_outgoing(hFuzzyWithin, 0);

    bool isNotPredicateNode = atomSpace.get_type(hGroundedPredicateNode) != GROUNDED_PREDICATE_NODE;
    bool isOutgoingNotPredicateNode = atomSpace.get_type(atomSpace.get_outgoing(hGroundedPredicateNode, 0)) != GROUNDED_PREDICATE_NODE;

    if ( hGroundedPredicateNode == opencog::Handle::UNDEFINED || (isNotPredicateNode && isOutgoingNotPredicateNode)) {

        logger().error("PsiDemandUpdaterAgent::Demand::%s - Expect a GroundedPredicateNode for demand '%s'. But got '%s'",
                       __FUNCTION__, this->demandName.c_str(), atomSpace.atom_as_string(hGroundedPredicateNode).c_str());
        return false;
    }

    // Get ListLink containing ExecutionOutputLink and parameters of FuzzyWithin
    Handle hListLink = atomSpace.get_outgoing(hFuzzyWithin, 1);

    if ( hListLink == opencog::Handle::UNDEFINED ||
         atomSpace.get_type(hListLink) != LIST_LINK ||
         atomSpace.get_arity(hListLink) != 3 ) {

        logger().error("PsiDemandUpdaterAgent::Demand::%s - Expect a ListLink for demand '%s' with three arity (parameters of FuzzyWithin). But got '%s'",
                       __FUNCTION__, this->demandName.c_str(), atomSpace.atom_as_string(hListLink).c_str());
        return false;
    }

    // Get the ExecutionOutputLink
    Handle hExecutionOutputLink = atomSpace.get_outgoing(hListLink, 2);

    if ( hExecutionOutputLink == opencog::Handle::UNDEFINED ||
         atomSpace.get_type(hExecutionOutputLink) != EXECUTION_OUTPUT_LINK ||
         atomSpace.get_arity(hExecutionOutputLink) != 1 ) {

        logger().error("PsiDemandUpdaterAgent::Demand::%s - Expect an ExecutionOutputLink (updater) for demand '%s' with only one arity. But got '%s'",
                       __FUNCTION__, this->demandName.c_str(), atomSpace.atom_as_string(hExecutionOutputLink).c_str());
        return false;
    }

#if HAVE_GUILE

    // Initialize scheme evaluator
    SchemeEval evaluator1(&atomSpace);
    std::string scheme_expression, scheme_return_value;

    // Store the updated Demand levels to AtomSpace
    // set_modulator_or_demand_value would create a new NumberNode and SilarityLink
    //
    // Note: Since OpenCog would forget (remove) those Nodes and Links gradually,
    //       unless you create them to be permanent, don't worry about the overflow of memory.
    scheme_expression = "( set_modulator_or_demand_value \"" +
                               this->demandName + "Demand\" " +
                               boost::lexical_cast<std::string>(this->currentDemandValue) + " " +
                               boost::lexical_cast<std::string>(timeStamp) + " " +
                        ")";

    // Run the scheme procedure
    scheme_return_value = evaluator1.eval(scheme_expression);

    if ( evaluator1.eval_error() ) {
        logger().error( "PsiDemandUpdaterAgent::Demand::%s - Failed to execute '%s'",
                         __FUNCTION__, scheme_expression.c_str());

        return false;
    }

    logger().debug("PsiDemandUpdaterAgent::Demand::%s - Updated the value of '%s' demand to %f and store it to AtomSpace",
                   __FUNCTION__, this->demandName.c_str(), this->currentDemandValue);

    // Get the fuzzy_within scheme procedure name, which should be "fuzzy_within"
    std::string demandGoalEvaluator = atomSpace.get_name(hGroundedPredicateNode);

    // Get min/ max acceptable values
    std::string minValueStr = atomSpace.get_name( atomSpace.get_outgoing(hListLink, 0) );
    std::string maxValueStr = atomSpace.get_name( atomSpace.get_outgoing(hListLink, 1) );

    scheme_expression = "( fuzzy_within " +
                               boost::lexical_cast<std::string>(this->currentDemandValue) + " " +
                               minValueStr + " " +
                               maxValueStr + " " +
                               "100"
                         ")";

    // Run the scheme procedure
    scheme_return_value = evaluator1.eval(scheme_expression);

    if ( evaluator1.eval_error() ) {
        logger().error( "PsiDemandUpdaterAgent::Demand::%s - Failed to execute '%s' for demand '%s'",
                         __FUNCTION__, scheme_expression.c_str(), this->demandName.c_str());

        return false;
    }

    // Store the result and update TruthValue of EvaluationLinkDemandGoal and EvaluationLinkFuzzyWithin
    // TODO: Use PLN forward chainer to handle this?
    TruthValuePtr demand_satisfaction = SimpleTruthValue::createTV(atof(scheme_return_value.c_str()), 1.0f);

    atomSpace.set_TV(this->hDemandGoal, demand_satisfaction);

    // Add AtTimeLink around EvaluationLink of  DemandGoal, which is required by fishgram
    Handle atTimeLink = timeServer().addTimeInfo(this->hDemandGoal, timeStamp, DEFAULT_TIMEDOMAIN, demand_satisfaction);

//    // Update the LatestLink
    std::string predicateName = this->demandName + "DemandGoal";
    Handle demandPredicateNode = atomSpace.add_node(PREDICATE_NODE, predicateName.c_str());

    std::vector <Handle> outgoings;
    Handle listLink = atomSpace.add_link(LIST_LINK, outgoings);
    outgoings.push_back(demandPredicateNode);
    outgoings.push_back(listLink);
    Handle evaluationLink = atomSpace.add_link(EVALUATION_LINK, outgoings);
    atomSpace.set_TV(evaluationLink, demand_satisfaction);

    atTimeLink = timeServer().addTimeInfo(evaluationLink, timeStamp, DEFAULT_TIMEDOMAIN, demand_satisfaction);

    AtomSpaceUtil::updateLatestDemand(atomSpace, atTimeLink, demandPredicateNode);

    atomSpace.set_TV( this->hFuzzyWithin, demand_satisfaction);

    this->currentDemandTruthValue = atof(scheme_return_value.c_str());

    logger().debug( "PsiDemandUpdaterAgent::Demand::%s - The level (truth value) of DemandGoal '%s' has been set to %s",
                     __FUNCTION__, this->demandName.c_str(), scheme_return_value.c_str());

    return true;

#else // HAVE_GUILE
    logger().error("PsiDemandUpdaterAgent::Demand::%s - guile is required",
                   __FUNCTION__);
    return false;
#endif // HAVE_GUILE

}

PsiDemandUpdaterAgent::~PsiDemandUpdaterAgent()
{
    logger().info("[PsiDemandUpdaterAgent] destructor");
}


PsiDemandUpdaterAgent::PsiDemandUpdaterAgent(CogServer& cs) : Agent(cs)
{
    this->cycleCount = 0;

    // This is needed for time stamping
    initReferenceTime();
    // Force the Agent initialize itself during its first cycle.
    this->forceInitNextCycle();

}

void PsiDemandUpdaterAgent::init()
{
    logger().debug( "PsiDemandUpdaterAgent::%s - Initialize the Agent [cycle = %d]",
                    __FUNCTION__, this->cycleCount);

    // Get AtomSpace
    AtomSpace& atomSpace = _cogserver.getAtomSpace();

    // Clear old demandList
    this->demandList.clear();

    // Get demand names from the configuration file
    std::string demandNames = config()["PSI_DEMANDS"];

    // Process Demands one by one
    boost::char_separator<char> sep(", ");
    boost::tokenizer< boost::char_separator<char> > demandNamesTok (demandNames, sep);

    std::string demandName, demandUpdater;
    Handle hDemandGoal, hFuzzyWithin;

    // Process Demands one by one
    for ( boost::tokenizer< boost::char_separator<char> >::iterator iDemandName = demandNamesTok.begin();
          iDemandName != demandNamesTok.end();
          iDemandName ++ ) {

        demandName = (*iDemandName);
        demandUpdater = demandName + "DemandUpdater";

        // Search the corresponding SimultaneousEquivalenceLink
        if ( !AtomSpaceUtil::getDemandEvaluationLinks(atomSpace, demandName, hDemandGoal, hFuzzyWithin) )
        {
            logger().warn( "PsiDemandUpdaterAgent::%s - Failed to get EvaluationLinks for demand '%s' [cycle = %d]",
                           __FUNCTION__, demandName.c_str(), this->cycleCount);

            continue;
        }

        this->demandList.push_back(Demand(demandName, hDemandGoal, hFuzzyWithin));

        logger().debug("PsiDemandUpdaterAgent::%s - Store the meta data of demand '%s' successfully [cycle = %d]",
                        __FUNCTION__, demandName.c_str(), this->cycleCount);
    }// for


    // Avoid initialize during next cycle
    this->bInitialized = true;

    hasPsiDemandUpdaterForTheFirstTime = false;
}

void PsiDemandUpdaterAgent::run()
{
    this->cycleCount = _cogserver.getCycleCount();

    logger().debug( "PsiDemandUpdaterAgent::%s - Executing run %d times",
                     __FUNCTION__, this->cycleCount);

    // Get AtomSpace
    AtomSpace& atomSpace = _cogserver.getAtomSpace();
    // Get current time stamp
    octime_t timeStamp = getElapsedMillis();

    // Initialize the Agent (demandList etc)
    if ( !this->bInitialized )
        this->init();

    // Update demand values
    for (Demand & demand : this->demandList) {
        logger().debug("PsiDemandUpdaterAgent::%s - Going to run updaters for demand '%s' [cycle = %d]",
                       __FUNCTION__, demand.getDemandName().c_str(), this->cycleCount);

        demand.runUpdater(atomSpace);
    }

    // Update Demand Goals
    for (Demand & demand : this->demandList) {
        logger().debug("PsiDemandUpdaterAgent::%s - Going to set the updated value to AtomSpace for demand '%s' [cycle = %d]",
                       __FUNCTION__, demand.getDemandName().c_str(), this->cycleCount);
        demand.updateDemandGoal(atomSpace, timeStamp);
    }

    hasPsiDemandUpdaterForTheFirstTime = true;

}

double PsiDemandUpdaterAgent::getDemandValue(string demanName) const
{
    for (const Demand & demand : this->demandList) {
        if ( demand.getDemandName() == demanName)
            return demand.getDemandValue();
    }

    return 0.0000;
}
