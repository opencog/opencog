/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiDemandUpdaterAgent.cc
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2011-05-11
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

#include "OAC.h"
#include "PsiDemandUpdaterAgent.h"

#include "PsiRuleUtil.h"

#include "opencog/web/json_spirit/json_spirit.h"

#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

using namespace OperationalAvatarController;

bool PsiDemandUpdaterAgent::Demand::runUpdater(AtomSpace & atomSpace)
{
    std::string demandUpdater = this->demandName + "DemandUpdater";

#if HAVE_GUILE    

    // Initialize scheme evaluator
    SchemeEval & evaluator = SchemeEval::instance(&atomSpace);    
    std::string scheme_expression, scheme_return_value;

    scheme_expression = "( " + demandUpdater + " )";

    // Run the Procedure that update Demands and get the updated value
    scheme_return_value = evaluator.eval(scheme_expression);

    if ( evaluator.eval_error() ) {
        logger().error( "PsiDemandUpdaterAgent::Demand::%s - Failed to execute '%s'", 
                         __FUNCTION__, 
                         scheme_expression.c_str() 
                      );

        return false; 
    }

    // Store the updated demand value (result)
    this->currentDemandValue = atof( scheme_return_value.c_str() );

    logger().debug("PsiDemandUpdaterAgent::Demand::%s - The level of demand '%s' will be set to '%f'", 
                   __FUNCTION__, 
                   this->demandName.c_str(), 
                   this->currentDemandValue
                  );

#endif // HAVE_GUILE    

    return true; 
}    

bool PsiDemandUpdaterAgent::Demand::updateDemandGoal (AtomSpace & atomSpace, const unsigned long timeStamp)
{
    // Get the GroundedPredicateNode "fuzzy_within"
    Handle hGroundedPredicateNode = atomSpace.getOutgoing(hFuzzyWithin, 0);

    if ( hGroundedPredicateNode == opencog::Handle::UNDEFINED ||
         atomSpace.getType(hGroundedPredicateNode) != GROUNDED_PREDICATE_NODE ) {

        logger().error("PsiDemandUpdaterAgent::Demand::%s - Expect a GroundedPredicateNode for demand '%s'. But got '%s'", 
                       __FUNCTION__, 
                       this->demandName.c_str(), 
                       atomSpace.atomAsString(hGroundedPredicateNode).c_str()
                      );
        return false; 
    }

    // Get ListLink containing ExecutionOutputLink and parameters of FuzzyWithin
    Handle hListLink = atomSpace.getOutgoing(hFuzzyWithin, 1);

    if ( hListLink == opencog::Handle::UNDEFINED ||
         atomSpace.getType(hListLink) != LIST_LINK || 
         atomSpace.getArity(hListLink) != 3 ) {

        logger().error("PsiDemandUpdaterAgent::Demand::%s - Expect a ListLink for demand '%s' with three arity (parameters of FuzzyWithin). But got '%s'", 
                       __FUNCTION__, 
                       this->demandName.c_str(), 
                       atomSpace.atomAsString(hListLink).c_str()
                      );
        return false; 
    }

    // Get the ExecutionOutputLink
    Handle hExecutionOutputLink = atomSpace.getOutgoing(hListLink, 2);

    if ( hExecutionOutputLink == opencog::Handle::UNDEFINED ||
         atomSpace.getType(hExecutionOutputLink) != EXECUTION_OUTPUT_LINK ||
         atomSpace.getArity(hExecutionOutputLink) != 2 ) {

        logger().error("PsiDemandUpdaterAgent::Demand::%s - Expect a ExecutionOutputLink for demand '%s' with two arity that contains an updater. But got '%s'", 
                       __FUNCTION__, 
                       this->demandName.c_str(), 
                       atomSpace.atomAsString(hExecutionOutputLink).c_str()
                      );
        return false; 
    }

#if HAVE_GUILE    

    // Initialize scheme evaluator
    SchemeEval & evaluator = SchemeEval::instance(&atomSpace);    
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
    scheme_return_value = evaluator.eval(scheme_expression);

    if ( evaluator.eval_error() ) {
        logger().error( "PsiDemandUpdaterAgent::Demand::%s - Failed to execute '%s'", 
                         __FUNCTION__, 
                         scheme_expression.c_str() 
                      );

        return false; 
    }

    logger().debug("PsiDemandUpdaterAgent::Demand::%s - Updated the value of '%s' demand to %f and store it to AtomSpace", 
                   __FUNCTION__, 
                   this->demandName.c_str(), 
                   this->currentDemandValue
                  );

    // Get the fuzzy_within scheme procedure name, which should be "fuzzy_within"
    std::string demandGoalEvaluator = atomSpace.getName(hGroundedPredicateNode);

    // Get min/ max acceptable values
    std::string minValueStr = atomSpace.getName( atomSpace.getOutgoing(hListLink, 0) );
    std::string maxValueStr = atomSpace.getName( atomSpace.getOutgoing(hListLink, 1) );

    scheme_expression = "( fuzzy_within " + 
                               boost::lexical_cast<std::string>(this->currentDemandValue) + " " + 
                               minValueStr + " " +
                               maxValueStr + " " +
                               "100"
                         ")";   

    // Run the scheme procedure
    scheme_return_value = evaluator.eval(scheme_expression);

    if ( evaluator.eval_error() ) {
        logger().error( "PsiDemandUpdaterAgent::Demand::%s - Failed to execute '%s' for demand '%s'", 
                         __FUNCTION__, 
                         scheme_expression.c_str(), 
                         this->demandName.c_str()
                      );

        return false; 
    }

    // Store the result and update TruthValue of EvaluationLinkDemandGoal and EvaluationLinkFuzzyWithin
    // TODO: Use PLN forward chainer to handle this?
    atomSpace.setTV( this->hDemandGoal,
                     SimpleTruthValue(atof(scheme_return_value.c_str()), 1.0f)
                   );

    atomSpace.setTV( this->hFuzzyWithin,
                     SimpleTruthValue(atof(scheme_return_value.c_str()), 1.0f)
                   );

    this->currentDemandTruthValue = atof(scheme_return_value.c_str());

    logger().debug( "PsiDemandUpdaterAgent::Demand::%s - The level (truth value) of DemandGoal '%s' has been set to %s", 
                     __FUNCTION__, 
                     this->demandName.c_str(),
                     scheme_return_value.c_str()
                  );

    return true; 

#endif // HAVE_GUILE    

}    

PsiDemandUpdaterAgent::~PsiDemandUpdaterAgent()
{
#ifdef HAVE_ZMQ
    delete this->publisher; 
#endif
}

PsiDemandUpdaterAgent::PsiDemandUpdaterAgent()
{
    this->cycleCount = 0;

    // Force the Agent initialize itself during its first cycle. 
    this->forceInitNextCycle();
}

#ifdef HAVE_ZMQ
void PsiDemandUpdaterAgent::publishUpdatedValue(Plaza & plaza, 
                                                zmq::socket_t & publisher, 
                                                const unsigned long timeStamp)
{
    using namespace json_spirit; 

    // Send the name of current mind agent which would be used as a filter key by subscribers
    std::string keyString = "PsiDemandUpdaterAgent"; 
    plaza.publishStringMore(publisher, keyString); 

    // Pack time stamp and all the Demand values in json format 
    Object jsonObj; // json_spirit::Object is of type std::vector< Pair >
    jsonObj.push_back( Pair("timestamp", (uint64_t) timeStamp) );

    foreach (Demand & demand, this->demandList) {
        jsonObj.push_back( Pair( demand.getDemandName()+"TruthValue", demand.getDemandTruthValue() ) );
    }

    // Publish the data packed in json format
    std::string dataString = write_formatted(jsonObj);
    plaza.publishString(publisher, dataString);
}
#endif // HAVE_ZMQ

void PsiDemandUpdaterAgent::init(opencog::CogServer * server) 
{
    logger().debug( "PsiDemandUpdaterAgent::%s - Initialize the Agent [cycle = %d]",
                    __FUNCTION__, 
                    this->cycleCount
                  );

    // Get OAC
    OAC * oac = (OAC *) server;

    // Get AtomSpace
    AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    // Get petId
    const std::string & petId = oac->getPet().getPetId();

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
                           __FUNCTION__, 
                           demandName.c_str(), 
                           this->cycleCount
                         );

            continue;
        }

        this->demandList.push_back(Demand(demandName, hDemandGoal, hFuzzyWithin));

        logger().debug("PsiDemandUpdaterAgent::%s - Store the meta data of demand '%s' successfully [cycle = %d]", 
                        __FUNCTION__, 
                        demandName.c_str(), 
                        this->cycleCount
                      );
    }// for

    // Initialize ZeroMQ publisher and add it to the plaza
#ifdef HAVE_ZMQ
    Plaza & plaza = oac->getPlaza();
    this->publisher = new zmq::socket_t (plaza.getZmqContext(), ZMQ_PUB);
    this->publishEndPoint = "ipc://" + petId + ".PsiDemandUpdaterAgent.ipc"; 
    this->publisher->bind( this->publishEndPoint.c_str() );

    plaza.addPublisher(this->publishEndPoint); 
#endif    

    // Avoid initialize during next cycle
    this->bInitialized = true;
}

void PsiDemandUpdaterAgent::run(opencog::CogServer * server)
{
    this->cycleCount ++;

    logger().debug( "PsiDemandUpdaterAgent::%s - Executing run %d times",
                     __FUNCTION__, 
                     this->cycleCount
                  );

    // Get OAC
    OAC * oac = (OAC *) server;

    // Get AtomSpace
    AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    // Get Pet
    Pet & pet = oac->getPet();

    // Get petId
    const std::string & petId = pet.getPetId();

    // Get ProcedureInterpreter
    Procedure::ProcedureInterpreter & procedureInterpreter = oac->getProcedureInterpreter();

    // Get Procedure repository
    const Procedure::ProcedureRepository & procedureRepository = oac->getProcedureRepository();

    // Get current time stamp
    unsigned long timeStamp = atomSpace.getTimeServer().getLatestTimestamp();

    // Check if map info data is available
    if ( atomSpace.getSpaceServer().getLatestMapHandle() == Handle::UNDEFINED ) {
        logger().warn( 
      "PsiDemandUpdaterAgent::%s - There is no map info available yet [cycle = %d]", 
                        __FUNCTION__, 
                        this->cycleCount
                     );
        return;
    }

    // Check if the pet spatial info is already received
    if ( !atomSpace.getSpaceServer().getLatestMap().containsObject(petId) ) {
        logger().warn(
 "PsiDemandUpdaterAgent::%s - Pet was not inserted in the space map yet [cycle = %d]", 
                     __FUNCTION__, 
                     this->cycleCount
                     );
        return;
    }

    // Initialize the Agent (demandList etc)
    if ( !this->bInitialized )
        this->init(server);

    // Update demand values
    foreach (Demand & demand, this->demandList) {
        logger().debug("PsiDemandUpdaterAgent::%s - Going to run updaters for demand '%s' [cycle = %d]", 
                       __FUNCTION__, 
                       demand.getDemandName().c_str(), 
                       this->cycleCount
                      );

        demand.runUpdater(atomSpace);
    }

    // Update Demand Goals
    foreach (Demand & demand, this->demandList) {
        logger().debug("PsiDemandUpdaterAgent::%s - Going to set the updated value to AtomSpace for demand '%s' [cycle = %d]", 
                       __FUNCTION__, 
                       demand.getDemandName().c_str(), 
                       this->cycleCount
                      );

        demand.updateDemandGoal(atomSpace, timeStamp);
    }

#ifdef HAVE_ZMQ    
    // Publish updated Demand values via ZeroMQ
    Plaza & plaza = oac->getPlaza();
//    this->publishUpdatedValue(plaza, *this->publisher, timeStamp); 
#endif 

    // Update the truth value of previous/ current demand goal
    Handle hReferenceLink, hPreviousDemandGoal, hCurrentDemandGoal; 

    hPreviousDemandGoal = PsiRuleUtil::getPreviousDemandGoal(atomSpace, hReferenceLink); 

    if ( hPreviousDemandGoal != opencog::Handle::UNDEFINED ) {
        logger().debug("PsiDemandUpdaterAgent::%s - Previous demand goal stored in pet is '%s' [cycle = %d]", 
                       __FUNCTION__, 
                       atomSpace.atomAsString(hPreviousDemandGoal).c_str(), 
                       this->cycleCount
                      );

        atomSpace.setTV( AtomSpaceUtil::getDemandGoalEvaluationLink(atomSpace, PREVIOUS_DEMAND_NAME), 
                         * atomSpace.getTV( hPreviousDemandGoal )
                       );

        logger().debug("PsiDemandUpdaterAgent::%s - Previous demand goal has been updated to: '%s' [cycle = %d]", 
                       __FUNCTION__, 
                       atomSpace.atomAsString(AtomSpaceUtil::getDemandGoalEvaluationLink(atomSpace, PREVIOUS_DEMAND_NAME)).c_str(), 
                       this->cycleCount
                      );
    }

    hCurrentDemandGoal = PsiRuleUtil::getCurrentDemandGoal(atomSpace, hCurrentDemandGoal); 

    if ( hCurrentDemandGoal != opencog::Handle::UNDEFINED ) {
        logger().debug("PsiDemandUpdaterAgent::%s - Current demand goal stored in pet is '%s' [cycle = %d]", 
                       __FUNCTION__, 
                       atomSpace.atomAsString(hCurrentDemandGoal).c_str(), 
                       this->cycleCount
                      );


        atomSpace.setTV( AtomSpaceUtil::getDemandGoalEvaluationLink(atomSpace, CURRENT_DEMAND_NAME), 
                         * atomSpace.getTV( hCurrentDemandGoal )
                       );

        logger().debug("PsiDemandUpdaterAgent::%s - Current demand goal has been updated to: '%s' [cycle = %d]", 
                       __FUNCTION__, 
                       atomSpace.atomAsString(AtomSpaceUtil::getDemandGoalEvaluationLink(atomSpace, CURRENT_DEMAND_NAME)).c_str(), 
                       this->cycleCount
                      );

    }

}

