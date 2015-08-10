/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiFeelingUpdaterAgent.cc
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2011-04-19
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
#include <lib/json_spirit/json_spirit.h>

#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/guile/SchemeEval.h>
#include <opencog/util/Config.h>
#include <opencog/server/CogServer.h>

// TODO: the methods from AtomSpaceUtil to openpsi directory
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>

#include "PsiFeelingUpdaterAgent.h"

using namespace opencog;

PsiFeelingUpdaterAgent::~PsiFeelingUpdaterAgent()
{
    logger().info("[PsiFeelingUpdaterAgent] destructor");
}

PsiFeelingUpdaterAgent::PsiFeelingUpdaterAgent(CogServer& cs) : Agent(cs)
{
    this->cycleCount = 0;

    // Force the Agent initialize itself during its first cycle.
    this->forceInitNextCycle();
}

void PsiFeelingUpdaterAgent::init()
{
    logger().debug( "PsiFeelingUpdaterAgent::%s - Initializing the Agent"
                    " [ cycle = %d ]", __FUNCTION__, this->cycleCount);

    // Get AtomSpace
    AtomSpace& atomSpace = _cogserver.getAtomSpace();

    //  This is a temporary hack for defining a procssing-agent (could be a game
    // world character or a conglormation of processes like the dialogue system)
    // TODO: Should handle morethan one agents and should get the info from the
    // atomspace not the opencog.conf becuase the configuration file should only
    // deal with controlling parameters and not the Agents/processes, active
    // the atomspace.
    const std::string & agentId = "PsiAgent1";

    // Get petHandle
    Handle petHandle = AtomSpaceUtil::getAgentHandle(atomSpace, agentId);

    if ( petHandle == Handle::UNDEFINED ) {
        logger().warn("PsiFeelingUpdaterAgent::%s - Failed to get the handle to"
                      " the pet ( id = '%s' ) [ cycle = %d ]",
                        __FUNCTION__, agentId.c_str(), this->cycleCount);
        return;
    }

    // Clear old feelingMetaMap;
    this->feelingMetaMap.clear();

    // Get feeling names from the configuration file
    std::string feelingNames = config()["PSI_FEELINGS"];

    // Process feelings one by one
    boost::tokenizer<> feelingNamesTok (feelingNames);
    std::string feeling, feelingUpdater;
    FeelingMeta feelingMeta;

    for ( boost::tokenizer<>::iterator iFeelingName = feelingNamesTok.begin();
          iFeelingName != feelingNamesTok.end();
          iFeelingName ++ ) {
        feeling = (*iFeelingName);
        feelingUpdater = feeling + "FeelingUpdater";

        logger().debug( "PsiFeelingUpdaterAgent::%s - Searching the meta data"
                        "of feeling '%s'.", __FUNCTION__, feeling.c_str());

        // Get the corresponding EvaluationLink of the pet's feeling
        Handle evaluationLink = this->getFeelingEvaluationLink(&_cogserver, feeling, petHandle);

        if ( evaluationLink == Handle::UNDEFINED )
        {
            logger().warn( "PsiFeelingUpdaterAgent::%s - Failed to get the EvaluationLink for feeling '%s'",
                           __FUNCTION__, feeling.c_str());

            continue;// TODO: the methods from AtomSpaceUtil to openpsi directory
        }

        // Insert the meta data of the feeling to feelingMetaMap
        feelingMeta.init(feelingUpdater, evaluationLink);
        feelingMetaMap[feeling] = feelingMeta;

        logger().debug( "PsiFeelingUpdaterAgent::%s - Store the meta data of feeling '%s' successfully.",
                        __FUNCTION__, feeling.c_str());
    }// for

    // Avoid initialize during next cycle
    this->bInitialized = true;
}

Handle PsiFeelingUpdaterAgent::getFeelingEvaluationLink(opencog::CogServer * server,
                                                        const std::string feelingName,
                                                        Handle petHandle)
{
    // Get AtomSpace
    AtomSpace& atomSpace = server->getAtomSpace();

    // Get the Handle to feeling (PredicateNode)
    Handle feelingPredicateHandle = atomSpace.get_handle(PREDICATE_NODE, feelingName);

    if (feelingPredicateHandle == Handle::UNDEFINED) {
        logger().warn("PsiFeelingUpdaterAgent::%s - Failed to find the PredicateNode for feeling '%s' [ cycle = %d ].",
                       __FUNCTION__, feelingName.c_str(), this->cycleCount);

        return opencog::Handle::UNDEFINED;
    }

    // Get the Handle to ListLink that contains the pet handle
    std::vector<Handle> listLinkOutgoing;

    listLinkOutgoing.push_back(petHandle);

    Handle listLinkHandle = atomSpace.get_handle(LIST_LINK, listLinkOutgoing);

    if (listLinkHandle == Handle::UNDEFINED) {
        logger().warn( "PsiFeelingUpdaterAgent::%s - Failed to find the ListLink containing the pet ( id = '%s' ) [ cycle = %d ].",
                        __FUNCTION__, atomSpace.get_name(petHandle).c_str(), this->cycleCount);

        return opencog::Handle::UNDEFINED;
    }

    // Get the Handle to EvaluationLink holding the pet's feeling
    std::vector<Handle> evaluationLinkOutgoing;

    evaluationLinkOutgoing.push_back(feelingPredicateHandle);
    evaluationLinkOutgoing.push_back(listLinkHandle);

    Handle evaluationLinkHandle = atomSpace.get_handle(EVALUATION_LINK, evaluationLinkOutgoing);

    if (evaluationLinkHandle == Handle::UNDEFINED) {
        logger().warn( "PsiFeelingUpdaterAgent::%s - Failed to find the EvaluationLink holding the feling '%s' of the pet ( id = '%s' ) [ cycle = %d ].",
                        __FUNCTION__, feelingName.c_str(),
                        atomSpace.get_name(petHandle).c_str(),
                        this->cycleCount);

        return opencog::Handle::UNDEFINED;
    }

    return evaluationLinkHandle;

}

void PsiFeelingUpdaterAgent::runUpdaters()
{
    logger().debug( "PsiFeelingUpdaterAgent::%s - Running feeling updaters (scheme scripts) [ cycle = %d ]",
                    __FUNCTION__ , this->cycleCount);

#if HAVE_GUILE
    // Initialize scheme evaluator
    SchemeEval evaluator1(&_cogserver.getAtomSpace());
    std::string scheme_expression, scheme_return_value;

    // Process feelings one by one
    std::map <std::string, FeelingMeta>::iterator iFeeling;

    std::string feeling, feelingUpdater;

    for ( iFeeling = feelingMetaMap.begin();
          iFeeling != feelingMetaMap.end();
          iFeeling ++ ) {

        feeling = iFeeling->first;
        feelingUpdater = iFeeling->second.updaterName;

        scheme_expression = "( " + feelingUpdater + " )";

        // Run the Procedure that update feeling and get the updated value
        scheme_return_value = evaluator1.eval(scheme_expression);

        if ( evaluator1.eval_error() ) {
            logger().error( "PsiFeelingUpdaterAgent::%s - Failed to execute"
                            "'%s'", __FUNCTION__, scheme_expression.c_str());

            iFeeling->second.bUpdated = false;

            continue;
        }
        else {
            iFeeling->second.bUpdated = true;
        }

        // Store updated value to FeelingMeta.updatedValue
        //
        // Note: Don't use boost::lexical_cast as below, because SchemeEval will append
        //       a '\n' to the end of the result (scheme_return_value),  which will make
        //       boost::lexical_cast throw exception
        //
        // iFeeling->second.updatedValue = boost::lexical_cast<double>(scheme_return_value);
        //
        iFeeling->second.updatedValue = atof( scheme_return_value.c_str() );

        // TODO: Change the log level to fine, after testing
        logger().debug( "PsiFeelingUpdaterAgent::%s - The new level of feeling '%s' will be %f",
                         __FUNCTION__, feeling.c_str(), iFeeling->second.updatedValue);
    }// for

#endif // HAVE_GUILE

}

void PsiFeelingUpdaterAgent::setUpdatedValues()
{
    logger().debug( "PsiFeelingUpdaterAgent::%s - Setting updated feelings to AtomSpace [ cycle =%d ]",
                    __FUNCTION__, this->cycleCount);

    // Get AtomSpace
    AtomSpace& atomSpace = _cogserver.getAtomSpace();

    // Process feelings one by one
    std::map <std::string, FeelingMeta>::iterator iFeeling;
    std::string feeling;
    double updatedValue;
    Handle evaluationLink;

    for ( iFeeling = feelingMetaMap.begin();
          iFeeling != feelingMetaMap.end();
          iFeeling ++ ) {

        if ( !iFeeling->second.bUpdated )
            continue;

        feeling = iFeeling->first;
        evaluationLink = iFeeling->second.evaluationLink;
        updatedValue = iFeeling->second.updatedValue;

        // Set truth value of corresponding EvaluationLink
        TruthValuePtr stvFeeling = SimpleTruthValue::createTV(updatedValue, 1.0);
        atomSpace.set_TV(evaluationLink, stvFeeling);

        // Reset bUpdated
        iFeeling->second.bUpdated = false;

        // TODO: Change the log level to fine, after testing
        logger().debug( "PsiFeelingUpdaterAgent::%s - Set the level of feeling '%s' to %f",
                         __FUNCTION__, feeling.c_str(), updatedValue);
    }// for
}

void PsiFeelingUpdaterAgent::run()
{
    this->cycleCount ++;

    logger().debug( "PsiFeelingUpdaterAgent::%s - Executing run %d times",
                     __FUNCTION__, this->cycleCount);

    // Initialize the Agent (feelingMetaMap etc)
    if ( !this->bInitialized )
        this->init();

    // Run feeling updaters (combo scripts)
    this->runUpdaters();

    // Set updated values to AtomSpace
    this->setUpdatedValues();
}
