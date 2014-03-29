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

#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/guile/SchemeEval.h>
#include <lib/json_spirit/json_spirit.h>

#include "OAC.h"
#include "PsiFeelingUpdaterAgent.h"



using namespace opencog::oac;

PsiFeelingUpdaterAgent::~PsiFeelingUpdaterAgent()
{
#ifdef HAVE_ZMQ
    delete this->publisher;
#endif
}

PsiFeelingUpdaterAgent::PsiFeelingUpdaterAgent(CogServer& cs) : Agent(cs)
{
    this->cycleCount = 0;

#ifdef HAVE_ZMQ
    this->publisher = NULL;
#endif

    // Force the Agent initialize itself during its first cycle.
    this->forceInitNextCycle();
}

#ifdef HAVE_ZMQ
void PsiFeelingUpdaterAgent::publishUpdatedValue(Plaza & plaza,
                                                 zmq::socket_t & publisher,
                                                 const unsigned long timeStamp)
{
    using namespace json_spirit;

    // Send the name of current mind agent which would be used as a filter key by subscribers
    std::string keyString = "PsiFeelingUpdaterAgent";
    plaza.publishStringMore(publisher, keyString);

    // Pack time stamp and all the feeling values in json format
    Object jsonObj; // json_spirit::Object is of type std::vector< Pair >
    jsonObj.push_back( Pair("timestamp", (uint64_t) timeStamp) );

    std::map <std::string, FeelingMeta>::iterator iFeeling;
    std::string feeling;
    double updatedValue;

    for ( iFeeling = feelingMetaMap.begin();
          iFeeling != feelingMetaMap.end();
          iFeeling ++ ) {

        feeling = iFeeling->first;
        updatedValue = iFeeling->second.updatedValue;

        jsonObj.push_back( Pair(feeling, updatedValue) );

    }// for

    // Publish the data packed in json format
    std::string dataString = write_formatted(jsonObj);
    plaza.publishString(publisher, dataString);
}
#endif // HAVE_ZMQ

void PsiFeelingUpdaterAgent::init()
{
    logger().debug( "PsiFeelingUpdaterAgent::%s - Initializing the Agent [ cycle = %d ]",
                    __FUNCTION__, this->cycleCount);

    // Get OAC
    OAC* oac = dynamic_cast<OAC*>(&_cogserver);
    OC_ASSERT(oac, "Did not get an OAC server");

    // Get AtomSpace
    AtomSpace& atomSpace = oac->getAtomSpace();

    // Get petId
    const std::string & petId = oac->getPet().getPetId();

    // Get petHandle
    Handle petHandle = AtomSpaceUtil::getAgentHandle(atomSpace, petId);

    if ( petHandle == Handle::UNDEFINED ) {
        logger().warn("PsiFeelingUpdaterAgent::%s - Failed to get the handle to the pet ( id = '%s' ) [ cycle = %d ]",
                        __FUNCTION__, petId.c_str(), this->cycleCount);
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

        logger().debug( "PsiFeelingUpdaterAgent::%s - Searching the meta data of feeling '%s'.",
                        __FUNCTION__, feeling.c_str());

        // Get the corresponding EvaluationLink of the pet's feeling
        Handle evaluationLink = this->getFeelingEvaluationLink(&_cogserver, feeling, petHandle);

        if ( evaluationLink == Handle::UNDEFINED )
        {
            logger().warn( "PsiFeelingUpdaterAgent::%s - Failed to get the EvaluationLink for feeling '%s'",
                           __FUNCTION__, feeling.c_str());

            continue;
        }

        // Insert the meta data of the feeling to feelingMetaMap
        feelingMeta.init(feelingUpdater, evaluationLink);
        feelingMetaMap[feeling] = feelingMeta;

        logger().debug( "PsiFeelingUpdaterAgent::%s - Store the meta data of feeling '%s' successfully.",
                        __FUNCTION__, feeling.c_str());
    }// for

    // Initialize ZeroMQ publisher and add it to the plaza
#ifdef HAVE_ZMQ
    Plaza & plaza = oac->getPlaza();
    this->publisher = new zmq::socket_t (plaza.getZmqContext(), ZMQ_PUB);
    this->publishEndPoint = "ipc://" + petId + ".PsiFeelingUpdaterAgent.ipc";
    this->publisher->bind( this->publishEndPoint.c_str() );

    plaza.addPublisher(this->publishEndPoint);
#endif

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
    Handle feelingPredicateHandle = atomSpace.getHandle(PREDICATE_NODE, feelingName);

    if (feelingPredicateHandle == Handle::UNDEFINED) {
        logger().warn("PsiFeelingUpdaterAgent::%s - Failed to find the PredicateNode for feeling '%s' [ cycle = %d ].",
                       __FUNCTION__, feelingName.c_str(), this->cycleCount);

        return opencog::Handle::UNDEFINED;
    }

    // Get the Handle to ListLink that contains the pet handle
    std::vector<Handle> listLinkOutgoing;

    listLinkOutgoing.push_back(petHandle);

    Handle listLinkHandle = atomSpace.getHandle(LIST_LINK, listLinkOutgoing);

    if (listLinkHandle == Handle::UNDEFINED) {
        logger().warn( "PsiFeelingUpdaterAgent::%s - Failed to find the ListLink containing the pet ( id = '%s' ) [ cycle = %d ].",
                        __FUNCTION__, atomSpace.getName(petHandle).c_str(), this->cycleCount);

        return opencog::Handle::UNDEFINED;
    }

    // Get the Handle to EvaluationLink holding the pet's feeling
    std::vector<Handle> evaluationLinkOutgoing;

    evaluationLinkOutgoing.push_back(feelingPredicateHandle);
    evaluationLinkOutgoing.push_back(listLinkHandle);

    Handle evaluationLinkHandle = atomSpace.getHandle(EVALUATION_LINK, evaluationLinkOutgoing);

    if (evaluationLinkHandle == Handle::UNDEFINED) {
        logger().warn( "PsiFeelingUpdaterAgent::%s - Failed to find the EvaluationLink holding the feling '%s' of the pet ( id = '%s' ) [ cycle = %d ].",
                        __FUNCTION__, feelingName.c_str(),
                        atomSpace.getName(petHandle).c_str(),
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
    // Get OAC
    OAC* oac = dynamic_cast<OAC*>(&_cogserver);
    OC_ASSERT(oac, "Did not get an OAC server");

    // Get AtomSpace
    AtomSpace& atomSpace = oac->getAtomSpace();


    // Initialize scheme evaluator
    SchemeEval* evaluator = new SchemeEval();
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
        scheme_return_value = evaluator->eval(scheme_expression);

        if ( evaluator->eval_error() ) {
            logger().error( "PsiFeelingUpdaterAgent::%s - Failed to execute '%s'",
                             __FUNCTION__, scheme_expression.c_str());

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

    delete evaluator;
#endif // HAVE_GUILE

}

void PsiFeelingUpdaterAgent::setUpdatedValues()
{
    logger().debug( "PsiFeelingUpdaterAgent::%s - Setting updated feelings to AtomSpace [ cycle =%d ]",
                    __FUNCTION__, this->cycleCount);

    // Get OAC
    OAC* oac = dynamic_cast<OAC*>(&_cogserver);
    OC_ASSERT(oac, "Did not get an OAC server");

    // Get AtomSpace
    AtomSpace& atomSpace = oac->getAtomSpace();

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
        atomSpace.setTV(evaluationLink, stvFeeling);

        // Reset bUpdated
        iFeeling->second.bUpdated = false;

        // TODO: Change the log level to fine, after testing
        logger().debug( "PsiFeelingUpdaterAgent::%s - Set the level of feeling '%s' to %f",
                         __FUNCTION__, feeling.c_str(), updatedValue);
    }// for
}

void PsiFeelingUpdaterAgent::sendUpdatedValues()
{
    logger().debug( "PsiFeelingUpdaterAgent::%s - Sending updated feelings to the virtual world where the pet lives [ cycle =%d ]",
                    __FUNCTION__, this->cycleCount);

    // Get OAC
    OAC * oac = dynamic_cast<OAC *>(&_cogserver);

    // Get AtomSpace
//    AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    // Get petName
    const std::string & petName = oac->getPet().getName();

    // Prepare the data to be sent
    std::map <std::string, FeelingMeta>::iterator iFeeling;
    std::map <std::string, float> feelingValueMap;
    std::string feeling;
    double updatedValue;

    for ( iFeeling = feelingMetaMap.begin();
          iFeeling != feelingMetaMap.end();
          iFeeling ++ ) {

        feeling = iFeeling->first;
        updatedValue = iFeeling->second.updatedValue;

        feelingValueMap[feeling] = updatedValue;

    }// for

    // Send updated feelings to the virtual world where the pet lives
    oac->getPAI().sendEmotionalFeelings(petName, feelingValueMap);
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

    // Send updated values to the virtual world where the pet lives
    this->sendUpdatedValues();

#ifdef HAVE_ZMQ
    // Publish updated modulator values via ZeroMQ
    OAC * oac = dynamic_cast<OAC *>(&_cogserver);
    unsigned long timeStamp = oac->getPAI().getLatestSimWorldTimestamp();

    Plaza & plaza = oac->getPlaza();
    this->publishUpdatedValue(plaza, *this->publisher, timeStamp);
#endif
}

