/*
 * @file opencog/embodiment/Control/OperationalAvatarController/StimulusUpdaterAgent.cc
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

#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/spacetime/TimeServer.h>

#include <opencog/web/json_spirit/json_spirit.h>

#include "OAC.h"
#include "StimulusUpdaterAgent.h"



using namespace opencog::oac;

bool StimulusUpdaterAgent::Stimulus::runUpdater (AtomSpace & atomSpace)
{
    std::string stimulusUpdater = this->stimulusName + "StimulusUpdater";

#if HAVE_GUILE

    // Initialize scheme evaluator
    SchemeEval & evaluator = SchemeEval::instance(&atomSpace);
    std::string scheme_expression, scheme_return_value;

    scheme_expression = "( " + stimulusUpdater + " )";

    // Run the Procedure that update Stimuluss and get the updated value
    scheme_return_value = evaluator.eval(scheme_expression);

    if ( evaluator.eval_error() ) {
        logger().error( "StimulusUpdaterAgent::Stimulus::%s - Failed to execute '%s'",
                         __FUNCTION__, scheme_expression.c_str());

        return false;
    }

    // Store the updated stimulus value (result)
    this->currentStimulusValue = atof( scheme_return_value.c_str() );

    logger().debug("StimulusUpdaterAgent::Stimulus::%s - The level of stimulus '%s' will be set to '%f'",
                   __FUNCTION__, this->stimulusName.c_str(), this->currentStimulusValue);

#endif // HAVE_GUILE

    return true;
}


void StimulusUpdaterAgent::Stimulus::initStimulus (AtomSpace & atomSpace, const unsigned long timeStamp)
{
    std::string predicateName = this->stimulusName + "Stimulus";
    Handle stimulusPredicateNode = atomSpace.addNode(PREDICATE_NODE, predicateName.c_str());
    TruthValuePtr stv = SimpleTruthValue::createTV(0.5, 1);

    std::vector<Handle> outgoings;
    Handle listLink = atomSpace.addLink(LIST_LINK, outgoings);
    outgoings.push_back(stimulusPredicateNode);
    outgoings.push_back(listLink);
    Handle evaluationLink = atomSpace.addLink(EVALUATION_LINK, outgoings);
    atomSpace.setTV(evaluationLink, stv);

//    Handle evaluationLink = AtomSpaceUtil::setPredicateValue(atomSpace, predicateName, stv);
    Handle atTimeLink = timeServer().addTimeInfo(evaluationLink, timeStamp, stv);

    AtomSpaceUtil::updateLatestStimulus(atomSpace, atTimeLink, stimulusPredicateNode);
}


bool StimulusUpdaterAgent::Stimulus::updateStimulus (AtomSpace & atomSpace, const unsigned long timeStamp)
{
    // Update LatestLink containig latest stimulus level
    std::string predicateName = this->stimulusName + "Stimulus";
    Handle stimulusPredicateNode = atomSpace.addNode(PREDICATE_NODE, predicateName.c_str());
    TruthValuePtr stv = SimpleTruthValue::createTV(this->currentStimulusValue, 1);

    std::vector <Handle> outgoings;
    Handle listLink = atomSpace.addLink(LIST_LINK, outgoings);
    outgoings.push_back(stimulusPredicateNode);
    outgoings.push_back(listLink);
    Handle evaluationLink = atomSpace.addLink(EVALUATION_LINK, outgoings);
    atomSpace.setTV(evaluationLink, stv);

//    Handle evaluationLink = AtomSpaceUtil::setPredicateValue(atomSpace, predicateName, stv);
    Handle atTimeLink = timeServer().addTimeInfo(evaluationLink, timeStamp, stv);

    AtomSpaceUtil::updateLatestStimulus(atomSpace, atTimeLink, stimulusPredicateNode);

#if HAVE_GUILE

    // TODO: implement Stimulus updaters like ModulatorUpdater
    // Initialize scheme evaluator
    //SchemeEval & evaluator = SchemeEval::instance(&atomSpace);
    //std::string scheme_expression, scheme_return_value;

    //// Store the updated Stimulus levels to AtomSpace
    //// set_stimulus_or_demand_value would create a new NumberNode and SimilarityLink
    ////
    //// Note: Since OpenCog would forget (remove) those Nodes and Links gradually,
    ////       unless you create them to be permanent, don't worry about the overflow of memory.
    //scheme_expression = "( set_modulator_or_demand_value \"" +
    //                           this->stimulusName + "Stimulus\" " +
    //                           boost::lexical_cast<std::string>(this->currentStimulusValue) + " " +
    //                           boost::lexical_cast<std::string>(timeStamp) + " " +
    //                     ")";

    //// Run the scheme procedure
    //scheme_return_value = evaluator.eval(scheme_expression);

    //if ( evaluator.eval_error() ) {
    //    logger().error( "StimulusUpdaterAgent::Stimulus::%s - Failed to execute '%s'",
    //                     __FUNCTION__,
    //                     scheme_expression.c_str()
    //                  );

    //    return false;
    //}

    //logger().debug("StimulusUpdaterAgent::Stimulus::%s - Updated the value of '%s' stimulus to %f and store it to AtomSpace",
    //               __FUNCTION__,
    //               this->stimulusName.c_str(),
    //               this->currentStimulusValue
    //              );
#endif // HAVE_GUILE

    return true;
}

StimulusUpdaterAgent::~StimulusUpdaterAgent()
{
#ifdef HAVE_ZMQ
    delete this->publisher;
#endif
}

StimulusUpdaterAgent::StimulusUpdaterAgent(CogServer& cs) : Agent(cs)
{
    this->cycleCount = 0;

#ifdef HAVE_ZMQ
    this->publisher = NULL;
#endif

    // Force the Agent initialize itself during its first cycle.
    this->forceInitNextCycle();
}

#ifdef HAVE_ZMQ
void StimulusUpdaterAgent::publishUpdatedValue(Plaza & plaza,
                                                   zmq::socket_t & publisher,
                                                   const unsigned long timeStamp)
{
    using namespace json_spirit;

    // Send the name of current mind agent which would be used as a filter key by subscribers
    std::string keyString = "StimulusUpdaterAgent";
    plaza.publishStringMore(publisher, keyString);

    // Pack time stamp and all the modulator values in json format
    Object jsonObj; // json_spirit::Object is of type std::vector< Pair >
    jsonObj.push_back( Pair("timestamp", (uint64_t) timeStamp) );

    foreach (Stimulus & stimulus, this->stimulusList) {
        jsonObj.push_back( Pair( stimulus.getStimulusName(), stimulus.getStimulusLevel() ) );
    }

    // Publish the data packed in json format
    std::string dataString = write_formatted(jsonObj);
    plaza.publishString(publisher, dataString);
}
#endif // HAVE_ZMQ

void StimulusUpdaterAgent::init()
{
    logger().debug( "StimulusUpdaterAgent::%s - Initialize the Agent [ cycle = %d ]",
                    __FUNCTION__, this->cycleCount);

    // Get OAC
    OAC * oac = dynamic_cast<OAC*>(&_cogserver);

    // Get AtomSpace
    // const AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    // Clear old stimulus list
    this->stimulusList.clear();

    // Get modulator names from the configuration file
    // then construct stimulus name like xxxStimulusUpdater
    std::string stimulusNames = config()["PSI_MODULATORS"];

    // Process Stimuluss one by one
    boost::char_separator<char> sep(", ");
    boost::tokenizer< boost::char_separator<char> > stimulusNamesTok (stimulusNames, sep);

    std::string stimulusName, stimulusUpdater;

    for ( boost::tokenizer< boost::char_separator<char> >::iterator iStimulusName = stimulusNamesTok.begin();
          iStimulusName != stimulusNamesTok.end();
          iStimulusName ++ ) {

        stimulusName = (*iStimulusName);
        stimulusUpdater = stimulusName + "StimulusUpdater";

        this->stimulusList.push_back(Stimulus(stimulusName));

        logger().debug("StimulusUpdaterAgent::%s - Store the meta data of stimulus '%s' successfully [cycle = %d]",
                        __FUNCTION__, stimulusName.c_str(), this->cycleCount);
    }// for

    // Add an additional modulator named "Pleasure".
    this->stimulusList.push_back(Stimulus("Pleasure"));

    unsigned long timeStamp = oac->getPAI().getLatestSimWorldTimestamp();

    // Init stimulus
    foreach (Stimulus & stimulus, this->stimulusList) {
        stimulus.initStimulus(oac->getAtomSpace(), timeStamp);
    }

    // Initialize ZeroMQ publisher and add it to the plaza
#ifdef HAVE_ZMQ
    // Get petId
    const std::string & petId = oac->getPet().getPetId();

    Plaza & plaza = oac->getPlaza();
    this->publisher = new zmq::socket_t (plaza.getZmqContext(), ZMQ_PUB);
    this->publishEndPoint = "ipc://" + petId + ".StimulusUpdaterAgent.ipc";
    this->publisher->bind( this->publishEndPoint.c_str() );

    plaza.addPublisher(this->publishEndPoint);
#endif

    // Avoid initialize during next cycle
    this->bInitialized = true;
}

void StimulusUpdaterAgent::run()
{
    this->cycleCount ++;

    logger().debug( "StimulusUpdaterAgent::%s - Executing run %d times",
                     __FUNCTION__, this->cycleCount);

    // Get OAC
    OAC* oac = dynamic_cast<OAC*>(&_cogserver);
    OC_ASSERT(oac, "Did not get an OAC server");

    // Get AtomSpace
    AtomSpace& atomSpace = oac->getAtomSpace();

    // Get current time stamp
    unsigned long timeStamp = oac->getPAI().getLatestSimWorldTimestamp();

    // Initialize the Agent (stimulusMetaMap etc)
    if ( !this->bInitialized )
        this->init();

    // Run stimulus updaters
    foreach (Stimulus & stimulus, this->stimulusList) {
        stimulus.runUpdater(atomSpace);
    }

    // Set the updated value to AtomSpace
    foreach (Stimulus & stimulus, this->stimulusList) {
        stimulus.updateStimulus(atomSpace, timeStamp);
    }


#ifdef HAVE_ZMQ
    // Publish updated stimulus values via ZeroMQ
    Plaza & plaza = oac->getPlaza();
    this->publishUpdatedValue(plaza, *this->publisher, timeStamp);
#endif
}

