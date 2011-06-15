/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiModulatorUpdaterAgent.cc
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2011-05-13
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
#include "PsiModulatorUpdaterAgent.h"

#include "opencog/web/json_spirit/json_spirit.h"

#include <boost/tokenizer.hpp>

using namespace OperationalAvatarController;

bool PsiModulatorUpdaterAgent::Modulator::runUpdater (AtomSpace & atomSpace)
{
    std::string modulatorUpdater = this->modulatorName + "ModulatorUpdater";

#if HAVE_GUILE    

    // Initialize scheme evaluator
    SchemeEval & evaluator = SchemeEval::instance(&atomSpace);    
    std::string scheme_expression, scheme_return_value;

    scheme_expression = "( " + modulatorUpdater + " )";

    // Run the Procedure that update Modulators and get the updated value
    scheme_return_value = evaluator.eval(scheme_expression);

    if ( evaluator.eval_error() ) {
        logger().error( "PsiModulatorUpdaterAgent::Modulator::%s - Failed to execute '%s'", 
                         __FUNCTION__, 
                         scheme_expression.c_str() 
                      );

        return false; 
    }

    // Store the updated modulator value (result)
    this->currentModulatorValue = atof( scheme_return_value.c_str() );

    logger().debug("PsiModulatorUpdaterAgent::Modulator::%s - The level of modulator '%s' will be set to '%f'", 
                   __FUNCTION__, 
                   this->modulatorName.c_str(), 
                   this->currentModulatorValue
                  );

#endif // HAVE_GUILE    

    return true; 
}    

bool PsiModulatorUpdaterAgent::Modulator::updateModulator (AtomSpace & atomSpace, const unsigned long timeStamp)
{
#if HAVE_GUILE    

    // Initialize scheme evaluator
    SchemeEval & evaluator = SchemeEval::instance(&atomSpace);    
    std::string scheme_expression, scheme_return_value;

    // Store the updated Modulator levels to AtomSpace
    // set_modulator_or_demand_value would create a new NumberNode and SilarityLink 
    //
    // Note: Since OpenCog would forget (remove) those Nodes and Links gradually, 
    //       unless you create them to be permanent, don't worry about the overflow of memory. 
    scheme_expression = "( set_modulator_or_demand_value \"" + 
                               this->modulatorName + "Modulator\" " +
                               boost::lexical_cast<std::string>(this->currentModulatorValue) + " " + 
                               boost::lexical_cast<std::string>(timeStamp) + " " +
                         ")";   

    // Run the scheme procedure
    scheme_return_value = evaluator.eval(scheme_expression);

    if ( evaluator.eval_error() ) {
        logger().error( "PsiModulatorUpdaterAgent::Modulator::%s - Failed to execute '%s'", 
                         __FUNCTION__, 
                         scheme_expression.c_str() 
                      );

        return false; 
    }

    logger().debug("PsiModulatorUpdaterAgent::Modulator::%s - Updated the value of '%s' modulator to %f and store it to AtomSpace", 
                   __FUNCTION__, 
                   this->modulatorName.c_str(), 
                   this->currentModulatorValue
                  );

    return true; 

#endif // HAVE_GUILE    
}    

PsiModulatorUpdaterAgent::~PsiModulatorUpdaterAgent()
{
#ifdef HAVE_ZMQ
    delete this->publisher; 
#endif
}

PsiModulatorUpdaterAgent::PsiModulatorUpdaterAgent()
{
    this->cycleCount = 0;
    this->publisher = NULL; 

    // Force the Agent initialize itself during its first cycle. 
    this->forceInitNextCycle();
}

#ifdef HAVE_ZMQ
void PsiModulatorUpdaterAgent::publishUpdatedValue(Plaza & plaza, 
                                                   zmq::socket_t & publisher, 
                                                   const unsigned long timeStamp)
{
    using namespace json_spirit; 

    // Send the name of current mind agent which would be used as a filter key by subscribers
    std::string keyString = "PsiModulatorUpdaterAgent"; 
    plaza.publishStringMore(publisher, keyString); 

    // Pack time stamp and all the modulator values in json format 
    Object jsonObj; // json_spirit::Object is of type std::vector< Pair >
    jsonObj.push_back( Pair("timestamp", (uint64_t) timeStamp) );

    foreach (Modulator & modulator, this->modulatorList) {
        jsonObj.push_back( Pair( modulator.getModulatorName(), modulator.getModulatorLevel() ) );
    }

    // Publish the data packed in json format
    std::string dataString = write_formatted(jsonObj);
    plaza.publishString(publisher, dataString);
}
#endif // HAVE_ZMQ

void PsiModulatorUpdaterAgent::init(opencog::CogServer * server) 
{
    logger().debug( "PsiModulatorUpdaterAgent::%s - Initialize the Agent [ cycle = %d ]",
                    __FUNCTION__, 
                    this->cycleCount
                  );

    // Get OAC
    OAC * oac = (OAC *) server;

    // Get AtomSpace
    const AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    // Get petId
    const std::string & petId = oac->getPet().getPetId();

    // Clear old modulator list
    this->modulatorList.clear();

    // Get modulator names from the configuration file
    std::string modulatorNames = config()["PSI_MODULATORS"];

    // Process Modulators one by one
    boost::char_separator<char> sep(", ");
    boost::tokenizer< boost::char_separator<char> > modulatorNamesTok (modulatorNames, sep);

    std::string modulatorName, modulatorUpdater;
    Handle hUpdater, hGroundedSchemaNode;

    for ( boost::tokenizer< boost::char_separator<char> >::iterator iModulatorName = modulatorNamesTok.begin();
          iModulatorName != modulatorNamesTok.end();
          iModulatorName ++ ) {

        modulatorName = (*iModulatorName);
        modulatorUpdater = modulatorName + "ModulatorUpdater";
        
        this->modulatorList.push_back(Modulator(modulatorName));

        logger().debug("PsiModulatorUpdaterAgent::%s - Store the meta data of modulator '%s' successfully [cycle = %d]", 
                        __FUNCTION__, 
                        modulatorName.c_str(), 
                        this->cycleCount
                      );
    }// for

    // Initialize ZeroMQ publisher and add it to the plaza
#ifdef HAVE_ZMQ
    Plaza & plaza = oac->getPlaza();
    this->publisher = new zmq::socket_t (plaza.getZmqContext(), ZMQ_PUB);
    this->publishEndPoint = "ipc://" + petId + ".PsiModulatorUpdaterAgent.ipc"; 
    this->publisher->bind( this->publishEndPoint.c_str() );

    plaza.addPublisher(this->publishEndPoint); 
#endif    

    // Avoid initialize during next cycle
    this->bInitialized = true;
}

void PsiModulatorUpdaterAgent::run(opencog::CogServer * server)
{
    this->cycleCount ++;

    logger().debug( "PsiModulatorUpdaterAgent::%s - Executing run %d times",
                     __FUNCTION__, 
                     this->cycleCount
                  );

    // Get OAC
    OAC * oac = (OAC *) server;

    // Get AtomSpace
    AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    // Get petId
    const std::string & petId = oac->getPet().getPetId();

    // Get current time stamp
    unsigned long timeStamp = oac->getPAI().getLatestSimWorldTimestamp();   

    // Check if map info data is available
    if ( atomSpace.getSpaceServer().getLatestMapHandle() == Handle::UNDEFINED ) {
        logger().warn( 
      "PsiModulatorUpdaterAgent::%s - There is no map info available yet [ cycle = %d ]", 
                        __FUNCTION__, 
                        this->cycleCount
                     );
        return;
    }

    // Check if the pet spatial info is already received
    if ( !atomSpace.getSpaceServer().getLatestMap().containsObject(petId) ) {
        logger().warn(
 "PsiModulatorUpdaterAgent::%s - Pet was not inserted in the space map yet [ cycle = %d ]", 
                     __FUNCTION__, 
                     this->cycleCount
                     );
        return;
    }

    // Initialize the Agent (modulatorMetaMap etc)
    if ( !this->bInitialized )
        this->init(server);

    // Run modulator updaters
    foreach (Modulator & modulator, this->modulatorList) {
        modulator.runUpdater(atomSpace);
    }

    // Set the updated value to AtomSpace
    foreach (Modulator & modulator, this->modulatorList) {
        modulator.updateModulator(atomSpace, timeStamp);
    }

#ifdef HAVE_ZMQ    
    // Publish updated modulator values via ZeroMQ
    Plaza & plaza = oac->getPlaza();
    this->publishUpdatedValue(plaza, *this->publisher, timeStamp); 
#endif 
}

