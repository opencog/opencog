/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiModulatorUpdaterAgent.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2011-03-11
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

#include<boost/tokenizer.hpp>

using namespace OperationalAvatarController;

bool PsiModulatorUpdaterAgent::Modulator::runUpdater
    (const AtomSpace & atomSpace, 
     Procedure::ProcedureInterpreter & procedureInterpreter, 
     const Procedure::ProcedureRepository & procedureRepository
    )
{
    // Get the GroundedPredicateNode
    Handle hGroundedSchemaNode = atomSpace.getOutgoing(this->hUpdater, 0);

    if ( hGroundedSchemaNode == opencog::Handle::UNDEFINED ||
         atomSpace.getType(hGroundedSchemaNode) != GROUNDED_SCHEMA_NODE ) {

        logger().error("PsiModulatorUpdaterAgent::Modulator::%s - Expect a GroundedPredicateNode for modulator '%s'. But got '%s'", 
                       __FUNCTION__, 
                       this->modulatorName.c_str(), 
                       atomSpace.atomAsString(hGroundedSchemaNode).c_str()
                      );
        return false; 

    }

    // Run the Procedure that update Modulator and get the updated value
    const std::string & updaterName = atomSpace.getName(hGroundedSchemaNode);
    std::vector <combo::vertex> schemaArguments;
    Procedure::RunningProcedureID executingSchemaId;
    combo::vertex result; // combo::vertex is actually of type boost::variant <...>

    const Procedure::GeneralProcedure & procedure =
                                        procedureRepository.get(updaterName);

    executingSchemaId = procedureInterpreter.runProcedure(procedure, schemaArguments);

    // Wait until the procedure is done
    while ( !procedureInterpreter.isFinished(executingSchemaId) )
        procedureInterpreter.run(NULL);  

    // Check if the the updater run successfully
    if ( procedureInterpreter.isFailed(executingSchemaId) ) {
        logger().error( "PsiModulatorUpdaterAgent::Modulator::%s - Failed to execute '%s' for updating modulator '%s'",
                         __FUNCTION__, 
                         updaterName.c_str(), 
                         this->modulatorName.c_str()
                      );

        return false;
    }

    // Store the updated modulator value (result)
    result = procedureInterpreter.getResult(executingSchemaId);
    this->currentModulatorValue = get_contin(result);

    logger().debug("PsiModulatorUpdaterAgent::Modulator::%s - The level of modulator '%s' will be set to '%f'", 
                   __FUNCTION__, 
                   this->modulatorName.c_str(), 
                   this->currentModulatorValue
                  );


    return true; 
}    

bool PsiModulatorUpdaterAgent::Modulator::updateModulator
    (AtomSpace & atomSpace, 
     Procedure::ProcedureInterpreter & procedureInterpreter,
     const Procedure::ProcedureRepository & procedureRepository, 
     const unsigned long timeStamp
    )
{
    // Create a new NumberNode and SilarityLink to store the result
    //
    // Note: Since OpenCog would forget (remove) those Nodes and Links gradually, 
    //       unless you create them to be permanent, don't worry about the overflow of memory. 

    // Create a new NumberNode that stores the updated value
    // If the Modulator doesn't change at all, it actually returns the old one
    Handle hNewNumberNode = AtomSpaceUtil::addNode( atomSpace,
                                                    NUMBER_NODE,
                                                    boost::lexical_cast<std::string>
                                                           (this->currentModulatorValue),
                                                    false // the atom should not be permanent (can be
                                                          // removed by decay importance task) 
                                                  );

    // Create a new SimilarityLink that holds the Modulator updater and NumberNode
    // If the Demand doesn't change at all, it actually returns the old one
    //
    // TODO: set the truth value and importance of SimilarityLink
    std::vector<Handle> similarityLinkOutgoing;

    similarityLinkOutgoing.push_back(hNewNumberNode);
    similarityLinkOutgoing.push_back(this->hUpdater);

    Handle hNewSimilarityLink = AtomSpaceUtil::addLink( atomSpace,
                                                        SIMILARITY_LINK, 
                                                        similarityLinkOutgoing,
                                                        false // the atom should not be permanent (can be
                                                              // removed by decay importance task)
                                                      );
  
    // Time stamp the SimilarityLink.
    Handle hAtTimeLink = atomSpace.getTimeServer().addTimeInfo(hNewSimilarityLink, timeStamp);

    logger().debug("PsiModulatorUpdaterAgent::Modulator::%s - Updated the value of '%s' modulator to %f and store it to AtomSpace as '%s'", 
                   __FUNCTION__, 
                   this->modulatorName.c_str(), 
                   this->currentModulatorValue, 
                   atomSpace.atomAsString(hNewSimilarityLink).c_str()
                  );

    return true; 
}    

PsiModulatorUpdaterAgent::~PsiModulatorUpdaterAgent()
{

}

PsiModulatorUpdaterAgent::PsiModulatorUpdaterAgent()
{
    this->cycleCount = 0;

    // Force the Agent initialize itself during its first cycle. 
    this->forceInitNextCycle();
}

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

    // Get Procedure repository
    const Procedure::ProcedureRepository & procedureRepository = 
                                               oac->getProcedureRepository();

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
        
        // Search modulator updater
        if ( !procedureRepository.contains(modulatorUpdater) ) {
            logger().warn( "PsiModulatorUpdaterAgent::%s - Failed to find '%s' in OAC's procedureRepository",
                           __FUNCTION__, 
                           modulatorUpdater.c_str()
                         );
            continue;
        }

        // Get the GroundedSchemaNode
        hGroundedSchemaNode = atomSpace.getHandle(GROUNDED_SCHEMA_NODE, modulatorUpdater); 

        if ( hGroundedSchemaNode == opencog::Handle::UNDEFINED ) {
            logger().warn( "PsiModulatorUpdaterAgent::%s - Failed to get the GroundedPredicateNode for updater '%s'", 
                           __FUNCTION__, 
                           modulatorUpdater.c_str()
                         );
            continue;
        }

        // Get the updater (ExecutionOutputLink)
        std::vector<Handle> executionOutputLinkSet;

        atomSpace.getHandleSet( back_inserter(executionOutputLinkSet), 
                                hGroundedSchemaNode,
                                EXECUTION_OUTPUT_LINK, 
                                false
                              );

        if ( executionOutputLinkSet.size() != 1 ) {
            logger().warn( "PsiModulatorUpdaterAgent::%s - The number of ExecutionOutputLink containing updater (ExecutionOutputLink) should be exactly 1. But got %d", 
                           __FUNCTION__, 
                           executionOutputLinkSet.size()
                         );
            continue;
        }

        hUpdater = executionOutputLinkSet[0];

        // Save the updater
        this->modulatorList.push_back( Modulator(modulatorName, hUpdater) );

        logger().debug("PsiModulatorUpdaterAgent::%s - Store the meta data of modulator '%s' successfully.", 
                        __FUNCTION__, 
                        modulatorName.c_str() 
                      );
    }// for

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

    // Get ProcedureInterpreter
    Procedure::ProcedureInterpreter & procedureInterpreter = oac->getProcedureInterpreter();

    // Get Procedure repository
    const Procedure::ProcedureRepository & procedureRepository = oac->getProcedureRepository();

    // Get current time stamp
    unsigned long timeStamp = atomSpace.getTimeServer().getLatestTimestamp();
   
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
    foreach (Modulator modulator, this->modulatorList) {
        modulator.runUpdater(atomSpace, procedureInterpreter, procedureRepository);
    }

    // Set the updated value to AtomSpace
    foreach (Modulator modulator, this->modulatorList) {
        modulator.updateModulator(atomSpace, procedureInterpreter, procedureRepository, timeStamp);
    }
}

