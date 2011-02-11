/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiModulatorUpdaterAgent.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2010-12-06
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

    // Clear old modulatorMetaMap; 
    this->modulatorMetaMap.clear();

    // Get modulator names from the configuration file
    std::string modulatorNames = config()["PSI_MODULATORS"];

    // Process Modulators one by one
    boost::tokenizer<> modulatorNamesTok (modulatorNames);
    std::string modulator, modulatorUpdater;
    Handle similarityLink;
    ModulatorMeta modulatorMeta;

    for ( boost::tokenizer<>::iterator iModulatorName = modulatorNamesTok.begin();
          iModulatorName != modulatorNamesTok.end();
          iModulatorName ++ ) {

        modulator = (*iModulatorName);
        modulatorUpdater = modulator + "ModulatorUpdater";

        logger().debug(
              "PsiModulatorUpdaterAgent::%s - Searching the meta data of modulator '%s'.", 
                        __FUNCTION__, 
                        modulator.c_str() 
                      );

        // Search modulator updater
        if ( !procedureRepository.contains(modulatorUpdater) ) {
            logger().warn( 
       "PsiModulatorUpdaterAgent::%s - Failed to find '%s' in OAC's procedureRepository",
                           __FUNCTION__, 
                           modulatorUpdater.c_str()
                         );
            continue;
        }
    
        // Search the corresponding SimilarityLink
        similarityLink =  AtomSpaceUtil::getModulatorSimilarityLink( atomSpace, 
                                                                     modulator,
                                                                     petId
                                                                   );

        if ( similarityLink == Handle::UNDEFINED )
        {
            logger().warn(
    "PsiModulatorUpdaterAgent::%s - Failed to get the SimilarityLink for modulator '%s'",
                           __FUNCTION__, 
                           modulator.c_str()
                         );

            continue;
        }

        // Insert the meta data of the Modulator to modulatorMetaMap
        modulatorMeta.init(modulatorUpdater, similarityLink);
        modulatorMetaMap[modulator] = modulatorMeta;

        logger().debug(
     "PsiModulatorUpdaterAgent::%s - Store the meta data of  modulator '%s' successfully.", 
                        __FUNCTION__, 
                        modulator.c_str() 
                      );
    }// for

    // Avoid initialize during next cycle
    this->bInitialized = true;
}

void PsiModulatorUpdaterAgent::runUpdaters(opencog::CogServer * server)
{
    logger().debug( 
            "PsiModulatorUpdaterAgent::%s - Run updaters (combo scripts) [ cycle = %d ]", 
                    __FUNCTION__ , 
                    this->cycleCount
                  );

    // Get OAC
    OAC * oac = (OAC *) server;

    // Get ProcedureInterpreter
    Procedure::ProcedureInterpreter & procedureInterpreter = oac->getProcedureInterpreter();

    // Get Procedure repository
    const Procedure::ProcedureRepository & procedureRepository =
                                                              oac->getProcedureRepository();

    // Process Modulators one by one
    std::map <std::string, ModulatorMeta>::iterator iModulator;

    std::string modulator, modulatorUpdater;
    std::vector <combo::vertex> schemaArguments;
    Procedure::RunningProcedureID executingSchemaId;
    combo::vertex result; // combo::vertex is actually of type boost::variant <...>

    for ( iModulator = modulatorMetaMap.begin();
          iModulator != modulatorMetaMap.end();
          iModulator ++ ) {

        modulator = iModulator->first;
        modulatorUpdater = iModulator->second.updaterName;

        // Run the Procedure that update Modulator and get the updated value
        const Procedure::GeneralProcedure & procedure =
                                            procedureRepository.get(modulatorUpdater);

        executingSchemaId = procedureInterpreter.runProcedure(procedure, schemaArguments);

        // TODO: What does this for?
        while ( !procedureInterpreter.isFinished(executingSchemaId) )
            procedureInterpreter.run(NULL);  

        // Check if the the updater run successfully
        if ( procedureInterpreter.isFailed(executingSchemaId) ) {
            logger().error( "PsiModulatorUpdaterAgent::%s - Failed to execute '%s'", 
                             __FUNCTION__, 
                             modulatorUpdater.c_str() 
                          );

            iModulator->second.bUpdated = false;

            continue;
        }
        else {
            iModulator->second.bUpdated = true;
        }

        result = procedureInterpreter.getResult(executingSchemaId);

        // Store updated value to ModulatorMeta.updatedValue
        // contin_t is actually of type double (see "comboreduct/combo/vertex.h") 
        iModulator->second.updatedValue = get_contin(result);

        // TODO: Change the log level to fine, after testing
        logger().debug( "PsiModulatorUpdaterAgent::%s - The new level of '%s' will be %f", 
                         __FUNCTION__, 
                         modulator.c_str(),
                         iModulator->second.updatedValue                      
                      );
    }// for

}    

void PsiModulatorUpdaterAgent::setUpdatedValues(opencog::CogServer * server)
{
    logger().debug(
            "PsiModulatorUpdaterAgent::%s - Set updated values to AtomSpace [ cycle =%d ]",
                    __FUNCTION__, 
                    this->cycleCount
                  );

    // Get OAC
    OAC * oac = (OAC *) server;

    // Get AtomSpace
    AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    // Process Modulators one by one
    std::map <std::string, ModulatorMeta>::iterator iModulator;

    std::string modulator;
    Handle oldSimilarityLink, oldNumberNode, oldExecutionOutputLink;
    Handle newNumberNode, newSimilarityLink;
    double updatedValue;

    for ( iModulator = modulatorMetaMap.begin();
          iModulator != modulatorMetaMap.end();
          iModulator ++ ) {

        if ( !iModulator->second.bUpdated )
            continue;

        modulator = iModulator->first;
        oldSimilarityLink = iModulator->second.similarityLink;
        updatedValue = iModulator->second.updatedValue;
        
        // Get the Handle to old NumberNode and ExecutionOutputLink
        //
        // Since SimilarityLink inherits from UnorderedLink, you should check each Atom in 
        // the Outgoing set to see if it is of type NumberNode or ExecutionOutputLink
        //
        // Because when you creating an UnorderedLink, it will sort its Outgoing set
        // automatically (more detail: "./atomspace/Link.cc", Link::setOutgoingSet method)
        //
        
        oldNumberNode = atomSpace.getOutgoing(oldSimilarityLink, 0);
        oldExecutionOutputLink = atomSpace.getOutgoing(oldSimilarityLink, 1); 
        
        if ( atomSpace.getType(oldNumberNode) != NUMBER_NODE ||
             atomSpace.getType(oldExecutionOutputLink) != EXECUTION_OUTPUT_LINK
           ) {
            
            oldNumberNode = atomSpace.getOutgoing(oldSimilarityLink, 1);   
            oldExecutionOutputLink = atomSpace.getOutgoing(oldSimilarityLink, 0); 

            if ( atomSpace.getType(oldNumberNode) != NUMBER_NODE ||
                 atomSpace.getType(oldExecutionOutputLink) != EXECUTION_OUTPUT_LINK
               ) {

                logger().error( "PsiModulatorUpdaterAgent::%s - The outgoing set of SimilarityLink for '%s' should contain a NumberNode and a ExecutionOutputLink, but got [0]:%s, [1]:%s",
                                __FUNCTION__, 
                                modulator.c_str(), 
                                classserver().getTypeName(
                                                  atomSpace.getType(oldSimilarityLink)
                                                         ).c_str(),  
                                classserver().getTypeName(
                                                  atomSpace.getType(oldNumberNode)
                                                         ).c_str()
                              );

                continue;
            }
        }// if

logger().fine( "PsiModulatorUpdaterAgent::%s - Modulator: %s, oldSimilarityLink: %s, oldNumberNode: %s, updatedValue: %f", 
               __FUNCTION__,
               modulator.c_str(), 
               atomSpace.atomAsString(oldSimilarityLink).c_str(),
               atomSpace.atomAsString(oldNumberNode).c_str(), 
               updatedValue
             );        

        // Create a new NumberNode that stores the updated value
        //
        // If the Modulator doesn't change at all, it actually returns the old one
        //
        newNumberNode = AtomSpaceUtil::addNode( atomSpace,
                                                NUMBER_NODE,
                                                boost::lexical_cast<std::string>
                                                                   (updatedValue), 
                                                false // the atom should not be permanent (can be
                                                      // removed by decay importance task) 
                                               );

logger().fine( "PsiModulatorUpdaterAgent::%s - newNumberNode: %s", 
               __FUNCTION__, 
               atomSpace.atomAsString(newNumberNode).c_str()
             );        

        // Create a new SimilarityLink that holds the Modulator
        //
        // Since SimilarityLink inherits from UnorderedLink, which will sort its Outgoing 
        // set automatically when being created, the sequence you adding Atoms to its 
        // Outgoing set makes none sense! 
        // (more detail: "./atomspace/Link.cc", Link::setOutgoingSet method)
        //
        // If the Modulator doesn't change at all, it actually returns the old one
        //
        HandleSeq similarityLinkHandleSeq;  // HandleSeq is of type std::vector<Handle>

        similarityLinkHandleSeq.push_back(newNumberNode);
        similarityLinkHandleSeq.push_back(oldExecutionOutputLink);

        newSimilarityLink = AtomSpaceUtil::addLink( atomSpace,
                                                    SIMILARITY_LINK, 
                                                    similarityLinkHandleSeq,
                                                    true
                                                  );

        iModulator->second.similarityLink = newSimilarityLink;

logger().fine( "PsiModulatorUpdaterAgent::%s - newSimilarityLink: %s", 
               __FUNCTION__, 
               atomSpace.atomAsString(newSimilarityLink).c_str()
             );        

        // Remove the old SimilarityLink and NumberNode
        //
        // Make sure the old one is different from the new one before removing.
        // Because if the Modulator does change at all, including its value True Value etc,
        // creating new one is actually returning the old one, then we shall not remove it.
        //

logger().fine( "PsiModulatorUpdaterAgent::%s - Going to remove oldSimilarityLink", 
                __FUNCTION__);

        if ( oldSimilarityLink != newSimilarityLink && 
             !atomSpace.removeAtom(oldSimilarityLink) 
           ) {
            logger().error(
                "PsiModulatorUpdaterAgent::%s - Unable to remove old SIMILARITY_LINK: %s",
                __FUNCTION__, 
                atomSpace.atomAsString(oldSimilarityLink).c_str()
                          );
        }// if

logger().fine("PsiModulatorUpdaterAgent::%s - Removed oldSimilarityLink", __FUNCTION__);

//logger().fine("PsiModulatorUpdaterAgent::%s - Going to remove oldNumberNode", __FUNCTION__);
//
//        if ( oldNumberNode != newNumberNode && 
//             !atomSpace.removeAtom(oldNumberNode) 
//           ) {
//            logger().error(
//                "PsiModulatorUpdaterAgent::%s - Unable to remove old NUMBER_NODE: %s",
//                __FUNCTION__, 
//                atomSpace.atomAsString(oldNumberNode).c_str()
//                          );
//        }// if
//
//logger().fine( "PsiModulatorUpdaterAgent::%s - Removed oldNumberNode", __FUNCTION__);              
        // Reset bUpdated  
        iModulator->second.bUpdated = false;

        // TODO: Change the log level to fine, after testing
        logger().debug( 
                    "PsiModulatorUpdaterAgent::%s - Set the level of modulator '%s' to %f", 
                         __FUNCTION__, 
                         modulator.c_str(),
                         updatedValue
                      );

    }// for

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

    // Run updaters (combo scripts)
    this->runUpdaters(server);

    // Set updated values to AtomSpace (NumberNodes)
    this->setUpdatedValues(server);
}

