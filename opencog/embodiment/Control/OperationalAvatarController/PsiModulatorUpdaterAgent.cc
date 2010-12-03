/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiModulatorUpdaterAgent.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2010-12-04
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
#include "ModulatorUpdaterAgent.h"

#include<boost/tokenizer.hpp>

using namespace OperationalAvatarController;

PsiModulatorUpdaterAgent::~PsiModulatorUpdaterAgent()
{

}

PsiModulatorUpdaterAgent::PsiModulatorUpdaterAgent()
{
    lastTickTime = 0;

    // Force the Agent initialize itself during its first cycle. 
    this->forceInitNextCycle();
}

void PsiModulatorUpdaterAgent::init(opencog::CogServer * server) 
{
    logger().debug("PsiModulatorUpdaterAgent::init - Initialize the Agent");

    // Get OPC
    OAC * oac = (OAC *) server;

    // Get AtomSpace
    const AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    // Get Procedure repository
    const Procedure::ProcedureRepository & procedureRepository = oac->getProcedureRepository();

    // Get petId
    const std::string & petId = oac->getPet().getPetId();

    // Clear old modulatorMetaMap; 
    this->modulatorMetaMap.clear();

    // Get modulator names from the configuration file
    std::string modulatorNames = config()["MODULATORS"];

    // Process Modulators one by one
    boost::tokenizer<> modulatorNamesTok (modulatorNames);
    std::string modulator, modulatorUpdater;
    Handle similarityLink, numberNode;
    ModulatorMeta modulatorMeta;

    for ( boost::tokenizer<>::iterator iModulatorName = modulatorNamesTok.begin();
          iModulatorName != modulatorNamesTok.end();
          iModulatorName ++ ) {

        modulator = (*iModulatorName);
        modulatorUpdater = modulator + "ModulatorUpdater";

        // Search modulator updater
        if ( !procedureRepository.contains(modulatorUpdater) ) {
            logger().warn( 
       "PsiModulatorUpdaterAgent::init - Failed to find '%s' in OPC's procedureRepository",
                           modulatorUpdater.c_str()
                         );
            continue;
        }
    
        // Search the corresponding SimilarityLink
        similarityLink =  AtomSpaceUtil::getModulatorSimilarityLink
                                                  ( atomSpace, 
                                                    modulator,
                                                    petId
                                                  );

        if ( similarityLink == Handle::UNDEFINED )
        {
            logger().warn(
    "PsiModulatorUpdaterAgent::init - Failed to get the SimilarityLink for modulator '%s'",
                      modulator.c_str()
                         );

            continue;
        }

        // Get NumberNode that stores the value of the Modulator
        //
        // We don't check if the type of the Atom returned is exactly NumberNode.
        // Because it has been done by AtomSpaceUtil::getModulatorSimilarityLink method.
        numberNode = atomSpace.getOutgoing(similarityLink, 0);

        // Insert the meta data of the Modulator to modulatorMetaMap
        modulatorMeta.init(modulatorUpdater, numberNode);
        modulatorMetaMap[modulator] = modulatorMeta;

        logger().debug(
    "PsiModulatorUpdaterAgent::init - Store the meta data of  modulator '%s' successfully.", 
                modulator.c_str() 
                );
    }// for

    // Avoid initialize during next cycle
    this->bInitialized = true;
}

void PsiModulatorUpdaterAgent::runUpdaters(opencog::CogServer * server)
{
    logger().debug("PsiModulatorUpdaterAgent::runUpdaters - Run updaters (combo scripts)");

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

        while ( !procedureInterpreter.isFinished(executingSchemaId) )
            procedureInterpreter.run(NULL);  

        // Check if the the updater run successfully
        if ( procedureInterpreter.isFailed(executingSchemaId) ) {
            logger().error( 
                    "PsiModulatorUpdaterAgent::runUpdaters - Failed to execute '%s'", 
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
        logger().debug( "PsiModulatorUpdater::run - The new level of '%s' will be %f", 
                         modulator.c_str(),
                         iModulator->second.updatedValue                      
                      );
    }// for

}    

void PsiModulatorUpdaterAgent::setUpdatedValues(opencog::CogServer * server)
{
    logger().debug("PsiModulatorUpdaterAgent::runUpdaters - Run updaters (combo scripts)");

    // Get OAC
    OAC * oac = (OAC *) server;

    // Get AtomSpace
    AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    // Process Modulators one by one
    std::map <std::string, ModulatorMeta>::iterator iModulator;

    std::string modulator;
    Handle numberNode;
    double updatedValue;

    for ( iModulator = modulatorMetaMap.begin();
          iModulator != modulatorMetaMap.end();
          iModulator ++ ) {

        if ( !iModulator->second.bUpdated )
            continue;

        modulator = iModulator->first;
        numberNode = iModulator->second.numberNode;
        updatedValue = iModulator->second.updatedValue;

        // Set updated values to the corresponding NumberNode
        atomSpace.setName( numberNode, boost::lexical_cast<std::string> (updatedValue) );

        iModulator->second.bUpdated = false;

        // TODO: Change the log level to fine, after testing
        logger().debug( 
          "PsiModulatorUpdater::setUpdatedValues - Set the level of modulator '%s' to %f", 
           modulator.c_str(),
           updatedValue
                      );
    }// for
}

void PsiModulatorUpdaterAgent::run(opencog::CogServer * server)
{
    logger().debug("PsiModulatorUpdaterAgent::run - Executing run");

    /*
    // Initialize the Agent (modulatorMetaMap etc)
    if ( !this->bInitialized )
        this->init(server);

    // Run updaters (combo scripts)
    this->runUpdaters(server);

    // Set updated values to AtomSpace (NumberNodes)
    this->setUpdatedValues(server);
    */
}

