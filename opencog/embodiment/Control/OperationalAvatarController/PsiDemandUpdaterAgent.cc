/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiDemandUpdaterAgent.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2010-12-09
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

#include<boost/tokenizer.hpp>

using namespace OperationalAvatarController;

PsiDemandUpdaterAgent::~PsiDemandUpdaterAgent()
{

}

PsiDemandUpdaterAgent::PsiDemandUpdaterAgent()
{
    this->cycleCount = 0;

    // Force the Agent initialize itself during its first cycle. 
    this->forceInitNextCycle();
}

void PsiDemandUpdaterAgent::init(opencog::CogServer * server) 
{
    logger().debug( "PsiDemandUpdaterAgent::%s - Initialize the Agent [ cycle = %d ]",
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

    // Clear old demandMetaMap; 
    this->demandMetaMap.clear();

    // Get demand names from the configuration file
    std::string demandNames = config()["PSI_DEMANDS"];

    // Process Demands one by one
    boost::tokenizer<> demandNamesTok (demandNames);
    std::string demand, demandUpdater;
    Handle similarityLink, simultaneousEquivalenceLink;
    DemandMeta demandMeta;

    for ( boost::tokenizer<>::iterator iDemandName = demandNamesTok.begin();
          iDemandName != demandNamesTok.end();
          iDemandName ++ ) {

        demand = (*iDemandName);
        demandUpdater = demand + "DemandUpdater";

        logger().debug(
              "PsiDemandUpdaterAgent::%s - Searching the meta data of demand '%s'.", 
                        __FUNCTION__, 
                        demand.c_str() 
                      );

        // Search demand updater
        if ( !procedureRepository.contains(demandUpdater) ) {
            logger().warn( 
       "PsiDemandUpdaterAgent::%s - Failed to find '%s' in OAC's procedureRepository",
                           __FUNCTION__, 
                           demandUpdater.c_str()
                         );
            continue;
        }
    
        // Search the corresponding SimilarityLink
        similarityLink =  AtomSpaceUtil::getDemandSimilarityLink( atomSpace, 
                                                                  demand,
                                                                  petId
                                                                );

        if ( similarityLink == Handle::UNDEFINED )
        {
            logger().warn(
    "PsiDemandUpdaterAgent::%s - Failed to get the SimilarityLink for demand '%s'",
                           __FUNCTION__, 
                           demand.c_str()
                         );

            continue;
        }

        // Search the corresponding SimultaneousEquivalenceLink
        simultaneousEquivalenceLink =  AtomSpaceUtil::getDemandSimultaneousEquivalenceLink
                                           ( atomSpace, 
                                             demand,
                                             petId
                                           );

        if ( simultaneousEquivalenceLink == Handle::UNDEFINED )
        {
            logger().warn( "PsiDemandUpdaterAgent::%s - Failed to get the SimultaneousEquivalenceLink for demand '%s'",
                           __FUNCTION__, 
                           demand.c_str()
                         );

            continue;
        }

        // Insert the meta data of the Demand to demandMetaMap
        demandMeta.init(demandUpdater, similarityLink, simultaneousEquivalenceLink);
        demandMetaMap[demand] = demandMeta;

        logger().debug(
     "PsiDemandUpdaterAgent::%s - Store the meta data of  demand '%s' successfully.", 
                        __FUNCTION__, 
                        demand.c_str() 
                      );
    }// for

    // Avoid initialize during next cycle
    this->bInitialized = true;
}

void PsiDemandUpdaterAgent::runUpdaters(opencog::CogServer * server)
{
    logger().debug( 
            "PsiDemandUpdaterAgent::%s - Run updaters (combo scripts) [ cycle = %d ]", 
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

    // Process Demands one by one
    std::map <std::string, DemandMeta>::iterator iDemand;

    std::string demand, demandUpdater;
    std::vector <combo::vertex> schemaArguments;
    Procedure::RunningProcedureID executingSchemaId;
    combo::vertex result; // combo::vertex is actually of type boost::variant <...>

    for ( iDemand = demandMetaMap.begin();
          iDemand != demandMetaMap.end();
          iDemand ++ ) {

        demand = iDemand->first;
        demandUpdater = iDemand->second.updaterName;

        // Run the Procedure that update Demand and get the updated value
        const Procedure::GeneralProcedure & procedure =
                                            procedureRepository.get(demandUpdater);

        executingSchemaId = procedureInterpreter.runProcedure(procedure, schemaArguments);

        // TODO: What does this for?
        while ( !procedureInterpreter.isFinished(executingSchemaId) )
            procedureInterpreter.run(NULL);  

        // Check if the the updater run successfully
        if ( procedureInterpreter.isFailed(executingSchemaId) ) {
            logger().error( "PsiDemandUpdaterAgent::%s - Failed to execute '%s'", 
                             __FUNCTION__, 
                             demandUpdater.c_str() 
                          );

            iDemand->second.bUpdated = false;

            continue;
        }
        else {
            iDemand->second.bUpdated = true;
        }

        result = procedureInterpreter.getResult(executingSchemaId);

        // Store updated value to DemandMeta.updatedValue
        // contin_t is actually of type double (see "comboreduct/combo/vertex.h") 
        iDemand->second.updatedValue = get_contin(result);

        // TODO: Change the log level to fine, after testing
        logger().debug( "PsiDemandUpdaterAgent::%s - The new level of '%s' will be %f", 
                         __FUNCTION__, 
                         demand.c_str(),
                         iDemand->second.updatedValue                      
                      );
    }// for

}    

void PsiDemandUpdaterAgent::setUpdatedValues(opencog::CogServer * server)
{
    logger().debug(
            "PsiDemandUpdaterAgent::%s - Set updated values to AtomSpace [ cycle =%d ]",
                    __FUNCTION__, 
                    this->cycleCount
                  );

    // Get OAC
    OAC * oac = (OAC *) server;

    // Get AtomSpace
    AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    // Process Demands one by one
    //
    // Don't be scared by the bulgy 'for' loop below, what it does is quite simple,
    // get old Handles, create new Handles and remove old Handles 
    std::map <std::string, DemandMeta>::iterator iDemand;

    std::string demand;

    Handle oldSimilarityLink, oldNumberNode, oldExecutionOutputLink;
    Handle newNumberNode, newSimilarityLink;

    Handle oldSimultaneousEquivalenceLink, oldEvaluationLinkDemandGoal,
           oldEvaluationLinkFuzzyWithin, oldListLink;
    Handle newSimultaneousEquivalenceLink, newEvaluationLinkFuzzyWithin, newListLink;

    double updatedValue;

    for ( iDemand = demandMetaMap.begin();
          iDemand != demandMetaMap.end();
          iDemand ++ ) {

        if ( !iDemand->second.bUpdated )
            continue;

        demand = iDemand->first;
        oldSimilarityLink = iDemand->second.similarityLink;
        oldSimultaneousEquivalenceLink = iDemand->second.simultaneousEquivalenceLink;
        updatedValue = iDemand->second.updatedValue;
        
        // Get the Handle to old NumberNode and ExecutionOutputLink
        //
        // Since SimilarityLink inherits from UnorderedLink, you should check each Atom in 
        // the Outgoing set to see if it is of type NumberNode or ExecutionOutputLink.
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

                logger().error( "PsiDemandUpdaterAgent::%s - The outgoing set of SimilarityLink for '%s' should contain a NumberNode and a ExecutionOutputLink, but got [0]:%s, [1]:%s",
                                __FUNCTION__, 
                                demand.c_str(), 
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

        // Get the Handle to old EvaluationLinkDemandGoal and EvaluationLinkFuzzyWithin
        //
        // Since SimultaneousEquivalenceLink inherits from UnorderedLink,
        // we should make a choice
        if ( atomSpace.getArity(
                                   atomSpace.getOutgoing(oldSimultaneousEquivalenceLink, 0)
                               ) ==  1 ) {
            oldEvaluationLinkDemandGoal = atomSpace.getOutgoing(
                                                        oldSimultaneousEquivalenceLink, 0
                                                               );

            oldEvaluationLinkFuzzyWithin = atomSpace.getOutgoing(
                                                        oldSimultaneousEquivalenceLink, 1
                                                                ); 
        }
        else {

            oldEvaluationLinkDemandGoal = atomSpace.getOutgoing(
                                                        oldSimultaneousEquivalenceLink, 1
                                                               );

            oldEvaluationLinkFuzzyWithin = atomSpace.getOutgoing(
                                                        oldSimultaneousEquivalenceLink, 0
                                                                ); 
        }// if

        // Get the Handle to old ListLink
        oldListLink = atomSpace.getOutgoing(oldEvaluationLinkFuzzyWithin, 1);

logger().fine( "PsiDemandUpdaterAgent::%s - DemandSchema: %s, oldSimilarityLink: %s, oldNumberNode: %s, updatedValue: %f", 
               __FUNCTION__,
               demand.c_str(), 
               atomSpace.atomAsString(oldSimilarityLink).c_str(),
               atomSpace.atomAsString(oldNumberNode).c_str(), 
               updatedValue
             );        

logger().fine( "PsiDemandUpdaterAgent::%s - DemandGoal: %s, oldSimultaneousEquivalenceLink: %s, oldEvaluationLinkDemandGoal: %s, oldEvaluationLinkFuzzyWithin: %s, oldListLink: %s", 
               __FUNCTION__,
               demand.c_str(), 
               atomSpace.atomAsString(oldSimultaneousEquivalenceLink).c_str(),
               atomSpace.atomAsString(oldEvaluationLinkDemandGoal).c_str(), 
               atomSpace.atomAsString(oldEvaluationLinkFuzzyWithin).c_str(),
               atomSpace.atomAsString(oldListLink).c_str() 
             );        

        // Create a new NumberNode that stores the updated value
        //
        // If the Demand doesn't change at all, it actually returns the old one
        //
        newNumberNode = AtomSpaceUtil::addNode( atomSpace,
                                                NUMBER_NODE,
                                                boost::lexical_cast<std::string>
                                                                   (updatedValue), 
                                                true // the atom should be permanent (not
                                                     // removed by decay importance task) 
                                               );

logger().fine( "PsiDemandUpdaterAgent::%s - newNumberNode: %s", 
               __FUNCTION__, 
               atomSpace.atomAsString(newNumberNode).c_str()
             );        

        // Create a new SimilarityLink that holds the DemandSchema
        //
        // Since SimilarityLink inherits from UnorderedLink, which will sort its Outgoing 
        // set automatically when being created, the sequence you adding Atoms to its 
        // Outgoing set makes none sense! 
        // (more detail: "./atomspace/Link.cc", Link::setOutgoingSet method)
        //
        // If the Demand doesn't change at all, it actually returns the old one
        //
        HandleSeq similarityLinkHandleSeq;  // HandleSeq is of type std::vector<Handle>

        similarityLinkHandleSeq.push_back(newNumberNode);
        similarityLinkHandleSeq.push_back(oldExecutionOutputLink);

        newSimilarityLink = AtomSpaceUtil::addLink( atomSpace,
                                                    SIMILARITY_LINK, 
                                                    similarityLinkHandleSeq,
                                                    true
                                                  );

        iDemand->second.similarityLink = newSimilarityLink;

logger().fine( "PsiDemandUpdaterAgent::%s - newSimilarityLink: %s", 
               __FUNCTION__, 
               atomSpace.atomAsString(newSimilarityLink).c_str()
             );        

        // Create a new ListLink that holds the SimilarityLink
        HandleSeq listLinkHandleSeq = atomSpace.getOutgoing(oldListLink);

        listLinkHandleSeq[2] = newSimilarityLink;
        
        newListLink = AtomSpaceUtil::addLink( atomSpace,
                                              LIST_LINK, 
                                              listLinkHandleSeq,
                                              true
                                            );

logger().fine( "PsiDemandUpdaterAgent::%s - newListLink: %s", 
               __FUNCTION__, 
               atomSpace.atomAsString(newListLink).c_str()
             );        

        // Create a new EvaluationLinkFuzzyWithin that holds the ListLink
        HandleSeq evaluationLinkFuzzyWithinHandleSeq = 
                                       atomSpace.getOutgoing(oldEvaluationLinkFuzzyWithin);

        evaluationLinkFuzzyWithinHandleSeq[1] = newListLink;
        
        newEvaluationLinkFuzzyWithin = AtomSpaceUtil::addLink
                                           ( atomSpace,
                                             EVALUATION_LINK, 
                                             evaluationLinkFuzzyWithinHandleSeq,
                                             true
                                            );

logger().fine( "PsiDemandUpdaterAgent::%s - newEvaluationLinkFuzzyWithin: %s", 
               __FUNCTION__, 
               atomSpace.atomAsString(newEvaluationLinkFuzzyWithin).c_str()
             );        

        // Create a new SimultaneousEquivalenceLink that holds the DemandGoal
        HandleSeq simultaneousEquivalenceLinkHandleSeq;

        simultaneousEquivalenceLinkHandleSeq.push_back(oldEvaluationLinkDemandGoal);
        simultaneousEquivalenceLinkHandleSeq.push_back(newEvaluationLinkFuzzyWithin);

        newSimultaneousEquivalenceLink = AtomSpaceUtil::addLink
                                           ( atomSpace,
                                             SIMULTANEOUS_EQUIVALENCE_LINK, 
                                             simultaneousEquivalenceLinkHandleSeq,
                                             true
                                            );

        iDemand->second.simultaneousEquivalenceLink = newSimultaneousEquivalenceLink;

logger().fine( "PsiDemandUpdaterAgent::%s - newSimultaneousEquivalenceLink: %s", 
               __FUNCTION__, 
               atomSpace.atomAsString(newSimultaneousEquivalenceLink).c_str()
             );        

        // Remove the old SimultaneousEquivalenceLink, EvaluationLinkFuzzyWithin, ListLink,
        // SimilarityLink and NumberNode
        //
        // Note:
        //     1. Make sure the old one is different from the new one before removing.
        //        Because if the Demand does not change at all, including its value, 
        //        True Value etc, creating new one is actually returning the old one,
        //        then we shall not remove it.
        //     2. The sequence of removing Atoms is contrary to that of creating. 
        //        That is you should remove the out layer Link firstly, and then its
        //        Outgoing set.  
        //

logger().fine( "PsiDemandUpdaterAgent::%s - Going to remove oldSimultaneousEquivalenceLink", 
                __FUNCTION__);

        if ( oldSimultaneousEquivalenceLink != newSimultaneousEquivalenceLink && 
             !atomSpace.removeAtom(oldSimultaneousEquivalenceLink) 
           ) {
            logger().error( "PsiDemandUpdaterAgent::%s - Unable to remove old SIMULTANEOUS_EQUIVALENCE_LINK: %s",
                            __FUNCTION__, 
                            atomSpace.atomAsString(oldSimultaneousEquivalenceLink).c_str()
                          );
        }// if

logger().fine( "PsiDemandUpdaterAgent::%s - Removed oldSimultaneousEquivalenceLink",
               __FUNCTION__
             );

logger().fine( "PsiDemandUpdaterAgent::%s - Going to remove oldEvaluationLinkFuzzyWithin", 
                __FUNCTION__);

        if ( oldEvaluationLinkFuzzyWithin != newEvaluationLinkFuzzyWithin && 
             !atomSpace.removeAtom(oldEvaluationLinkFuzzyWithin) 
           ) {
            logger().error( "PsiDemandUpdaterAgent::%s - Unable to remove old EvaluationLinkFuzzyWithin: %s",
                            __FUNCTION__, 
                            atomSpace.atomAsString(oldEvaluationLinkFuzzyWithin).c_str()
                          );
        }// if

logger().fine( "PsiDemandUpdaterAgent::%s - Removed oldEvaluationLinkFuzzyWithin",
               __FUNCTION__
             );

logger().fine( "PsiDemandUpdaterAgent::%s - Going to remove oldListLink", 
                __FUNCTION__);

        if ( oldListLink != newListLink && 
             !atomSpace.removeAtom(oldListLink) 
           ) {
            logger().error(
                "PsiDemandUpdaterAgent::%s - Unable to remove old LIST_LINK: %s",
                __FUNCTION__, 
                atomSpace.atomAsString(oldListLink).c_str()
                          );
        }// if

logger().fine("PsiDemandUpdaterAgent::%s - Removed oldListLink", __FUNCTION__);

logger().fine( "PsiDemandUpdaterAgent::%s - Going to remove oldSimilarityLink", 
                __FUNCTION__);

        if ( oldSimilarityLink != newSimilarityLink && 
             !atomSpace.removeAtom(oldSimilarityLink) 
           ) {
            logger().error(
                "PsiDemandUpdaterAgent::%s - Unable to remove old SIMILARITY_LINK: %s",
                __FUNCTION__, 
                atomSpace.atomAsString(oldSimilarityLink).c_str()
                          );
        }// if

logger().fine("PsiDemandUpdaterAgent::%s - Removed oldSimilarityLink", __FUNCTION__);

logger().fine("PsiDemandUpdaterAgent::%s - Going to remove oldNumberNode", __FUNCTION__);

        if ( oldNumberNode != newNumberNode && 
             !atomSpace.removeAtom(oldNumberNode) 
           ) {
            logger().error(
                "PsiDemandUpdaterAgent::%s - Unable to remove old NUMBER_NODE: %s",
                __FUNCTION__, 
                atomSpace.atomAsString(oldNumberNode).c_str()
                          );
        }// if

logger().fine( "PsiDemandUpdaterAgent::%s - Removed oldNumberNode", __FUNCTION__); 

        // Reset bUpdated  
        iDemand->second.bUpdated = false;

        // TODO: Change the log level to fine, after testing
        logger().debug( 
                    "PsiDemandUpdaterAgent::%s - Set the level of demand '%s' to %f", 
                         __FUNCTION__, 
                         demand.c_str(),
                         updatedValue
                      );

    }// for

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

    // Get petId
    const std::string & petId = oac->getPet().getPetId();

    // Check if map info data is available
    if ( atomSpace.getSpaceServer().getLatestMapHandle() == Handle::UNDEFINED ) {
        logger().warn( 
      "PsiDemandUpdaterAgent::%s - There is no map info available yet [ cycle = %d ]", 
                        __FUNCTION__, 
                        this->cycleCount
                     );
        return;
    }

    // Check if the pet spatial info is already received
    if ( !atomSpace.getSpaceServer().getLatestMap().containsObject(petId) ) {
        logger().warn(
 "PsiDemandUpdaterAgent::%s - Pet was not inserted in the space map yet [ cycle = %d ]", 
                     __FUNCTION__, 
                     this->cycleCount
                     );
        return;
    }

    // Initialize the Agent (demandMetaMap etc)
    if ( !this->bInitialized )
        this->init(server);

    // Run updaters (combo scripts)
    this->runUpdaters(server);

    // Set updated values to AtomSpace (NumberNodes)
    this->setUpdatedValues(server);
}

