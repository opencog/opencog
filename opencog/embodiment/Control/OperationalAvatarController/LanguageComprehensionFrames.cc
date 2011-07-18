/*
 * opencog/embodiment/Control/OperationalAvatarController/LanguageComprehensionFrames.cc
 *
 * Copyright (C) 2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Samir Araujo
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

#include <opencog/embodiment/Control/OperationalAvatarController/LanguageComprehension.h>
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include <opencog/embodiment/Control/EmbodimentConfig.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/guile/SchemeSmob.h>

using namespace opencog::oac;
using namespace opencog;
using namespace opencog::spatial;

opencog::control::AvatarInterface* LanguageComprehension::localAgent = NULL;

#ifdef HAVE_GUILE
// XXX TODO This code should not be using SCM directly, it should
// do all of its work with the existing interfaces into guile.
// At some point, it should be re-written to do so ...
SCM LanguageComprehension::execute(SCM objectObserver, SCM figureSemeNode, SCM groundSemeNode, SCM ground2SemeNode ) {
    opencog::AtomSpace& atomSpace = localAgent->getAtomSpace( );
    HandleSeq resultingFrames;
    if ( scm_is_null(objectObserver) ) {
        logger().error( "ComputeSpatialRelations::%s - Invalid null observer reference",
                        __FUNCTION__ );
        return SchemeSmob::handle_to_scm( atomSpace.addLink( LIST_LINK, resultingFrames ) );
    } // if
    if ( scm_is_null(groundSemeNode) ) {
        logger().error( "ComputeSpatialRelations::%s - Invalid null reference ground object",
                        __FUNCTION__ );
        return SchemeSmob::handle_to_scm( atomSpace.addLink( LIST_LINK, resultingFrames ) );
    } // if
    if ( scm_is_null(figureSemeNode) ) {
        logger().error( "ComputeSpatialRelations::%s - Invalid null reference figure object",
                        __FUNCTION__ );
        return SchemeSmob::handle_to_scm( atomSpace.addLink( LIST_LINK, resultingFrames ) );
    } // if
    
    Handle observer = SchemeSmob::scm_to_handle(objectObserver);
    Handle objectA = SchemeSmob::scm_to_handle(figureSemeNode);
    Handle objectB = SchemeSmob::scm_to_handle(groundSemeNode);
    Handle objectC = (scm_is_pair(ground2SemeNode) && !scm_is_null(SCM_CAR(ground2SemeNode)) ) ? 
        SchemeSmob::scm_to_handle(SCM_CAR(ground2SemeNode)) : Handle::UNDEFINED;
    
    const SpaceServer::SpaceMap& spaceMap = 
        atomSpace.getSpaceServer( ).getLatestMap( );
    
    double besideDistance = spaceMap.getNextDistance( );
    
    {
        std::stringstream msg;
        msg << "observer[" << atomSpace.atomAsString( observer ) << "] ";
        msg << "objectA[" << atomSpace.atomAsString( objectA ) << "] ";
        msg << "objectB[" << atomSpace.atomAsString( objectB ) << "] ";
        if ( objectC != Handle::UNDEFINED ) {
            msg << "objectC[" << atomSpace.getName( objectC ) << "] ";
        } // if            
        logger().debug( "ComputeSpatialRelations::%s - Computing spatial relations for '%s'",
                        __FUNCTION__, msg.str( ).c_str( ) );
    }
    
    std::vector<std::string> entitiesA;
    std::vector<std::string> entitiesB;
    std::vector<std::string> entitiesC;
   
    // Get the corresponding ObjectNode (or child of it) of SemeNode. 
    // ObjectA, ObjectB and ObjectC are all SemeNodes. 
    //
    // (ReferenceLink (stv 1 1) (av -8 1 0)
    //     (AccessoryNode "id_4410" (av -8 1 0))
    //     (SemeNode "id_4410" (stv 1 0.051008303))
    // )
    if ( atomSpace.getType( objectA ) == VARIABLE_NODE ) {
        spaceMap.getAllObjects( std::back_inserter( entitiesA ) );
    } else {
        HandleSeq incoming = atomSpace.getIncoming( objectA );
        unsigned int i;
        for( i = 0; i < incoming.size( ); ++i ) {
            Handle firstElement = atomSpace.getOutgoing(incoming[i], 0 );
            if ( classserver().isA( atomSpace.getType(firstElement), OBJECT_NODE ) ) {
                entitiesA.push_back( atomSpace.getName( firstElement ) );
            } // if
        } // for            
    } // else
    
    if ( atomSpace.getType( objectB ) == VARIABLE_NODE ) {
        spaceMap.getAllObjects( std::back_inserter( entitiesB ) );
    } else {
        HandleSeq incoming = atomSpace.getIncoming( objectB );
        unsigned int i;
        for( i = 0; i < incoming.size( ); ++i ) {
            Handle firstElement = atomSpace.getOutgoing(incoming[i], 0 );
            if ( classserver().isA( atomSpace.getType(firstElement), OBJECT_NODE ) ) {
                entitiesB.push_back( atomSpace.getName( firstElement ) );
            } // if
        } // for
    } // else
    
    if ( objectC != Handle::UNDEFINED ) {
        if ( atomSpace.getType( objectC ) == VARIABLE_NODE ) {
            spaceMap.getAllObjects( std::back_inserter( entitiesC ) );
        } else {
            HandleSeq incoming = atomSpace.getIncoming( objectC );
            unsigned int i;
            for( i = 0; i < incoming.size( ); ++i ) {
                Handle firstElement = atomSpace.getOutgoing(incoming[i], 0 );
                if ( classserver().isA( atomSpace.getType(firstElement), OBJECT_NODE ) ) {
                    entitiesC.push_back( atomSpace.getName( firstElement ) );
                } // if
            } // for
        } // else            
    } // if
    
    logger().debug( "ComputeSpatialRelations::%s - %d candidates for objectA. %d candidates for objectB. %d candidates for objectC",
                    __FUNCTION__, entitiesA.size( ), entitiesB.size( ), entitiesC.size( ) );
    
    try {
        const spatial::EntityPtr& observerEntity = spaceMap.getEntity( atomSpace.getName( observer ) );
        
        unsigned int i, j, k;
        for( i = 0; i < entitiesA.size( ); ++i ) {
            const spatial::EntityPtr& entityA = spaceMap.getEntity( entitiesA[i] );
            for( j = 0; j < entitiesB.size( ); ++j ) {
                if ( entitiesA[i] == entitiesB[j] ) {
                    continue;
                } // if
                const spatial::EntityPtr& entityB = spaceMap.getEntity( entitiesB[j] );
                if ( entitiesC.size( ) > 0 ) {
                    for( k = 0; k < entitiesC.size( ); ++k ) {
                        if ( entitiesA[i] == entitiesC[k] || entitiesB[j] == entitiesC[k] ) {
                            continue;
                        } // if
                        const spatial::EntityPtr& entityC = spaceMap.getEntity( entitiesC[k] );
                        createFrameInstancesFromRelations( atomSpace, resultingFrames,
                            entityA->computeSpatialRelations( *observerEntity, besideDistance, *entityB, *entityC ),
                                entitiesA[i], entitiesB[j], entitiesC[k] );
                    } // for
                } else {
                    createFrameInstancesFromRelations( atomSpace, resultingFrames,
                        entityA->computeSpatialRelations( *observerEntity, besideDistance, *entityB ),
                            entitiesA[i], entitiesB[j], "" );                        
                } // else
            } // for
        } // for
    } catch( const opencog::NotFoundException& ex ) {
        logger().error( "LanguageComprehension::%s - %s", __FUNCTION__, ex.getMessage( ) );
        return SchemeSmob::handle_to_scm( atomSpace.addLink( LIST_LINK, resultingFrames ) );
    } // if
        
    return SchemeSmob::handle_to_scm( atomSpace.addLink( LIST_LINK, resultingFrames ) );
}
#endif

void LanguageComprehension::createFrameInstancesFromRelations( 
    AtomSpace& atomSpace, HandleSeq& resultingFrames,
        const std::list<spatial::Entity::SPATIAL_RELATION>& relations,
            const std::string& objectA, const std::string& objectB, const std::string& objectC ) {

    std::list<spatial::Entity::SPATIAL_RELATION>::const_iterator it;
    for( it = relations.begin( ); it != relations.end( ); ++it ) {
        std::string relationName = spatial::Entity::spatialRelationToString( *it );

        std::map<std::string, Handle> elements;
        elements["Figure"] = atomSpace.getHandle( SEME_NODE, objectA );
        elements["Ground"] = atomSpace.getHandle( SEME_NODE, objectB );
        elements["Relation_type"] = atomSpace.addNode( CONCEPT_NODE, relationName );

        std::stringstream instanceName;
        instanceName << objectA;
        instanceName << "_";
        instanceName << objectB;

        if ( *it == spatial::Entity::BETWEEN ) {
            elements["Ground_2"] = atomSpace.getHandle( SEME_NODE, objectC );
            instanceName << "_";
            instanceName << objectC;
        } // if            

        instanceName << "_" << relationName;
        resultingFrames.push_back(
            AtomSpaceUtil::setPredicateFrameFromHandles(
                atomSpace, "#Locative_relation", instanceName.str( ), 
                    elements, SimpleTruthValue(1.0, 1.0), false ) );
    } // for
}


void LanguageComprehension::loadFrames( void )
{
    opencog::AtomSpace& atomSpace = agent.getAtomSpace( );

    { // try to load the frames
        std::string fileName = config().get("FRAMES_FILE");
        boost::trim(fileName);
        if ( fileName.length( ) == 0 ) {
            logger().debug( "LanguageComprehension::%s - No frames filename was defined",
                            __FUNCTION__ );
            return;
        } // if
        
        
        std::ifstream input( fileName.c_str( ) );
        
        if ( !input ) {
            logger().error( "LanguageComprehension::%s - Cannot load frames from: %s",
                            __FUNCTION__, fileName.c_str( ) );
            return;
        } // if
        
        while( !input.eof( ) ) {
            std::string line;
            std::getline( input, line );
            boost::trim(line);
            if ( line.length( ) == 0 ) {
                continue;
            } // if
            std::vector<std::string> tokens;
            boost::split( tokens, line, boost::is_any_of( ";" ) );

            std::string frameName = "#" + tokens[0];

            Handle frameNode = atomSpace.addNode( DEFINED_FRAME_NODE, frameName );
            HandleSeq element(2);
            element[0] = frameNode;

            unsigned int i;
            for( i = 1; i < tokens.size( ); ++i ) {
                std::string frameElementName = frameName + ":" + tokens[i];
                element[1] = atomSpace.addNode( DEFINED_FRAME_ELEMENT_NODE, frameElementName );
                Handle elementLink = atomSpace.addLink( FRAME_ELEMENT_LINK, element );
                atomSpace.setTV( elementLink, opencog::TruthValue::TRUE_TV( ) );
                atomSpace.setLTI( elementLink, 1 );
            } // if

        } // while
    } // end block

    { // try to load the frames relations (only inheritance is supported for now)
        std::string fileName = config().get("FRAMES_INHERITANCE_FILE");
        boost::trim(fileName);    
        if ( fileName.length( ) == 0 ) {
            logger().debug( "LanguageComprehension::%s - No frames relations filename was defined",
                            __FUNCTION__ );
            return;
        } // if
        
        
        std::ifstream input( fileName.c_str( ) );
        
        if ( !input ) {
            logger().error( "LanguageComprehension::%s - Cannot load frames relations from: %s",
                            __FUNCTION__, fileName.c_str( ) );
            return;
        } // if
                
        while( !input.eof( ) ) {
            std::string line;
            std::getline( input, line );
            boost::trim(line);
            if ( line.length( ) == 0 ) {
                continue;
            } // if
            std::string inheritance;
            std::string elementsMap;
            {
                std::vector<std::string> tokens;
                boost::split( tokens, line, boost::is_any_of( "|" ) );
                inheritance = tokens[0];
                elementsMap = tokens[1];
            }
            {
                std::vector<std::string> tokens;
                boost::split( tokens, inheritance, boost::is_any_of( ";" ) );
                HandleSeq inheritance;
                inheritance.push_back( atomSpace.addNode( DEFINED_FRAME_NODE, "#"+tokens[1] ) ); // child
                inheritance.push_back( atomSpace.addNode( DEFINED_FRAME_NODE, "#"+tokens[0] ) ); // parent

                
                Handle inheritanceLink = atomSpace.addLink( INHERITANCE_LINK, inheritance );
                atomSpace.setTV( inheritanceLink, opencog::TruthValue::TRUE_TV( ) );
                atomSpace.setLTI( inheritanceLink, 1 );
            }

        } // while

    } // end block
}
