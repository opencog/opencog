/*
 * opencog/embodiment/Control/OperationalAvatarController/EntityExperienceAgent.cc
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


#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/nlp/types/atom_types.h>
#include <opencog/spacetime/atom_types.h>
#include <opencog/spacetime/SpaceServer.h>

#include <opencog/embodiment/Control/OperationalAvatarController/OAC.h>
#include <opencog/embodiment/Control/OperationalAvatarController/EntityExperienceAgent.h>

using namespace opencog::oac;

EntityExperienceAgent::~EntityExperienceAgent()
{
}

EntityExperienceAgent::EntityExperienceAgent(CogServer& cs) : Agent(cs)
{
    this->elapsedMoments = 1.0f;
}

void EntityExperienceAgent::run()
{
    // Disable this agent
    return;

    logger().debug("EntityExperienceAgent::%s - Experiencing entities at moment '%d'", 
                   __FUNCTION__, this->elapsedMoments );

    AtomSpace& atomSpace = _cogserver.getAtomSpace();

    if ( spaceServer().getLatestMapHandle( ) == Handle::UNDEFINED ) {
        logger().warn( "EntityExperienceAgent::%s - There is no map info available yet!",
            __FUNCTION__ );
        return;
    } // if

    const SpaceServer::SpaceMap& map = spaceServer( ).getLatestMap( );

    HandleSeq semeNodes;
    {
        Type type[] = { OBJECT_NODE, SEME_NODE };
        bool subclasses[] = { true, false };
        HandleSeq links, link(2);
        link[0] = Handle::UNDEFINED;
        link[1] = Handle::UNDEFINED;
        atomSpace.getHandlesByOutgoing( std::back_inserter(links), link, &type[0],
                                 &subclasses[0], 2, REFERENCE_LINK, false );
        unsigned int i;
        for( i = 0; i < links.size( ); ++i ) {
            semeNodes.push_back( atomSpace.getOutgoing( links[i], 1 ) );
        } // for
    }

    std::map<Handle, bool > nodeStatus;

    // first, for each seme node, retrieve its class node(s) and then
    // compute its importance giving the agent perception of that object
    unsigned int i;
    for ( i = 0; i < semeNodes.size( ); ++i ) {        

        HandleSeq typeLinks, classLinks;
        {
            Type type[] = { SEME_NODE, CONCEPT_NODE };
            HandleSeq link = { semeNodes[i], Handle::UNDEFINED };
            atomSpace.getHandlesByOutgoing( std::back_inserter( typeLinks ), link, 
                                     &type[0], NULL, 2, INHERITANCE_LINK, false );
        }
        {
            Type type[] = { CONCEPT_NODE, SEME_NODE };
            HandleSeq link = { Handle::UNDEFINED, semeNodes[i] };
            atomSpace.getHandlesByOutgoing( std::back_inserter( classLinks ), link, 
                                     &type[0], NULL, 2, REFERENCE_LINK, false );
        }
        

        const std::string& name = atomSpace.getName( semeNodes[i] );
        TruthValuePtr tv = atomSpace.getTV( semeNodes[i]);

        strength_t mean = 1.0;
        count_t count = 1.0;
        if ( tv->getType() == SIMPLE_TRUTH_VALUE ) {
            const SimpleTruthValue& stv = *dynamic_cast<const SimpleTruthValue*>( tv.get() );
            mean = stv.getMean();
            count = stv.getCount();
        } // if 

        /// @todo this is not a proper use of the definition of
        /// TV.count, which is actually equivalent to elapesMoments
        mean = static_cast<strength_t>( count ) / 
            static_cast<strength_t>( this->elapsedMoments );

        bool containsObject = map.containsObject( semeNodes[i] );
        if ( containsObject ) {
            // ok. it was perceived by the agent, so its strength
            // will be increased
            ++count;
        } // if

        logger().debug("EntityExperienceAgent::%s - Entity '%s' now has strength '%f' and count '%f'", 
                       __FUNCTION__, name.c_str( ), mean, count );

        atomSpace.setTV( semeNodes[i], SimpleTruthValue::createTV( mean, count ) );


        unsigned int j;
        for( j = 0; j < typeLinks.size( ); ++j ) {
            Handle typeNode = atomSpace.getOutgoing( typeLinks[j], 1 );
            if ( nodeStatus.find( typeNode ) == nodeStatus.end( ) ) {
                nodeStatus.insert( std::map<Handle, bool >::value_type( typeNode, false ) );
            } // if
            nodeStatus[ typeNode ] |= containsObject;
        } // for
        for( j = 0; j < classLinks.size( ); ++j ) {
            Handle classNode = atomSpace.getOutgoing( classLinks[j], 0 );
            if ( nodeStatus.find( classNode ) == nodeStatus.end( ) ) {
                nodeStatus.insert( std::map<Handle, bool >::value_type( classNode, false ) );
            } // if
            nodeStatus[ classNode ] |= containsObject;
        } // for
        
    } // for

    // now, compute the importance of the objects classes.
    // at least one object of each class is necessary to
    // enhance the class count
    std::map<Handle, bool >::const_iterator it;
    for( it = nodeStatus.begin( ); it != nodeStatus.end( ); ++it ) {
        TruthValuePtr tv = atomSpace.getTV( it->first );

        strength_t mean = 1;
        count_t count = 1;
        if ( tv->getType() == SIMPLE_TRUTH_VALUE ) {
            mean = tv->getMean();
            count = tv->getCount();
        } // if

        mean = static_cast<strength_t>( count ) / 
            static_cast<strength_t>( this->elapsedMoments );

        if ( it->second ) {
            ++count;
        } // if

        logger().debug("EntityExperienceAgent::%s - Class '%s' now has strength '%f' and count '%f'", 
                       __FUNCTION__, atomSpace.getName( it->first ).c_str( ), mean, count );

        atomSpace.setTV( it->first, SimpleTruthValue::createTV( mean, count ) );
    } // for
    

    ++this->elapsedMoments;
    
}
