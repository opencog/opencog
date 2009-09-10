/*
 * opencog/embodiment/Control/OperationalPetController/LanguageComprehension.cc
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

#include "LanguageComprehension.h"
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include <opencog/guile/SchemeEval.h>
#include <opencog/atomspace/SimpleTruthValue.h>

using namespace OperationalPetController;
using namespace opencog;

LanguageComprehension::LanguageComprehension( Control::PetInterface& agent ) : agent( agent )
{
}

LanguageComprehension::~LanguageComprehension( void )
{
}

void LanguageComprehension::init( void )
{
    static bool initialized = false;
    if ( !initialized ) {
        initialized = true;
        std::stringstream script;
        script << "(define agentSemeNode (SemeNode \"";
        script << agent.getPetId( ) << "\") )" << std::endl;
        std::string answer = SchemeEval::instance().eval( script.str( ) );
        
        if ( SchemeEval::instance().eval_error() ) {
            logger().error( "LanguageComprehension::%s - An error occurred while trying to setup the agent seme node: %s",
                            __FUNCTION__, answer.c_str( ) );
        } // if
        
        SchemeEval::instance().clear_pending( );        
    } // if
}

void LanguageComprehension::solveLatestSentenceReference( void )
{
    init();

    std::string answer = SchemeEval::instance().eval( "(solve-reference)");    
    logger().info( "LanguageComprehension::%s - (solve-reference) answer: %s", __FUNCTION__, answer.c_str() );
    if ( SchemeEval::instance().eval_error() ) {
        logger().error( "LanguageComprehension::%s - An error occurred while trying to solve reference: %s",
                        __FUNCTION__, answer.c_str( ) );
    } // if
    SchemeEval::instance().clear_pending( );    
}

void LanguageComprehension::answerLatestQuestion( void )
{
    init();

    std::string answer = SchemeEval::instance().eval( "(answer-question)");    
    logger().info( "LanguageComprehension::%s - (answer-question) answer: %s", __FUNCTION__, answer.c_str() );
    if ( SchemeEval::instance().eval_error() ) {
        logger().error( "LanguageComprehension::%s - An error occurred while trying to solve reference: %s",
                        __FUNCTION__, answer.c_str( ) );
    } // if
    SchemeEval::instance().clear_pending( );
}


void LanguageComprehension::solveLatestSentenceCommand( void )
{
    init();

    std::string answer = SchemeEval::instance().eval( "(solve-command)");
    logger().info( "LanguageComprehension::%s - (solve-command) answer: %s", __FUNCTION__, answer.c_str() );
    if ( SchemeEval::instance().eval_error() ) {
        logger().error( "LanguageComprehension::%s - An error occurred while trying to solve command: %s",
                        __FUNCTION__, answer.c_str( ) );
    } // if
    SchemeEval::instance().clear_pending( );

    
    opencog::AtomSpace& as = agent.getAtomSpace( );
    HandleSeq commands(2);
    commands[0] = as.addNode( PREDICATE_NODE, "latestAvatarRequestedCommands" );
    commands[1] = Handle::UNDEFINED;
    
    Type types[] = {PREDICATE_NODE, LIST_LINK };
    HandleSeq evalLinks;
    as.getHandleSet( back_inserter(evalLinks),
                     commands, &types[0], NULL, 2, EVALUATION_LINK, false );
    logger().debug( "LanguageComprehension::%s - # of detected predicates 'latestAvatarRequestedCommands': %d", 
                    __FUNCTION__, evalLinks.size( ) );

    // only one eval link must has a tv = true
    unsigned int k;
    bool activeEvalLinkFound = false;
    for( k = 0; k < evalLinks.size(); ++k ) {

        Handle latestEvalLink = evalLinks[k];
        if ( as.getTV( latestEvalLink ).isNullTv( ) || as.getTV( latestEvalLink ).getMean( ) == 0 ) {
            continue;
        } // if

        if ( activeEvalLinkFound ) {
            std::stringstream msg;
            msg << "LanguageComprehension::" << __FUNCTION__ 
                << " - An active EvalLink was already handled. "
                << "You must keep active (TruthValue = TRUE) only ONE EvalLink that makes reference to the "
                << "latestAvatarRequestedCommands predicate.";
        
            logger().error( msg.str( ).c_str( ) );

            return;
        } // if

        activeEvalLinkFound = true;

        Handle commandsList = as.getOutgoing( latestEvalLink, 1);
        unsigned int numberOfCommands = as.getArity( commandsList );
        unsigned int i;
        for( i = 0; i < numberOfCommands; ++i ) {
            Handle execLink = as.getOutgoing( commandsList, i );
            if (EXECUTION_LINK != as.getType( execLink ) ) {
                logger().error( "LanguageComprehension::%s - Only Execution links are allowed to be here.",
                                __FUNCTION__ );
                return;
            } // if
            if ( as.getArity( execLink ) != 2 ) {
                logger().error( "LanguageComprehension::%s - Malformed Execution link. It should has 2 outgoings but %d was found.", __FUNCTION__, as.getArity( execLink ) );
                return;
            } // if
            Handle gsn = as.getOutgoing(execLink, 0 );
            Handle argumentsList = as.getOutgoing(execLink, 1 );
            if ( GROUNDED_SCHEMA_NODE != as.getType( gsn ) || LIST_LINK != as.getType(argumentsList) ) {
                logger().error( "LanguageComprehension::%s - Malformed Execution link. It should link a GroundedSchemaNode and a ListLink of arguments. But types are 0 -> %d and 1 -> %d", __FUNCTION__, as.getType( gsn ), as.getType(argumentsList) );
                return;
            } // if
            std::vector<std::string> arguments;
            arguments.push_back( as.getName( gsn ) );
            unsigned int j;
            unsigned int numberOfArguments = as.getArity( argumentsList );
            for( j = 0; j < numberOfArguments; ++j ) {
                arguments.push_back( as.getName( as.getOutgoing( argumentsList, j ) ) );
            } // for
            
            std::stringstream argsDump;
            std::copy( arguments.begin( ), arguments.end( ), std::ostream_iterator<std::string>(argsDump, " ") );
            logger().debug( "LanguageComprehension::%s - A new schema to be executed was detected: '%s'",
                            __FUNCTION__, argsDump.str( ).c_str( ) );
            
            agent.getCurrentModeHandler( ).handleCommand( "requestedCommand", arguments );
        } // for
    } // if
}

std::string LanguageComprehension::resolveFrames2Relex( )
{
    std::vector < std::pair<std::string, Handle> > handles; 
    std::set< std::string > pre_conditions;

    std::map<std::string, unsigned int> frame_elements_count;

    opencog::AtomSpace& as = agent.getAtomSpace( );

    HandleSeq predicateHandles(2);
    predicateHandles[0] = as.getHandle( PREDICATE_NODE, "latestQuestionFrames" );
    predicateHandles[1] = Handle::UNDEFINED;
    
    Type types[] = {PREDICATE_NODE, LIST_LINK };
    HandleSeq evalLinks;
    as.getHandleSet( back_inserter(evalLinks),
                     predicateHandles, &types[0], NULL, 2, EVALUATION_LINK, false );
    logger().debug( "LanguageComprehension::%s - # of detected predicates 'latestQuestionFrames': %d", 
                    __FUNCTION__, evalLinks.size( ) );

    // only one eval link must has a tv = true
    unsigned int i;
    for( i = 0; i < evalLinks.size(); ++i ) {

        Handle latestEvalLink = evalLinks[i];
        if ( as.getTV( latestEvalLink ).isNullTv( ) || as.getTV( latestEvalLink ).getMean( ) == 0 ) {
            continue;
        } // if
       
        Handle elementsList = as.getOutgoing( latestEvalLink, 1);

        unsigned int j;
        unsigned int numberOfElements = as.getArity( elementsList );
        for( j = 0; j < numberOfElements; ++j ) {
            Handle predicateElement = as.getOutgoing( elementsList, j );

            //get the frame name
            std::string frameName;
            { // check the inheritance
                HandleSeq inheritanceLink;
                inheritanceLink.push_back( predicateElement );
                inheritanceLink.push_back( Handle::UNDEFINED );

                Type inheritanceLinkTypes[] = { PREDICATE_NODE, DEFINED_FRAME_NODE };
                HandleSeq inheritanceLinks;
                as.getHandleSet( back_inserter( inheritanceLinks ),
                                        inheritanceLink,
                                        &inheritanceLinkTypes[0], NULL, 2, INHERITANCE_LINK, false );        
                if ( inheritanceLinks.size( ) > 0 ) {
                    // ok it is a frame instance
                    if ( inheritanceLinks.size( ) > 1 ) {
                        logger().error(
                           "LanguageComprehension::%s - The given handle represents more than one instance of Frame, what is unacceptable. Only the first occurrence will be considered.", __FUNCTION__ );                
                    } // if
                    frameName = as.getName( as.getOutgoing( inheritanceLinks[0], 1 ) );
                    logger().debug( "LanguageComprehension::%s - FrameName detected: %s", 
                                                                __FUNCTION__, frameName.c_str() );                    
                } else {
                    logger().debug(
                        "LanguageComprehension::%s - The given handle isn't a Frame instance. It doesn't inherits from a DEFINED_FRAME_NODE: %s", __FUNCTION__, as.getName( predicateElement ).c_str( ) );
                    return "";
                } // else
            } // end block


            std::map<std::string, Handle> elements =
                    AtomSpaceUtil::getFrameInstanceElementsValues( as, predicateElement );
            
            logger().debug( "LanguageComprehension::%s - # of elements found: %d", 
                            __FUNCTION__, elements.size());

            std::map<std::string, Handle>::iterator it;
            for(it = elements.begin(); it != elements.end(); ++it) {
                std::string frameElementName = frameName+":"+it->first;
                handles.push_back( std::pair<std::string, Handle>(frameElementName, it->second ) );
               //count the number of times each element appears to be part of
                //the pre-conditions
                if( frame_elements_count[frameElementName] > 0 ){
                    frame_elements_count[frameElementName] = frame_elements_count[frameElementName] + 1;
                }else{
                    frame_elements_count[frameElementName] = 1;
                }
            }//for
       }//for
       //set the TV of the list to NULL
       as.setTV( latestEvalLink, SimpleTruthValue(0.0f, 0.0f) );
    }
  

    //iterate the frame_elements_count to get the preconditions appended with
    //the element count, like: #Color:Color_2, #Entity:Entity_1, and so on
    std::map< std::string, unsigned int>::const_iterator iter;
    for( iter = frame_elements_count.begin(); iter != frame_elements_count.end(); ++iter) {
        std::string precondition = iter->first+"$"+boost::lexical_cast<std::string>(iter->second);
        //std::cout << "Pre-Condition: " << precondition << std::endl;
        logger().debug( "LanguageComprehension::%s - Pre-Condition detected: %s", 
                                                                __FUNCTION__, precondition.c_str() );
        pre_conditions.insert( precondition );
    }

   logger().debug("LanguageComprehension::%s - Pre-Conditions",__FUNCTION__);
   std::set< std::string >::iterator it;
   for(it = pre_conditions.begin(); it != pre_conditions.end(); ++it){
        logger().debug("LanguageComprehension::%s - Pre-Condition: %s",__FUNCTION__,(*it).c_str());
   }
   logger().debug("LanguageComprehension::%s - End of Pre-Conditions",__FUNCTION__);   
   
   OutputRelex* output_relex = FramesToRelexRuleEngine::instance().resolve( pre_conditions );
   if( output_relex == NULL ){
       logger().debug("LanguageComprehension::%s - Output Relex is NULL for the Pre-Conditions",__FUNCTION__);
       return "";
   }

   return output_relex->getOutput( as, handles );

}
