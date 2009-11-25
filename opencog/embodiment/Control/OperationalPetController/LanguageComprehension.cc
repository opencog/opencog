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

#include <opencog/embodiment/Control/OperationalPetController/LanguageComprehension.h>
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/embodiment/Control/EmbodimentConfig.h>
#include <boost/regex.hpp>
#include <opencog/guile/SchemeSmob.h>

using namespace OperationalPetController;
using namespace opencog;


class ComputeSpatialRelations : public SchemeFunction {
public:
    
    ComputeSpatialRelations( Control::PetInterface& agent ) :
        SchemeFunction( 
            "cog-emb-compute-spatial-relations", 3, 0, 1 ) { localAgent = &agent; }

    virtual FunctionPointer getFunctionPointer( void ) {
        return (SchemeFunction::FunctionPointer)&ComputeSpatialRelations::execute;
    }

private:
    
    static SCM execute(SCM objectObserver, SCM figureSemeNode, SCM groundSemeNode, SCM ground2SemeNode ) {
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
            msg << "observer[" << TLB::getAtom( observer )->toString( ) << "] ";
            msg << "objectA[" << TLB::getAtom( objectA )->toString( ) << "] ";
            msg << "objectB[" << TLB::getAtom( objectB )->toString( ) << "] ";
            if ( objectC != Handle::UNDEFINED ) {
                msg << "objectC[" << atomSpace.getName( objectC ) << "] ";
            } // if            
            logger().debug( "ComputeSpatialRelations::%s - Computing spatial relations for '%s'",
                            __FUNCTION__, msg.str( ).c_str( ) );
        }


        std::vector<std::string> entitiesA;
        std::vector<std::string> entitiesB;
        std::vector<std::string> entitiesC;

        if ( atomSpace.getType( objectA ) == VARIABLE_NODE ) {
            spaceMap.getAllObjects( std::back_inserter( entitiesA ) );
        } else {
            HandleSeq incoming = atomSpace.getIncoming( objectA );
            unsigned int i;
            for( i = 0; i < incoming.size( ); ++i ) {
                Handle firstElement = atomSpace.getOutgoing(incoming[i], 0 );
                if ( atomSpace.inheritsType( atomSpace.getType(firstElement), OBJECT_NODE ) ) {
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
                if ( atomSpace.inheritsType( atomSpace.getType(firstElement), OBJECT_NODE ) ) {
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
                    if ( atomSpace.inheritsType( atomSpace.getType(firstElement), OBJECT_NODE ) ) {
                        entitiesC.push_back( atomSpace.getName( firstElement ) );
                    } // if
                } // for
            } // else            
        } // if

        logger().debug( "ComputeSpatialRelations::%s - %d candidates for objectA. %d candidates for objectB. %d candidates for objectC",
                        __FUNCTION__, entitiesA.size( ), entitiesB.size( ), entitiesC.size( ) );

        try {
            const Spatial::EntityPtr& observerEntity = spaceMap.getEntity( atomSpace.getName( observer ) );
            
            unsigned int i, j, k;
            for( i = 0; i < entitiesA.size( ); ++i ) {
                const Spatial::EntityPtr& entityA = spaceMap.getEntity( entitiesA[i] );
                for( j = 0; j < entitiesB.size( ); ++j ) {
                    if ( entitiesA[i] == entitiesB[j] ) {
                        continue;
                    } // if
                    const Spatial::EntityPtr& entityB = spaceMap.getEntity( entitiesB[j] );
                    if ( entitiesC.size( ) > 0 ) {
                        for( k = 0; k < entitiesC.size( ); ++k ) {
                            if ( entitiesA[i] == entitiesC[k] || entitiesB[j] == entitiesC[k] ) {
                                continue;
                            } // if
                            const Spatial::EntityPtr& entityC = spaceMap.getEntity( entitiesC[k] );
                            createFrameInstancesFromRelations( atomSpace, resultingFrames,
                               spaceMap.computeSpatialRelations( observerEntity->getPosition( ), besideDistance, *entityA, *entityB, *entityC ),
                                  entitiesA[i], entitiesB[j], entitiesC[k] );
                        } // for
                    } else {
                            createFrameInstancesFromRelations( atomSpace, resultingFrames,
                               spaceMap.computeSpatialRelations( observerEntity->getPosition( ), besideDistance, *entityA, *entityB ),
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

    static void createFrameInstancesFromRelations( AtomSpace& atomSpace, HandleSeq& resultingFrames,
        const std::list<Spatial::LocalSpaceMap2D::SPATIAL_RELATION>& relations,
           const std::string& objectA, const std::string& objectB, const std::string& objectC ) {

        std::list<Spatial::LocalSpaceMap2D::SPATIAL_RELATION>::const_iterator it;
        for( it = relations.begin( ); it != relations.end( ); ++it ) {
            std::string relationName = Spatial::LocalSpaceMap2D::spatialRelationToString( *it );

            std::map<std::string, Handle> elements;
            elements["Figure"] = atomSpace.getHandle( SEME_NODE, objectA );
            elements["Ground"] = atomSpace.getHandle( SEME_NODE, objectB );
            elements["Relation_type"] = atomSpace.addNode( CONCEPT_NODE, relationName );

            std::stringstream instanceName;
            instanceName << objectA;
            instanceName << "_";
            instanceName << objectB;

            if ( *it == Spatial::LocalSpaceMap2D::BETWEEN ) {
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

    static Control::PetInterface* localAgent;

};

Control::PetInterface* ComputeSpatialRelations::localAgent = NULL;

LanguageComprehension::LanguageComprehension( Control::PetInterface& agent ) : 
    agent( agent ), spatialRelationsEvaluatorCaller( NULL )
{
}

LanguageComprehension::~LanguageComprehension( void )
{
    delete nlgenClient;
    if ( this->spatialRelationsEvaluatorCaller != NULL ) {
        delete this->spatialRelationsEvaluatorCaller;
        this->spatialRelationsEvaluatorCaller = NULL;
    } // if
}

void LanguageComprehension::init( void )
{
    static bool initialized = false;
    if ( !initialized ) {
        
        this->spatialRelationsEvaluatorCaller = new ComputeSpatialRelations( this->agent );
        scm_c_define_gsubr( this->spatialRelationsEvaluatorCaller->getName( ).c_str( ),
                  this->spatialRelationsEvaluatorCaller->getNumberOfRequiredArguments( ),
                  this->spatialRelationsEvaluatorCaller->getNumberOfOptionalArguments( ),
                  this->spatialRelationsEvaluatorCaller->getNumberOfRestArguments( ),
                  this->spatialRelationsEvaluatorCaller->getFunctionPointer( )
                  );

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

        this->nlgen_server_port = config().get_int("NLGEN_SERVER_PORT");
        this->nlgen_server_host = config().get("NLGEN_SERVER_HOST");

        nlgenClient = new NLGenClient(this->nlgen_server_host, this->nlgen_server_port);

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

    boost::trim(answer);

    std::string finalSentence;
    if ( answer == "#truth-query" ) {
        // yes/no question
        HandleSeq elements = getActivePredicateArguments( "latestQuestionFrames" );
        finalSentence = ( elements.size( ) > 0 ? "Yes" : "No" );
    } else {
        std::string relations = resolveFrames2Relex( );
        // call nlgen using relations
        finalSentence = relations;
    } // else

    if ( finalSentence.length( ) > 0 ) {
        agent.getCurrentModeHandler( ).setProperty( "customMessage", finalSentence );
        AtomSpaceUtil::setPredicateValue( agent.getAtomSpace( ), "has_something_to_say", TruthValue::TRUE_TV( ),
                                          AtomSpaceUtil::getAgentHandle( agent.getAtomSpace( ), agent.getPetId( ) ) ); 
    } // if

}

HandleSeq LanguageComprehension::getActivePredicateArguments( const std::string& predicateName ) 
{
    opencog::AtomSpace& as = agent.getAtomSpace( );
    HandleSeq commands(2);
    commands[0] = as.addNode( PREDICATE_NODE, predicateName );
    commands[1] = Handle::UNDEFINED;
    
    Type types[] = {PREDICATE_NODE, LIST_LINK };
    HandleSeq evalLinks;
    as.getHandleSet( back_inserter(evalLinks),
                     commands, &types[0], NULL, 2, EVALUATION_LINK, false );
    logger().debug( "LanguageComprehension::%s - # of detected predicates '%s': %d", 
                    __FUNCTION__, predicateName.c_str( ), evalLinks.size( ) );

    HandleSeq elements;
    bool activeEvalLinkFound = false;
    unsigned int k;
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
                << predicateName << " predicate.";
        
            logger().error( msg.str( ).c_str( ) );
            continue;
        } // if

        activeEvalLinkFound = true;
        logger().debug( "LanguageComprehension::%s - Chosen link '%s'", 
                        __FUNCTION__, TLB::getAtom( evalLinks[k] )->toString( ).c_str( ) );
        elements = as.getOutgoing(as.getOutgoing( evalLinks[k], 1 ));
    } // for
    return elements;
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
    HandleSeq elements = getActivePredicateArguments( "latestAvatarRequestedCommands" );

    unsigned int i;
    for( i = 0; i < elements.size( ); ++i ) {
        Handle execLink = elements[i];
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

}

std::string LanguageComprehension::resolveFrames2Relex( )
{
    std::vector < std::pair<std::string, Handle> > handles; 
    std::set< std::string > pre_conditions;

    std::map<std::string, unsigned int> frame_elements_count;

    opencog::AtomSpace& as = agent.getAtomSpace( );
    HandleSeq elements = getActivePredicateArguments( "latestQuestionFrames" );

    if ( elements.size( ) == 0 ) {
        // there is no question to answer
        logger().debug( "LanguageComprehension::%s - elements size of the answer is 0. 'I don't know' answer will be reported.",
                        __FUNCTION__ );
        return "I don't know.";
    } // if

    unsigned int j;
    for( j = 0; j < elements.size(); ++j ) {
        Handle predicateElement = elements[j];

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
            
        logger().debug( "LanguageComprehension::%s - # of elements found: '%d' for predicate: '%s'", 
                        __FUNCTION__, elements.size(), TLB::getAtom(predicateElement)->toString().c_str( ) );
        
        
        
        std::map<std::string, Handle>::iterator it;
        for(it = elements.begin(); it != elements.end(); ++it) {
            logger().debug( "LanguageComprehension::%s - Inspecting element: '%s' atom: '%s'", 
                            __FUNCTION__, it->first.c_str(), TLB::getAtom(it->second)->toString( ).c_str()  );
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
  

    //iterate the frame_elements_count to get the preconditions appended with
    //the element count, like: #Color:Color_2, #Entity:Entity_1, and so on
    std::map< std::string, unsigned int>::const_iterator iter;
    for( iter = frame_elements_count.begin(); iter != frame_elements_count.end(); ++iter) {
        std::string precondition = iter->first+"$"+boost::lexical_cast<std::string>(iter->second);
        logger().debug( "LanguageComprehension::%s - Pre-Condition detected: '%s'", 
                        __FUNCTION__, precondition.c_str() );
        pre_conditions.insert( precondition );
    }
    
    logger().debug("LanguageComprehension::%s - Pre-Conditions",__FUNCTION__);
    std::set< std::string >::iterator it;
    for(it = pre_conditions.begin(); it != pre_conditions.end(); ++it){
        logger().debug("LanguageComprehension::%s - Pre-Condition: %s",__FUNCTION__,(*it).c_str());
    }
    logger().debug("LanguageComprehension::%s - End of Pre-Conditions",__FUNCTION__);   
    
    OutputRelex* output_relex = framesToRelexRuleEngine.resolve( pre_conditions );
    if( output_relex == NULL ){
        logger().debug("LanguageComprehension::%s - Output Relex is NULL for the pre-conditions. No rules were found.",__FUNCTION__);
        return "I know it, but I don't know how to say it.";//TODO return empty or relex string
    }
    
    std::string text = output_relex->getOutput( as, handles );
    if( text.empty() ){
        logger().error("LanguageComprehension::%s - Output Relex returned an empty string.",__FUNCTION__);
        return "";
    }

    return resolveRelex2Sentence( text );
}

std::string LanguageComprehension::resolveRelex2Sentence( const std::string& relexInput ) 
{
    
    //connect to the NLGen server and try to get the sentence from the relex
    //content
    std::string nlgen_sentence = nlgenClient->send(relexInput);
    logger().debug("LanguageComprehension::%s - NLGen Sentence: %s",__FUNCTION__,nlgen_sentence.c_str() );
    
    if( nlgen_sentence.empty() ){
        logger().debug("LanguageComprehension::%s - NLGen Sentence is EMPTY. Probably it was not possible to connect to the NLGen server socket on host %s and port %d",__FUNCTION__, nlgen_server_host.c_str(), nlgen_server_port);
        return relexInput;
    }
    //TODO check NLGEn error codes
    if(boost::starts_with(nlgen_sentence, "ERROR")){
        //TODO get the code and message
        const boost::regex errorRegex( "ERROR\\[(\\d)\\](.*)");
        boost::cmatch matches;
        if ( boost::regex_match( nlgen_sentence.c_str( ), matches, errorRegex ) ) {
            std::string code = matches[1];
            std::string msg = matches[2];
            logger().debug("LanguageComprehension::%s - NLGen Sentence returned an ERROR message with code %s and message %s",__FUNCTION__, code.c_str(), msg.c_str());
            if( code == "1"){
                return "I don't know!";
            }else if( code == "2" ){
                return "I know the answer, but I don't know how to say it.";
            }
        }else{
            logger().debug("LanguageComprehension::%s - NLGen Sentence returned an ERROR but it was not possible to get the code. Error: %s",__FUNCTION__, nlgen_sentence.c_str());
        }
        return relexInput;
    }
    
    
    return nlgen_sentence;
    
}
