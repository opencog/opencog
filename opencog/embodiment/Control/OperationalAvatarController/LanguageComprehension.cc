/*
 * opencog/embodiment/Control/OperationalAvatarController/LanguageComprehension.cc
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

#include <boost/regex.hpp>
#include <boost/bind.hpp>
#include <boost/asio/ip/tcp.hpp>

using namespace opencog::oac;
using namespace opencog::spatial;
using namespace opencog;

opencog::control::AvatarInterface* LanguageComprehension::localAgent = NULL;

///////////////////////////////////////////////////////////////////////////////
//
// Core functions of LanguageComprehension

LanguageComprehension::LanguageComprehension( opencog::control::AvatarInterface& agent ) : 
    agent( agent ), nlgenClient( NULL ), initialized( false )
{

}

LanguageComprehension::~LanguageComprehension( void )
{
    if ( nlgenClient != NULL ) {
        delete nlgenClient;
        nlgenClient = NULL;
    } 
}

void LanguageComprehension::handleCommand(const std::string& name, const std::vector<std::string>& arguments)
{
    if ( name == "requestedCommand" ) {
        if ( arguments.size() == 0 ) {
            logger().warn("LanguageComprehension::%s (%s) - command[requestedCommand] Invalid number of arguments: 0", 
                          __FUNCTION__, 
                          name.c_str()
                         ); 
            return;
        }

        logger().debug("LanguageComprehension::%s (%s) - Scheduling command",
                        __FUNCTION__, 
                        name.c_str()
                      ); 
        this->commandsQueue.push(arguments);

    } 
    else if ( name == "updateFact" ) {
        logger().debug("LanguageComprehension::%s (%s) - Evaluating a new parsed sentence", 
                       __FUNCTION__, 
                       name.c_str()
                      ); 


        logger().debug("LanguageComprehension::%s (%s) - Starting latest sentence reference resolution", 
                       __FUNCTION__, 
                       name.c_str()
                      ); 
        this->resolveLatestSentenceReference();
        logger().debug("LanguageComprehension::%s (%s) - Reference resolution done", 
                       __FUNCTION__, 
                       name.c_str()
                      ); 

        logger().debug("LanguageComprehension::%s (%s) - Starting storing a new fact", 
                       __FUNCTION__, 
                       name.c_str()
                      );
        this->updateFact();
        logger().debug("LanguageComprehension::%s (%s) - Fact stored",
                       __FUNCTION__, 
                       name.c_str()
                      );
        
    }
    else if ( name == "evaluateSentence" ) {
        logger().debug("LanguageComprehension::%s (%s) - Evaluating a new parsed sentence", 
                       __FUNCTION__, 
                       name.c_str()
                      );

        logger().debug("LanguageComprehension::%s (%s) - Starting latest sentence reference resolution", 
                       __FUNCTION__, 
                       name.c_str()
                      );
        this->resolveLatestSentenceReference();
        logger().debug("LanguageComprehension::%s (%s) Reference resolution done", 
                       __FUNCTION__, 
                       name.c_str()
                      );

        logger().debug("LanguageComprehension::%s (%s) - Starting latest sentence command resolution", 
                       __FUNCTION__, 
                       name.c_str()
                      );
        this->resolveLatestSentenceCommand();
        logger().debug("LanguageComprehension::%s (%s) - Command resolution done", 
                       __FUNCTION__, 
                       name.c_str()
                      );

    } 
    else if ( name == "answerQuestion") {
        logger().debug("LanguageComprehension::%s (%s) - Answering a question", 
                        __FUNCTION__, 
                        name.c_str()
                      );

        logger().debug("LanguageComprehension::%s (%s) - Starting latest sentence reference resolution", 
                       __FUNCTION__, 
                       name.c_str()
                      );
        this->resolveLatestSentenceReference();
        logger().debug("LanguageComprehension::%s (%s) - Reference resolution done", 
                       __FUNCTION__, 
                       name.c_str()
                      );
        
    }
}

void LanguageComprehension::init( void )
{
    if ( !initialized ) {
        initialized = true;

        // Ensure SchemeEval is initialised with AtomSpace.
        opencog::AtomSpace& as = this->agent.getAtomSpace();
        SchemeEval::instance(&as);
        
#ifdef HAVE_GUILE
        std::stringstream script;
        script << "(define agentSemeNode (SemeNode \"";
        script << agent.getPetId( ) << "\") )" << std::endl;
        std::string answer = SchemeEval::instance().eval( script.str( ) );
        if ( SchemeEval::instance().eval_error() ) {
            logger().error( "LanguageComprehension::%s - An error occurred while trying to setup the agent seme node: %s",
                            __FUNCTION__, answer.c_str( ) );
        } // if
        SchemeEval::instance().clear_pending( );

        loadFrames( );

        LanguageComprehension::localAgent = &this->agent;
        scm_c_define_gsubr("cog-emb-compute-spatial-relations", 3, 0, 1, 
                           (SCM (*)())&LanguageComprehension::execute );
#endif
                
        this->nlgen_server_port = config().get_int("NLGEN_SERVER_PORT");
        this->nlgen_server_host = config().get("NLGEN_SERVER_HOST");

        nlgenClient = new NLGenClient(this->nlgen_server_host, this->nlgen_server_port);
        loadDialogControllers( );

    } // if
}

void LanguageComprehension::resolveLatestSentenceReference( void )
{
    init();

#ifdef HAVE_GUILE
    std::string answer = SchemeEval::instance().eval( "(resolve-reference)");
    logger().info( "LanguageComprehension::%s - (resolve-reference) answer: %s", __FUNCTION__, answer.c_str() );
    if ( SchemeEval::instance().eval_error() ) {
        logger().error( "LanguageComprehension::%s - An error occurred while trying to resolve reference: %s",
                        __FUNCTION__, answer.c_str( ) );
    } // if
    SchemeEval::instance().clear_pending();
#endif
}

HandleSeq LanguageComprehension::getActivePredicateArguments( const std::string& predicateName ) 
{
    opencog::AtomSpace& as = this->agent.getAtomSpace();
    HandleSeq commands(2);
    commands[0] = as.addNode( PREDICATE_NODE, predicateName );
    commands[1] = Handle::UNDEFINED;
    
    Type types[] = {PREDICATE_NODE, LIST_LINK };
    HandleSeq evalLinks;
    as.getHandleSet( back_inserter(evalLinks),
                     commands, &types[0], NULL, 2, EVALUATION_LINK, false );
    logger().debug( "LanguageComprehension::%s - # of EvalLinks for '%s': %d",
                    __FUNCTION__, predicateName.c_str(), evalLinks.size() );

    HandleSeq elements;
    bool activeEvalLinkFound = false;
    unsigned int k;
    for( k = 0; k < evalLinks.size(); ++k ) {
        Handle latestEvalLink = evalLinks[k];
        TruthValuePtr tvp = as.getTV(latestEvalLink);
        if ( tvp->isNullTv() || tvp->getMean() == 0 ) {
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
                        __FUNCTION__, as.atomAsString( evalLinks[k] ).c_str() );
        elements = as.getOutgoing(as.getOutgoing( evalLinks[k], 1 ));
    } // for
    return elements;
}

void LanguageComprehension::resolveLatestSentenceCommand( void )
{
    init();

#ifdef HAVE_GUILE
    std::string answer = SchemeEval::instance().eval( "(resolve-command)");
    logger().info( "LanguageComprehension::%s - (resolve-command) answer: %s", __FUNCTION__, answer.c_str() );
    if ( SchemeEval::instance().eval_error() ) {
        logger().error( "LanguageComprehension::%s - An error occurred while trying to resolve command: %s",
                        __FUNCTION__, answer.c_str( ) );
    } // if
    SchemeEval::instance().clear_pending();
#endif
    
    opencog::AtomSpace& as = agent.getAtomSpace();
    HandleSeq elements = getActivePredicateArguments( "latestAvatarRequestedCommands" );

    unsigned int i;
    for( i = 0; i < elements.size(); ++i ) {
        Handle execLink = elements[i];
        if (EXECUTION_LINK != as.getType( execLink ) ) {
            logger().error( "LanguageComprehension::%s - Only Execution links are allowed to be here.",
                            __FUNCTION__ );
            return;
        } // if
        if ( as.getArity( execLink ) != 2 ) {
            logger().error( "LanguageComprehension::%s - Malformed Execution link. It should has 2 outgoings but %d was found.", 
                             __FUNCTION__, as.getArity(execLink) );
            return;
        } // if
        Handle gsn = as.getOutgoing(execLink, 0);
        Handle argumentsList = as.getOutgoing(execLink, 1 );
        if ( GROUNDED_SCHEMA_NODE != as.getType(gsn) || LIST_LINK != as.getType(argumentsList) ) {
            logger().error( "LanguageComprehension::%s - Malformed Execution link. It should link a GroundedSchemaNode and a ListLink of arguments. But types are 0 -> %d and 1 -> %d", 
                            __FUNCTION__, as.getType( gsn ), as.getType(argumentsList) );
            return;
        } // if
        std::vector<std::string> arguments;
        arguments.push_back( as.getName(gsn) );
        unsigned int j;
        unsigned int numberOfArguments = as.getArity( argumentsList );
        for( j = 0; j < numberOfArguments; ++j ) {
            arguments.push_back( as.getName( as.getOutgoing( argumentsList, j ) ) );
        } // for
        
        std::stringstream argsDump;
        std::copy( arguments.begin(), arguments.end(), std::ostream_iterator<std::string>(argsDump, " ") );
        logger().debug( "LanguageComprehension::%s - A new schema to be executed was detected: '%s'",
                        __FUNCTION__, argsDump.str().c_str() );
        
        agent.getCurrentModeHandler().handleCommand( "requestedCommand", arguments );
    } // for
}

std::string LanguageComprehension::resolveFrames2Relex( )
{
    init();

    std::vector < std::pair<std::string, Handle> > handles; 
    std::set< std::string > pre_conditions;

    std::map<std::string, unsigned int> frame_elements_count;

    opencog::AtomSpace& as = agent.getAtomSpace( );
    HandleSeq elements = getActivePredicateArguments( "latestQuestionFrames" );

    if ( elements.size( ) == 0 ) {
        // there is no question to answer
        logger().debug( "LanguageComprehension::%s - elements size of the answer is 0. 'I don't know' answer will be reported.",
                        __FUNCTION__ );
        return "";
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
                        __FUNCTION__, elements.size(), as.atomAsString(predicateElement).c_str() );
        
        std::map<std::string, Handle>::iterator it;
        for(it = elements.begin(); it != elements.end(); ++it) {
            logger().debug( "LanguageComprehension::%s - Inspecting element: '%s' atom: '%s'", 
                            __FUNCTION__, it->first.c_str(), as.atomAsString(it->second).c_str() );
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
        return "I know the answer, but I don't know how to say it. [Frames2Relex did not have a suitable rule]";
    }
    
    std::string text = output_relex->getOutput( as, handles );
    if( text.empty() ){
        logger().error("LanguageComprehension::%s - Output Relex returned an empty string.",__FUNCTION__);
        return "...";
    }

    return resolveRelex2Sentence(text);
}

std::string LanguageComprehension::resolveRelex2Sentence( const std::string& relexInput ) 
{
    init();
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
                return "I know the answer, but I don't know how to say it. [NLGen could not match the Relex output to a sentence]";
            }else if( code == "2" ){
                return "I know the answer, but I don't know how to say it. [Frames2Relex generated malformed output]";
            }
        }else{
            logger().debug("LanguageComprehension::%s - NLGen Sentence returned an ERROR but it was not possible to get the code. Error: %s",__FUNCTION__, nlgen_sentence.c_str());
        }
        return relexInput;
    }
    
    
    return nlgen_sentence;
    
}

void LanguageComprehension::updateFact( void )
{
    init();

#ifdef HAVE_GUILE
    std::string answer = SchemeEval::instance().eval( "(update-fact)");    

    logger().info( "LanguageComprehension::%s - (update-fact) newly created or deleted frame instances (fact): %s", 
                   __FUNCTION__, 
                   answer.c_str()
                 );

    if ( SchemeEval::instance().eval_error() ) {
        logger().error( "LanguageComprehension::%s - (update-fact) An error occurred while trying to update fact: %s",
                        __FUNCTION__, 
                        answer.c_str() 
                      );
    } // if

    SchemeEval::instance().clear_pending( );    
#endif
}

HandleSeq LanguageComprehension::getHeardSentencePredicates( void )
{
    AtomSpace& atomSpace = agent.getAtomSpace( );
    Handle agentHandle = AtomSpaceUtil::getAgentHandle( atomSpace, agent.getPetId( ) );

    HandleSeq heardSentences;
    
    Handle node = atomSpace.getHandle( PREDICATE_NODE, "heard_sentence" );
    if ( node == Handle::UNDEFINED ) {
        return heardSentences;
    } // if
    HandleSeq incoming = atomSpace.getIncoming( node );
    unsigned int i;
    for( i = 0; i < incoming.size( ); ++i ) {
        if ( atomSpace.getType( incoming[i] ) != EVALUATION_LINK ||
             atomSpace.getTV( incoming[i] )->isNullTv() || 
             atomSpace.getMean( incoming[i] ) == 0 ||
             atomSpace.getArity( incoming[i] ) != 2 ||
             atomSpace.getOutgoing( incoming[i], 1 ) == node ) {
            continue;
        } // if
        Handle listLink = atomSpace.getOutgoing( incoming[i], 1 );
        Handle sentenceNode = atomSpace.getOutgoing( listLink, 0 );
        if ( atomSpace.getType( sentenceNode ) != SENTENCE_NODE ) {
            continue;
        } // if
        // "to:agentId: sentencetexthere"
        std::string nodeName = atomSpace.getName( sentenceNode );
        static const boost::regex pattern( "^to:[^:]+:\\s.+$");
        if ( !boost::regex_match( nodeName, pattern ) ) {
            logger().error( "LanguageComprehension::%s - Invalid sentence node '%s'. "
                            "Should be in format 'to:agentId: sentencetext'",
                            __FUNCTION__, nodeName.c_str( ) );
        } // if
        heardSentences.push_back( incoming[i] );
    } // for
    
    return heardSentences;
}

std::string LanguageComprehension::getTextFromSentencePredicate( Handle evalLink )
{
    opencog::AtomSpace& atomSpace = agent.getAtomSpace( );    
    Handle predicateNode = atomSpace.getHandle( PREDICATE_NODE, "heard_sentence" );
    if ( evalLink == Handle::UNDEFINED ||
         predicateNode == Handle::UNDEFINED ||
         atomSpace.getType( evalLink ) != EVALUATION_LINK || 
         atomSpace.getArity( evalLink ) != 2 ||
         atomSpace.getOutgoing( evalLink, 0 ) != predicateNode ) {
        return "";
    } // if
    Handle listLink = atomSpace.getOutgoing( evalLink, 1 );
    Handle sentenceNode = atomSpace.getOutgoing( listLink, 0 );

    std::string nodeName = atomSpace.getName( sentenceNode );
    static const boost::regex pattern( "^to:[^:]+:\\s(.+)$");
    boost::smatch matchResults;
    if ( !boost::regex_match( nodeName, matchResults, pattern ) ) {
        return "";
    } // if
    std::string text( matchResults[1].first, matchResults[1].second );
    return text;
}

///////////////////////////////////////////////////////////////////////////////
//
// Functions related to DialogController

void LanguageComprehension::addDialogController( DialogController* dialogController )
{
    if ( dialogController == NULL ) {
        return;
    } // if
    std::list<DialogController*>::const_iterator it;
    for( it = this->dialogControllers.begin( ); it != this->dialogControllers.end( ); ++it ) {
        if ( (*it)->getName( ) == dialogController->getName( ) ) {
            return;
        } // if
    } // for

    logger().debug("LanguageComprehension::%s - New Dialog Controller registered: %s",
                   __FUNCTION__, dialogController->getName( ).c_str( ) );
    
    this->dialogControllers.push_back( dialogController );
}

void LanguageComprehension::updateDialogControllers( long elapsedTime )
{
    AtomSpace& atomSpace = agent.getAtomSpace( );
    Handle agentHandle = 
        AtomSpaceUtil::getAgentHandle( atomSpace, agent.getPetId( ) );

    bool hasSomethingToSay = AtomSpaceUtil::isPredicateTrue( atomSpace,
        "has_something_to_say", agentHandle );
    
    if ( hasSomethingToSay ) {
        // there is already something to be said, so ignore this call
        return;
    } // if

    init();

    std::list<DialogController* >::iterator it;

    HandleSeq sentences; 
    sentences.push_back( atomSpace.addNode( ANCHOR_NODE, "# Possible Sentences" ) );
    for( it = this->dialogControllers.begin( ); it != this->dialogControllers.end( ); ++it ) {
        DialogController::Result candidate = 
            (*it)->processSentence( elapsedTime );

        // get only valid answers
        if ( candidate.status && candidate.sentence.length( ) > 0 ) {
            sentences.push_back( atomSpace.addNode( SENTENCE_NODE, candidate.sentence ) );
        } // if
        
    } // for

    atomSpace.addLink( LIST_LINK, sentences, SimpleTruthValue( 1, 1 ) );

    bool hasQuestionToAnswer = false;    
    { // check if there is a question to be answered
        HandleSeq heardSentences = getHeardSentencePredicates( );
        
        if ( heardSentences.size( ) > 0 ) {
            Handle listLink = atomSpace.getOutgoing( heardSentences.back( ), 1 );
            Handle sentenceNode = atomSpace.getOutgoing( listLink, 0 );
            
            hasQuestionToAnswer |= (AtomSpaceUtil::isPredicateTrue( 
                atomSpace, "is_question", sentenceNode ) &&
                                    !AtomSpaceUtil::isPredicateTrue( 
                atomSpace, "was_answered", sentenceNode ));
        } // if
    }

    std::string finalSentence;

#ifdef HAVE_GUILE
    std::string answer = SchemeEval::instance().eval( "(choose-sentence)");
    logger().debug( "QuestionAnsweringDialogController::%s - (choose-sentence) answer: %s", __FUNCTION__, answer.c_str() );
    if ( SchemeEval::instance().eval_error() ) {
        logger().error( "QuestionAnsweringDialogController::%s - An error occurred while trying to choose a sentence: %s",
                        __FUNCTION__, answer.c_str( ) );
    } else {
        boost::trim( answer );
        if ( answer != "#f" ) {
            finalSentence  = answer;
        } // if
    } // else

    SchemeEval::instance().clear_pending( );
#else
    finalSentence = sentences.size( ) > 0 ? atomSpace.getName( sentences[0] ) : "";
#endif

    { // now set to false all heard sentences
      // and set as answered all heard questions
        HandleSeq heardSentences = getHeardSentencePredicates( );        
        // set the TV of all heard sentences to false
        unsigned int i;
        for( i = 0; i < heardSentences.size( ); ++i ) {
            atomSpace.setTV( heardSentences[i], SimpleTruthValue( 0, 1) );

            Handle listLink = atomSpace.getOutgoing( heardSentences[i], 1 );
            Handle sentenceNode = atomSpace.getOutgoing( listLink, 0 );
    
            if ( AtomSpaceUtil::isPredicateTrue( atomSpace, "is_question", sentenceNode ) ) {
                AtomSpaceUtil::setPredicateValue( atomSpace,
                    "was_answered", TruthValue::TRUE_TV( ), sentenceNode );
            } // if
        } // for
    } // end block

    if ( finalSentence.length( ) == 0 && hasQuestionToAnswer ) {
        // ok, the agent did not find an answer but still there is a pending
        // question, so report its ignorance about the matter
        finalSentence = "I don't know";
    } // if

    if ( finalSentence.length( ) > 0 ) {
        agent.getCurrentModeHandler( ).setProperty( "customMessage", finalSentence );
        // and finally, the agent has something to say
        AtomSpaceUtil::setPredicateValue( atomSpace, "has_something_to_say", 
                                          SimpleTruthValue( 1, 1 ), agentHandle );
    } // if

}

void LanguageComprehension::loadDialogControllers( void )
{
    std::vector<std::string> controllers;
    boost::split( controllers, config().get("DIALOG_CONTROLLERS"), boost::is_any_of(",;") );
    unsigned int i;
    for ( i = 0; i < controllers.size(); ++i ) {
        boost::trim( controllers[i] );
        addDialogController( createDialogController( controllers[i] ) );
    } // for
}


// Controllers classes


/**
 * This DC implementation will look for unanswered questions
 * and execute the internal question answering mechanism
 * to try to answer it
 */
class QuestionAnsweringDialogController : public LanguageComprehension::DialogController 
{
public:
    QuestionAnsweringDialogController( LanguageComprehension* langComp ) :
        LanguageComprehension::DialogController::DialogController( "QuestionAnswering", langComp ) 
    { }

    virtual ~QuestionAnsweringDialogController( void ) 
    { }

    virtual LanguageComprehension::DialogController::Result processSentence( long elapsedTime )
    {
#ifdef HAVE_GUILE

        LanguageComprehension::DialogController::Result result;
        result.questionAnswering = true;

        if ( AtomSpaceUtil::isPredicateTrue( agent->getAtomSpace( ), 
            "has_something_to_say", this->agentHandle ) ) {
            // do not process a second sentence if there
            // already one to be said
            logger().debug( "QuestionAnsweringDialogController::%s - There is already something to say. "
                            "So this call will be ignoring.",
                            __FUNCTION__ );
            return result;
        } // if

        HandleSeq heardSentences = this->langComp->getHeardSentencePredicates( );
        
        if ( heardSentences.size( ) == 0 ) {
            logger().debug( "QuestionAnsweringDialogController::%s - There is no cached heard sentence. ",
                            __FUNCTION__ );
            return result;
        } // if
        
        Handle listLink = agent->getAtomSpace( ).getOutgoing( heardSentences.back( ), 1 );
        Handle sentenceNode = agent->getAtomSpace( ).getOutgoing( listLink, 0 );

        if ( !AtomSpaceUtil::isPredicateTrue( agent->getAtomSpace( ),
                 "is_question", sentenceNode ) ||
             AtomSpaceUtil::isPredicateTrue( 
                agent->getAtomSpace( ), "was_answered", sentenceNode ) ) {
            // latest heard sentence is not a question
            logger().debug( "QuestionAnsweringDialogController::%s - Heard sentence isn't a question or was already answered.",
                            __FUNCTION__ );
            return result;
        } // if

        result.heardSentence = 
            this->langComp->getTextFromSentencePredicate( heardSentences.back( ) );
        
        std::string answer = SchemeEval::instance().eval( "(answer-question)");    
        logger().debug( "QuestionAnsweringDialogController::%s - (answer-question) answer: %s", __FUNCTION__, answer.c_str() );
        if ( SchemeEval::instance().eval_error() ) {
            logger().error( "QuestionAnsweringDialogController::%s - An error occurred while trying to resolve reference: %s",
                            __FUNCTION__, answer.c_str( ) );
        } // if
        SchemeEval::instance().clear_pending( );

        boost::trim(answer);

        if ( answer == "#truth-query" ) {
            // yes/no question
            HandleSeq elements = this->langComp->getActivePredicateArguments( 
                "latestQuestionFrames" );

            if ( elements.size( ) > 0 ) {
                result.sentence = "Yes";
            } else {
                opencog::AtomSpace& as = agent->getAtomSpace( );
                HandleSeq link(2);
                link[0] = as.getHandle( PREDICATE_NODE, "unknownTerm" );
                link[1] = Handle::UNDEFINED;
                if ( link[0] != Handle::UNDEFINED ) {                   

                    Type types[] = {PREDICATE_NODE, LIST_LINK };
                    HandleSeq evalLinks;
                    as.getHandleSet( back_inserter(evalLinks),
                                     link, &types[0], NULL, 2, EVALUATION_LINK, false );
                    bool unknownTermFound = false;
                    unsigned int i;
                    for (i = 0; i < evalLinks.size( ); ++i ) {
                        TruthValuePtr ev_tv = as.getTV( evalLinks[i] );
                        if ( ev_tv->isNullTv() || ev_tv->getMean() == 0 ) {
                            continue;                    
                        } // if
                        as.setTV( evalLinks[i], TruthValue::FALSE_TV() );
                        unknownTermFound = true;
                    } // for
                    
                    if ( unknownTermFound ) {
                        result.sentence = "Could you be more specific, please?";
                    } else {
                        result.sentence = "No";
                    } // else

                } else {
                    result.sentence = "No";
                } // else

            } // else
            result.status = result.sentence.length( ) > 0;
        } else {
            // call nlgen using relations
            result.status = true;
            result.sentence = langComp->resolveFrames2Relex( );
        } // else
        logger().debug("QuestionAnsweringDialogController::%s - (%s) Sent[%s] Received[%s]",
                       __FUNCTION__, getName( ).c_str( ), result.heardSentence.c_str( ), result.sentence.c_str( ));

        return result;
#endif
        
    }
};


/**
 * This DC is a generic network DC. It will use the given host/port to try to
 * retrieve a sentence in a server outside the application. Basically, at each
 * update, this DC will send a sentence and receive another as the answer of
 * the first.
 */
class NetworkDialogController : public LanguageComprehension::DialogController 
{
public:
    NetworkDialogController( const std::string& name, LanguageComprehension* langComp, 
                             const std::string& host, const std::string& port ) :
        LanguageComprehension::DialogController::DialogController( name, langComp )
    {        
        try {
            logger().debug("NetworkDialogController::%s - opening socket connection",__FUNCTION__);

            boost::asio::ip::tcp::resolver resolver(service);
            boost::asio::ip::tcp::resolver::query query(boost::asio::ip::tcp::v4( ), host, port);
            this->iterator = resolver.resolve(query);
        } catch( const std::exception& e){
            logger().error("NetworkDialogController::%s - Failed to open socket. Exception Message: %s",__FUNCTION__,e.what());
            
        } // catch
        
    }

    virtual ~NetworkDialogController( void ) 
    {
    }

    virtual LanguageComprehension::DialogController::Result processSentence( long elapsedTime )
    {
        LanguageComprehension::DialogController::Result result;
        try {
            boost::asio::ip::tcp::resolver::iterator end;
            if ( iterator == end ) {
                logger().error("NetworkDialogController::%s - (%s) Invalid iterator. "
                               "Socket cannot be opened.",
                               __FUNCTION__, getName( ).c_str( ));
                return result;
            } // if

            HandleSeq heardSentences = this->langComp->getHeardSentencePredicates( );
            
            if ( heardSentences.size( ) == 0 ) {
                logger().error("NetworkDialogController::%s - (%s) There is no heard sentence available. ",
                               __FUNCTION__, getName( ).c_str( ) );
                return result;
            } // if

            result.heardSentence = 
                this->langComp->getTextFromSentencePredicate( heardSentences.back( ) );

            boost::asio::ip::tcp::socket socket(this->service);
            socket.connect( *this->iterator );

            logger().debug("NetworkDialogController::%s - (%s) socket connection opened with success",
                __FUNCTION__, getName( ).c_str( ));

            {
                std::stringstream sentence;
                sentence << result.heardSentence;
                sentence << std::endl;
                socket.send(boost::asio::buffer(sentence.str().c_str(), sentence.str().length( ) ) );
            }

            std::stringstream message;
            do {
                char buffer[1024] = { '\0' };
                socket.receive( boost::asio::buffer(buffer, 1024) );
                message << buffer;
            } while( socket.available( ) > 0 );
            
            socket.close( );

            logger().debug("NetworkDialogController::%s - (%s) socket connection closed with success",
                               __FUNCTION__, getName( ).c_str( ));

            std::string sentence = message.str( );
            boost::trim(sentence);

            result.sentence = sentence;
            result.status = sentence.length( ) > 0;

            logger().debug("NetworkDialogController::%s - (%s) Sent[%s] Received[%s]",
                               __FUNCTION__, getName( ).c_str( ), result.heardSentence.c_str( ), result.sentence.c_str( ));

            return result;
        } catch( const std::exception& e){
            logger().error("NetworkDialogController::%s - (%s) Failed to open socket. Exception Message: %s",
                           __FUNCTION__, getName( ).c_str( ),e.what());
            return result;
        } // catch
    }

private:
    boost::asio::io_service service;    
    boost::asio::ip::tcp::resolver::iterator iterator;
};


/**
 * A Network Question Answerer DC will look for unanswered questions into the
 * AtomSpace and will use the host service to try to retrieve an answer for
 * a given question.
 */
class NetworkQuestionAnswererDialogController : public NetworkDialogController
{

public:
    NetworkQuestionAnswererDialogController( const std::string& name, LanguageComprehension* langComp, 
                                             const std::string& host, const std::string& port ) :
        NetworkDialogController::NetworkDialogController( name, langComp, host, port )
    {
        
    }
    
    virtual LanguageComprehension::DialogController::Result processSentence( long elapsedTime )
    {

        LanguageComprehension::DialogController::Result result;
        result.questionAnswering = true;
        
        if ( AtomSpaceUtil::isPredicateTrue( agent->getAtomSpace( ), 
            "has_something_to_say", this->agentHandle ) ) {
            // do not process a second sentence if there
            // already one to be said
            return result;
        } // if

        HandleSeq heardSentences = this->langComp->getHeardSentencePredicates( );
        
        if ( heardSentences.size( ) == 0 ) {
            return result;
        } // if
        Handle listLink = agent->getAtomSpace( ).getOutgoing( heardSentences.back( ), 1 );
        Handle sentenceNode = agent->getAtomSpace( ).getOutgoing( listLink, 0 );

        if ( !AtomSpaceUtil::isPredicateTrue( agent->getAtomSpace( ),
                 "is_question", sentenceNode ) ||
             AtomSpaceUtil::isPredicateTrue( 
                agent->getAtomSpace( ), "was_answered", sentenceNode )) {
            // latest heard sentence is not a question
            return result;
        } // if

        result.heardSentence = 
            this->langComp->getTextFromSentencePredicate( heardSentences.back( ) );

        // define a new sentence to be sent to the remote chat bot
        LanguageComprehension::DialogController::Result innerResult;
        innerResult = NetworkDialogController::processSentence( elapsedTime );

        if ( innerResult.status && innerResult.heardSentence == result.heardSentence ) {
            result.status = true;
            result.sentence = innerResult.sentence;
        } // if

        return result;
    }
    
};


/**
 * Dialog Controller Factory Method. If you have created a new DC and want to 
 * use it in the DC chain, you can define some configuration parameters in:
 * opencog/embodiment/Control/EmbodimentConfig.h and add some custom values for
 * that arguments in the .conf file.
 */
LanguageComprehension::DialogController* 
LanguageComprehension::createDialogController( const std::string& name )
{
    if ( name == "QuestionAnswering" ) {
        return new QuestionAnsweringDialogController( this );

    } else if ( name == "MegaHal" ) {
        std::string host = opencog::config().get( "MEGAHAL_SERVER_HOST" );
        std::string port = opencog::config().get( "MEGAHAL_SERVER_PORT" );
        return new NetworkQuestionAnswererDialogController( "MegaHalQuestionAnswering", this, host, port );

    } else if ( name == "AliceBot" ) {
        std::string host = opencog::config().get( "ALICEBOT_SERVER_HOST" );
        std::string port = opencog::config().get( "ALICEBOT_SERVER_PORT" );
        return new NetworkQuestionAnswererDialogController( "AliceBotQuestionAnswering", this, host, port );

    } else if ( name == "Ramona" ) {
        std::string host = opencog::config().get( "RAMONA_SERVER_HOST" );
        std::string port = opencog::config().get( "RAMONA_SERVER_PORT" );
        return new NetworkQuestionAnswererDialogController( "RamonaQuestionAnswering", this, host, port );

    } else {
        return NULL;
    } // else
}

///////////////////////////////////////////////////////////////////////////////
//
// Functions related to frame

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


void LanguageComprehension::loadFrames(void)
{
    opencog::AtomSpace& atomSpace = agent.getAtomSpace();

    { // try to load the frames
        std::string fileName = config().get("FRAMES_FILE");
        boost::trim(fileName);
        if ( fileName.length() == 0 ) {
            logger().debug( "LanguageComprehension::%s - No frames filename was defined",
                            __FUNCTION__ );
            return;
        } // if
        
        std::ifstream input( fileName.c_str() );
        
        if ( !input ) {
            logger().error( "LanguageComprehension::%s - Cannot load frames from: %s",
                            __FUNCTION__, fileName.c_str() );
            return;
        } // if
        
        while( !input.eof() ) {
            std::string line;
            std::getline(input, line);
            boost::trim(line);
            if ( line.length() == 0 ) {
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
                atomSpace.setTV( elementLink, opencog::TruthValue::TRUE_TV() );
                atomSpace.setLTI( elementLink, 1 );
                atomSpace.incVLTI(elementLink); // prvent it from removing
            } // if

        } // while
    } // end block

    { // try to load the frames relations (only inheritance is supported for now)
        std::string fileName = config().get("FRAMES_INHERITANCE_FILE");
        boost::trim(fileName);    
        if ( fileName.length() == 0 ) {
            logger().debug( "LanguageComprehension::%s - No frames relations filename was defined",
                            __FUNCTION__ );
            return;
        } // if
        
        std::ifstream input( fileName.c_str() );
        
        if ( !input ) {
            logger().error( "LanguageComprehension::%s - Cannot load frames relations from: %s",
                            __FUNCTION__, fileName.c_str() );
            return;
        } // if
                
        while( !input.eof() ) {
            std::string line;
            std::getline( input, line );
            boost::trim(line);
            if ( line.length() == 0 ) {
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
                atomSpace.incVLTI(inheritanceLink); // prevent it from removing
            }

        } // while

    } // end block
}

