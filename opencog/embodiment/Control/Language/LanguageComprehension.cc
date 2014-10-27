/*
 * opencog/embodiment/Control/Language/LanguageComprehension.cc
 *
 * Copyright (C) 2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Samir Araujo
 *
 * Update: by Zhenhua Cai <czhedu@gmail.com>, on 2011-11-03
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
#include <boost/regex.hpp>
#include <boost/bind.hpp>
#include <boost/asio/ip/tcp.hpp>

#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/guile/SchemeSmob.h>
#include <opencog/nlp/types/atom_types.h>
#include <opencog/spacetime/atom_types.h>
#include <opencog/spacetime/SpaceServer.h>
#include <opencog/util/foreach.h>

#include <opencog/embodiment/Control/Language/LanguageComprehension.h>
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include <opencog/embodiment/Control/EmbodimentConfig.h>

#include <opencog/embodiment/AtomSpaceExtensions/atom_types.h>

using namespace opencog::oac;
using namespace opencog::spatial;
using namespace opencog;

opencog::control::AvatarInterface* LanguageComprehension::localAgent = NULL;

///////////////////////////////////////////////////////////////////////////////
//
// Core functions of LanguageComprehension

LanguageComprehension::LanguageComprehension( opencog::control::AvatarInterface& agent ) : 
    initialized( false ), agent( agent ), nlgenClient( NULL )
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
}

#ifdef HAVE_GUILE
SchemeEval* LanguageComprehension::evaluator = NULL;
#endif

void LanguageComprehension::init( void )
{
    if ( !initialized ) {
        initialized = true;

#ifdef HAVE_GUILE
        // Ensure SchemeEval is initialised with AtomSpace.
        opencog::AtomSpace& as = this->agent.getAtomSpace();
        evaluator = new SchemeEval(&as);
        
        std::stringstream script;
        script << "(define agentSemeNode (SemeNode \"";
        script << agent.getPetId( ) << "\") )" << std::endl;
        std::string answer = evaluator->eval( script.str( ) );
        if ( evaluator->eval_error() ) {
            logger().error( "LanguageComprehension::%s - An error occurred while trying to setup the agent seme node: %s",
                            __FUNCTION__, answer.c_str( ) );
        } // if
        evaluator->clear_pending();

        loadFrames( );

#ifdef HAVE_GUILE2
 #define C(X) ((scm_t_subr) X)
#else
 #define C(X) ((SCM (*) ()) X)
#endif

        LanguageComprehension::localAgent = &this->agent;
        scm_c_define_gsubr("cog-emb-compute-spatial-relations", 3, 0, 1, 
                           C(&LanguageComprehension::execute));
#endif
                
        this->nlgen_server_port = config().get_int("NLGEN_SERVER_PORT");
        this->nlgen_server_host = config().get("NLGEN_SERVER_HOST");

        nlgenClient = new NLGenClient(this->nlgen_server_host, this->nlgen_server_port);

    } // if
}

void LanguageComprehension::resolveLatestSentenceReference( void )
{
    init();

#ifdef HAVE_GUILE
    std::string answer = evaluator->eval( "(resolve-reference)");
    logger().debug( "LanguageComprehension::%s - (resolve-reference) resolved references: \n%s",
                    __FUNCTION__,
                    answer.c_str() 
                  );
    if ( evaluator->eval_error() ) {
        logger().error( "LanguageComprehension::%s - An error occurred while trying to resolve reference: %s",
                        __FUNCTION__, answer.c_str( ) );
    } // if
    evaluator->clear_pending();
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
    as.getHandlesByOutgoing( back_inserter(evalLinks),
                     commands, &types[0], NULL, 2, EVALUATION_LINK, false );
    logger().debug( "LanguageComprehension::%s - Number of EvaluationLinks for Predicate '%s': %d",
                    __FUNCTION__, predicateName.c_str(), evalLinks.size() );

    HandleSeq arguments;
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
        arguments = as.getOutgoing(as.getOutgoing( evalLinks[k], 1 ));
    } // for

    return arguments;
}

void LanguageComprehension::resolveLatestSentenceCommand( void )
{
    init();

#ifdef HAVE_GUILE
    std::string answer = evaluator->eval( "(resolve-command)");
    logger().debug( "LanguageComprehension::%s - (resolve-command) answer: %s", __FUNCTION__, answer.c_str() );
    if ( evaluator->eval_error() ) {
        logger().error( "LanguageComprehension::%s - An error occurred while trying to resolve command: %s",
                        __FUNCTION__, answer.c_str( ) );
    } // if
    evaluator->clear_pending();
#endif
    
    opencog::AtomSpace& as = agent.getAtomSpace();
    // TODO: the variable name here is quite confusing. We should rename 'elements' to 'frameIntances'
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
            logger().error( "LanguageComprehension::%s - Malformed Execution link. "
                            "It should link a GroundedSchemaNode and a ListLink of arguments. "
                            "But types are 0 -> %d and 1 -> %d", 
                            __FUNCTION__, 
                            as.getType(gsn), as.getType(argumentsList) 
                          );
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

std::string LanguageComprehension::resolveFrames2Sentence(void)
{
    init();

    std::vector < std::pair<std::string, Handle> > handles; 
    std::set< std::string > pre_conditions;

    std::map<std::string, unsigned int> frame_elements_count;

    opencog::AtomSpace& as = agent.getAtomSpace();
    HandleSeq answerFrameInstances = getActivePredicateArguments("latestAnswerFrames");

    // there is no frame instance related to the answer
    if ( answerFrameInstances.size() == 0 ) {
        logger().debug( "LanguageComprehension::%s - number of frame instances related to the answer is 0. "
                        "'I don't know' answer will be reported.",
                        __FUNCTION__ );
        return "";
    } 

    foreach (Handle frameInstance, answerFrameInstances) {
        // try to get InheritanceLinks holding both the given frame instance
        // (a PredicateNode) and an DefinedFrameNode. 
        //
        // Here is an example:
        //
        // (InheritanceLink (stv 1 0.99999988) (av -3 1 0)
        //     (PredicateNode "G_red@ce9b596d-fad6-4858-a2cc-9bd03e029389_Color" (av -3 0 0))
        //     (DefinedFrameNode "#Color" (av -2 0 0))
        // )
        //
        std::string frameName;
        HandleSeq inheritanceLink;
        inheritanceLink.push_back( frameInstance );
        inheritanceLink.push_back( Handle::UNDEFINED );
        
        Type inheritanceLinkTypes[] = { PREDICATE_NODE, DEFINED_FRAME_NODE };
        HandleSeq inheritanceLinks;
        as.getHandlesByOutgoing( back_inserter( inheritanceLinks ),
                         inheritanceLink,
                         &inheritanceLinkTypes[0], NULL, 2, INHERITANCE_LINK, false );

        // If it is a valid frame instance, then retrieve the frame name
        if ( inheritanceLinks.size() > 0 ) {
            if ( inheritanceLinks.size() > 1 ) {
                logger().error( "LanguageComprehension::%s - The given handle represents more than one instance of Frame, what is unacceptable. Only the first occurrence will be considered.", __FUNCTION__ );
            } 

            frameName = as.getName( as.getOutgoing( inheritanceLinks[0], 1 ) );
            logger().debug( "LanguageComprehension::%s - FrameName detected: %s", 
                            __FUNCTION__, 
                            frameName.c_str() 
                          );
        }
        else {
            logger().debug( "LanguageComprehension::%s - The given handle '%s' isn't a Frame instance. It doesn't inherit from a DEFINED_FRAME_NODE",
                            __FUNCTION__,
                            as.getName(frameInstance).c_str() 
                          );
            return "";
        } 

        // Get frame element name-value pairs        
        std::map<std::string, Handle> frameElementInstanceNameValues =
            AtomSpaceUtil::getFrameElementInstanceNameValues( as, frameInstance );
            
        logger().debug( "LanguageComprehension::%s - Number of frame elements found: '%d' for frame instance: '%s'", 
                        __FUNCTION__, 
                        frameElementInstanceNameValues.size(), 
                        as.atomAsString(frameInstance).c_str() 
                      );
        
        std::map<std::string, Handle>::iterator it;
        for( it = frameElementInstanceNameValues.begin(); 
             it != frameElementInstanceNameValues.end(); 
             ++it ) {
            logger().debug( "LanguageComprehension::%s - Inspecting frame element instance name: '%s' value: '%s'", 
                            __FUNCTION__, 
                            it->first.c_str(),
                            as.atomAsString(it->second).c_str() 
                          );
            std::string frameElementName = frameName + ":" + it->first;
            handles.push_back( std::pair<std::string, Handle>(frameElementName, it->second) );

            // count the number of times each element appears to be part of
            // the pre-conditions
            frame_elements_count[frameElementName] > 0 ? 
                frame_elements_count[frameElementName] += 1:
                frame_elements_count[frameElementName] = 1; 
        }// for

    }// foreach (Handle frameInstance, answerFrameInstances)
  
    // iterate the frame_elements_count to get the preconditions appended with
    // the element count, like: #Color:Color$2, #Entity:Entity$1, and so on
    std::map< std::string, unsigned int>::const_iterator iter;
    for( iter = frame_elements_count.begin(); iter != frame_elements_count.end(); ++iter) {
        std::string precondition = iter->first + "$" + boost::lexical_cast<std::string>(iter->second);
        logger().debug( "LanguageComprehension::%s - Pre-Condition detected: '%s'", 
                        __FUNCTION__, precondition.c_str() );
        pre_conditions.insert( precondition );
    }
   
    // log preconditions
    logger().debug("LanguageComprehension::%s - Begin of Pre-Conditions",__FUNCTION__);
    std::set< std::string >::iterator it;
    for(it = pre_conditions.begin(); it != pre_conditions.end(); ++it){
        logger().debug("LanguageComprehension::%s - Pre-Condition: %s",__FUNCTION__,(*it).c_str());
    }
    logger().debug("LanguageComprehension::%s - End of Pre-Conditions",__FUNCTION__);   
   
    // get all the answer related frame instances, which would be returned if
    // frames to relex or relex to sentence fails
    std::string answerFrameInstancesStr; 

    foreach (Handle frameInstance, answerFrameInstances) {
        answerFrameInstancesStr += as.atomAsString(frameInstance) + "    "; 
    }

    // resolve frame to relex
    OutputRelex* output_relex = framesToRelexRuleEngine.resolve( pre_conditions );

    if( output_relex == NULL ){
        logger().debug("LanguageComprehension::%s - "
                       "Output Relex is NULL for the pre-conditions. No rules were found.",
                       __FUNCTION__
                      );

        return "Found the answer but lack of suitable Frames2Relex. The answer involved frame instances are: " +
               answerFrameInstancesStr;
    }
    
    std::string text = output_relex->getOutput( as, handles );
    if( text.empty() ){
        logger().error("LanguageComprehension::%s - Output Relex returned an empty string.", __FUNCTION__);
        return "Found the answer but Relex2Sentence returns an empty string. The answer involved frame instances are: " +
                answerFrameInstancesStr;
    }

    // resolve relex to sentence and return the result
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

void LanguageComprehension::updateFact(void)
{
    init();

#ifdef HAVE_GUILE
    std::string answer = evaluator->eval( "(update-fact)");    

    logger().debug( "LanguageComprehension::%s - (update-fact) newly created or deleted frame instances (fact): \n%s", 
                    __FUNCTION__, 
                    answer.c_str()
                  );

    if ( evaluator->eval_error() ) {
        logger().error( "LanguageComprehension::%s - (update-fact) An error occurred while trying to update fact: \n%s",
                        __FUNCTION__, 
                        answer.c_str() 
                      );
    } // if

    evaluator->clear_pending( );    
#endif
}

HandleSeq LanguageComprehension::getHeardSentencePredicates( void )
{
    AtomSpace & atomSpace = agent.getAtomSpace( );
    Handle agentHandle = AtomSpaceUtil::getAgentHandle( atomSpace, agent.getPetId() );

    HandleSeq heardSentences;
    
    Handle node = atomSpace.getHandle( PREDICATE_NODE, "heard_sentence" );
    if ( node == Handle::UNDEFINED ) {
        return heardSentences;
    } 

    HandleSeq incomingSet = atomSpace.getIncoming(node);

    foreach (Handle incoming, incomingSet) {
        if ( atomSpace.getType(incoming) != EVALUATION_LINK ||
             atomSpace.getTV(incoming)->isNullTv() || 
             atomSpace.getMean(incoming) == 0 ||
             atomSpace.getArity(incoming) != 2 ||
             atomSpace.getOutgoing(incoming, 1) == node ) {
            continue;
        } 

        Handle listLink = atomSpace.getOutgoing(incoming, 1);
        Handle sentenceNode = atomSpace.getOutgoing(listLink, 0);
        if ( atomSpace.getType(sentenceNode) != SENTENCE_NODE ) {
            continue;
        }

        // "to:agentId: sentencetext"
        std::string nodeName = atomSpace.getName( sentenceNode );
        static const boost::regex pattern( "^to:[^:]+:\\s.+$");
        if ( !boost::regex_match( nodeName, pattern ) ) {
            logger().error( "LanguageComprehension::%s - Invalid sentence node '%s'. "
                            "Should be in format 'to:agentId: sentencetext'",
                            __FUNCTION__, nodeName.c_str() );
        }

        heardSentences.push_back(incoming);
    } // foreach
    
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

void LanguageComprehension::answerQuestion()
{
#ifdef HAVE_GUILE
    // -------------------------------------------------------------------------
    // Step 0: preparation
    
    init();

    AtomSpace & atomSpace = this->agent.getAtomSpace(); 
    Handle agentHandle = AtomSpaceUtil::getAgentHandle( atomSpace, agent.getPetId() );

    std::string answer = ""; 

    // Do not process a second sentence if there's already one to be said
    if ( AtomSpaceUtil::isPredicateTrue( atomSpace, "has_something_to_say", agentHandle ) ) {
        logger().debug( "LanguageComprehension::%s - There is already something to say. "
                        "So this call will be ignoring.",
                        __FUNCTION__
                      );
        return; 
    } 

    HandleSeq heardSentences = this->getHeardSentencePredicates();
    
    if ( heardSentences.size() == 0 ) {
        logger().debug( "LanguageComprehension::%s - There is no cached heard sentence. ",
                        __FUNCTION__ 
                      );
        return; 
    } 

    foreach (Handle hEvalSentence, heardSentences) {
        logger().debug("LanguageComprehension::%s - Cached heard sentence: %s", 
                       __FUNCTION__, 
                       atomSpace.atomAsString(hEvalSentence).c_str()
                      ); 
    }

    Handle listLink = atomSpace.getOutgoing( heardSentences.front(), 1 );
    Handle sentenceNode = atomSpace.getOutgoing( listLink, 0 );

    // If latest heard sentence is not a question, do not process any more. 
    bool bIsQuestion = AtomSpaceUtil::isPredicateTrue( atomSpace, "is_question", sentenceNode ); 
    bool bWasAnswered = AtomSpaceUtil::isPredicateTrue( atomSpace, "was_answered", sentenceNode ); 

    if ( !bIsQuestion || bWasAnswered ) {
        logger().debug( "LanguageComprehension::%s - Heard sentence isn't a question or was already answered. "
                        "is_question: %s, was_answered: %s, sentence: %s",
                        __FUNCTION__, 
                        bIsQuestion? "True": "False", 
                        bWasAnswered? "True": "False", 
                        atomSpace.atomAsString(sentenceNode).c_str()
                      );
        return; 
    }

    std::string heardSentence = this->getTextFromSentencePredicate( heardSentences.front() );

    // --------------------------------------------------------------------------
    // Step 1: get frames representing answers by pattern matching

    // Launch a pattern matching process to find the answer in atomspace.
    // The question type is returned and answer frames are written into atomspace. 
    //
    // Answer frames are stored as 
    //   EvaluationLink
    //       PredicateNode "latestAnswerFrames"
    //       ListLink
    //           Frame1
    //           Frame 2
    //           ...
    //
    std::string question_type = evaluator->eval( "(answer-question)");    
    logger().debug( "LanguageComprehension::%s - (answer-question) question type: %s",
                    __FUNCTION__,
                    question_type.c_str()
                  );
    if ( evaluator->eval_error() ) {
        logger().error( "LanguageComprehension::%s - An error occurred while trying to answer the question: %s",
                        __FUNCTION__, 
                        question_type.c_str()
                      );
    } 
    evaluator->clear_pending( );

    // ------------------------------------------------------------------------
    // Step 2: create sentences based on answer frames via frame2relex and relex2sentence. 

    boost::trim(question_type);
    std::string answer_sentence = ""; 

    // yes/no question
    // TODO: generate answers based on the truth value (such as pretty sure, maybe etc.)
    if ( question_type == "#truth-query" ) {
        HandleSeq elements = this->getActivePredicateArguments("latestAnswerFrames");

        if ( elements.size() > 0 ) {
            answer_sentence = "Yes";
        } 
        else {
            HandleSeq link(2);
            link[0] = atomSpace.getHandle( PREDICATE_NODE, "unknownTerm" );
            link[1] = Handle::UNDEFINED;

            if ( link[0] != Handle::UNDEFINED ) {

                Type types[] = {PREDICATE_NODE, LIST_LINK };
                HandleSeq evalLinks;
                atomSpace.getHandlesByOutgoing( back_inserter(evalLinks),
                                        link, &types[0], NULL, 2, EVALUATION_LINK, false );

                // search for unknown terms
                bool unknownTermFound = false;
                unsigned int i;
                for (i = 0; i < evalLinks.size( ); ++i ) {
                    TruthValuePtr ev_tv = atomSpace.getTV( evalLinks[i] );
                    if ( ev_tv->isNullTv() || ev_tv->getMean() == 0 ) {
                        continue;                    
                    }
                    atomSpace.setTV( evalLinks[i], TruthValue::FALSE_TV() );
                    unknownTermFound = true;
                } // for
                
                if ( unknownTermFound ) {
                    answer_sentence = "Could you be more specific, please?";
                } 
                else {
                    answer_sentence = "No";
                } 
            }
            else {
                answer_sentence = "No";
            } // if ( link[0] != Handle::UNDEFINED )

        } // if ( elements.size() > 0 )
    }
    // other types of question, call nlgen using relations
    else {
        answer_sentence = this->resolveFrames2Sentence();
    } // if ( question_type == "#truth-query" )

    logger().debug("LanguageComprehension::%s - question: '%s' answer: '%s'",
                   __FUNCTION__,
                   heardSentence.c_str(),   // original question
                   answer_sentence.c_str()  // answer
                  );

    // -------------------------------------------------------------------------
    // Step 3: choose the sentence that best answers the question and do clean up work.  
    //         (PsiActionSelectionAgent::executeAction will generate 'say' actions later)

    HandleSeq sentences; 
    sentences.push_back( atomSpace.addNode( ANCHOR_NODE, "# Possible Sentences" ) );
    sentences.push_back( atomSpace.addNode( SENTENCE_NODE, answer_sentence) ); 
    atomSpace.addLink( LIST_LINK, sentences, SimpleTruthValue::createTV( 1, 1 ) );

    answer = evaluator->eval( "(choose-sentence)");
    logger().debug( "LanguageComprehension::%s - (choose-sentence) answer: %s",
                    __FUNCTION__, 
                    answer.c_str()
                  );
    if ( evaluator->eval_error() ) {
        logger().error( "LanguageComprehension::%s - "
                        "An error occurred while trying to choose a sentence: %s",
                        __FUNCTION__,
                        answer.c_str( ) 
                      );
    } 

    evaluator->clear_pending( );

    // now set to false all heard sentences and set as answered all heard questions
    for( unsigned int i = 0; i < heardSentences.size(); ++i ) {
        atomSpace.setTV( heardSentences[i], SimpleTruthValue::createTV( 0, 1) );

        Handle listLink = atomSpace.getOutgoing( heardSentences[i], 1 );
        Handle sentenceNode = atomSpace.getOutgoing( listLink, 0 );

        if ( AtomSpaceUtil::isPredicateTrue( atomSpace, "is_question", sentenceNode ) ) {
            AtomSpaceUtil::setPredicateValue( atomSpace,
                                              "was_answered",
                                              TruthValue::TRUE_TV(), 
                                              sentenceNode 
                                            );
        } 
    } // for

    AtomSpaceUtil::setPredicateValue( atomSpace,
                                      "has_unanswered_question",
                                      TruthValue::FALSE_TV() 
                                    );

#endif
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
    
    const SpaceServer::SpaceMap& spaceMap = spaceServer( ).getLatestMap( );
    
    // double besideDistance = spaceMap.getNextDistance( );
    
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
        const spatial::Entity3D* observerEntity = spaceMap.getEntity( atomSpace.getName( observer ) );
        
        unsigned int i, j, k;
        for( i = 0; i < entitiesA.size( ); ++i ) {
            const spatial::Entity3D* entityA = spaceMap.getEntity( entitiesA[i] );
            for( j = 0; j < entitiesB.size( ); ++j ) {
                if ( entitiesA[i] == entitiesB[j] ) {
                    continue;
                } // if
                const spatial::Entity3D* entityB = spaceMap.getEntity( entitiesB[j] );
                if ( entitiesC.size( ) > 0 ) {
                    for( k = 0; k < entitiesC.size( ); ++k ) {
                        if ( entitiesA[i] == entitiesC[k] || entitiesB[j] == entitiesC[k] ) {
                            continue;
                        } // if
                        const spatial::Entity3D* entityC = spaceMap.getEntity( entitiesC[k] );
                        createFrameInstancesFromRelations( atomSpace, resultingFrames,
                            spaceMap.computeSpatialRelations( entityA, entityB, entityC,observerEntity ),
                                entitiesA[i], entitiesB[j], entitiesC[k] );
                    } // for
                } else {
                    createFrameInstancesFromRelations( atomSpace, resultingFrames,
                        spaceMap.computeSpatialRelations(  entityA,entityB,0,observerEntity ),
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
        const std::set<spatial::SPATIAL_RELATION>& relations,
            const std::string& objectA, const std::string& objectB, const std::string& objectC ) {

    std::set<spatial::SPATIAL_RELATION>::const_iterator it;
    for( it = relations.begin( ); it != relations.end( ); ++it ) {
        std::string relationName = SpaceServer::SpaceMap::spatialRelationToString( *it );

        std::map<std::string, Handle> elements;
        elements["Figure"] = atomSpace.getHandle( SEME_NODE, objectA );
        elements["Ground"] = atomSpace.getHandle( SEME_NODE, objectB );
        elements["Relation_type"] = atomSpace.addNode( CONCEPT_NODE, relationName );

        std::stringstream instanceName;
        instanceName << objectA;
        instanceName << "_";
        instanceName << objectB;

        if ( *it == spatial::BETWEEN ) {
            elements["Ground_2"] = atomSpace.getHandle( SEME_NODE, objectC );
            instanceName << "_";
            instanceName << objectC;
        } // if            

        instanceName << "_" << relationName;
        resultingFrames.push_back(
            AtomSpaceUtil::setPredicateFrameFromHandles(
                atomSpace, "#Locative_relation", instanceName.str( ), 
                    elements, SimpleTruthValue::createTV(1.0, 1.0), false ) );
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

///////////////////////////////////////////////////////////////////////////////
//
// Controllers classes
//
// Note: Since openpsi will take over dialogue control, all these controllers are
//       not necessary. However, many methods of these controllers are still useful. 
//       Move these useful methods to LanguageComprehension class gradually and use
//       LanguageComprehension directly. 
// 

///**
// * This DC is a generic network DC. It will use the given host/port to try to
// * retrieve a sentence in a server outside the application. Basically, at each
// * update, this DC will send a sentence and receive another as the answer of
// * the first.
// */
//class NetworkDialogController : public DialogController 
//{
//public:
//    NetworkDialogController( const std::string& name, LanguageComprehension* langComp, 
//                             const std::string& host, const std::string& port ) :
//                             DialogController::DialogController( name, langComp )
//    {        
//        try {
//            logger().debug("NetworkDialogController::%s - opening socket connection",__FUNCTION__);
//
//            boost::asio::ip::tcp::resolver resolver(service);
//            boost::asio::ip::tcp::resolver::query query(boost::asio::ip::tcp::v4( ), host, port);
//            this->iterator = resolver.resolve(query);
//        } catch( const std::exception& e){
//            logger().error("NetworkDialogController::%s - Failed to open socket. Exception Message: %s",__FUNCTION__,e.what());
//            
//        } // catch
//        
//    }
//
//    virtual ~NetworkDialogController( void ) 
//    {
//    }
//
//    virtual DialogController::Result processSentence( long elapsedTime )
//    {
//        DialogController::Result result;
//        try {
//            boost::asio::ip::tcp::resolver::iterator end;
//            if ( iterator == end ) {
//                logger().error("NetworkDialogController::%s - (%s) Invalid iterator. "
//                               "Socket cannot be opened.",
//                               __FUNCTION__, getName( ).c_str( ));
//                return result;
//            } // if
//
//            HandleSeq heardSentences = this->langComp->getHeardSentencePredicates( );
//            
//            if ( heardSentences.size( ) == 0 ) {
//                logger().error("NetworkDialogController::%s - (%s) There is no heard sentence available. ",
//                               __FUNCTION__, getName( ).c_str( ) );
//                return result;
//            } // if
//
//            result.heardSentence = 
//                this->langComp->getTextFromSentencePredicate( heardSentences.back( ) );
//
//            boost::asio::ip::tcp::socket socket(this->service);
//            socket.connect( *this->iterator );
//
//            logger().debug("NetworkDialogController::%s - (%s) socket connection opened with success",
//                __FUNCTION__, getName( ).c_str( ));
//
//            {
//                std::stringstream sentence;
//                sentence << result.heardSentence;
//                sentence << std::endl;
//                socket.send(boost::asio::buffer(sentence.str().c_str(), sentence.str().length( ) ) );
//            }
//
//            std::stringstream message;
//            do {
//                char buffer[1024] = { '\0' };
//                socket.receive( boost::asio::buffer(buffer, 1024) );
//                message << buffer;
//            } while( socket.available( ) > 0 );
//            
//            socket.close( );
//
//            logger().debug("NetworkDialogController::%s - (%s) socket connection closed with success",
//                               __FUNCTION__, getName( ).c_str( ));
//
//            std::string sentence = message.str( );
//            boost::trim(sentence);
//
//            result.sentence = sentence;
//            result.status = sentence.length( ) > 0;
//
//            logger().debug("NetworkDialogController::%s - (%s) Sent[%s] Received[%s]",
//                               __FUNCTION__, getName( ).c_str( ), result.heardSentence.c_str( ), result.sentence.c_str( ));
//
//            return result;
//        } catch( const std::exception& e){
//            logger().error("NetworkDialogController::%s - (%s) Failed to open socket. Exception Message: %s",
//                           __FUNCTION__, getName( ).c_str( ),e.what());
//            return result;
//        } // catch
//    }
//
//private:
//    boost::asio::io_service service;    
//    boost::asio::ip::tcp::resolver::iterator iterator;
//};
//
//
///**
// * A Network Question Answerer DC will look for unanswered questions into the
// * AtomSpace and will use the host service to try to retrieve an answer for
// * a given question.
// */
//class NetworkQuestionAnswererDialogController : public NetworkDialogController
//{
//
//public:
//    NetworkQuestionAnswererDialogController( const std::string& name, LanguageComprehension* langComp, 
//                                             const std::string& host, const std::string& port ) :
//        NetworkDialogController::NetworkDialogController( name, langComp, host, port )
//    {
//        
//    }
//    
//    virtual DialogController::Result processSentence( long elapsedTime )
//    {
//        DialogController::Result result;
//        result.questionAnswering = true;
//        
//        if ( AtomSpaceUtil::isPredicateTrue( agent->getAtomSpace( ), 
//            "has_something_to_say", this->agentHandle ) ) {
//            // do not process a second sentence if there
//            // already one to be said
//            return result;
//        } // if
//
//        HandleSeq heardSentences = this->langComp->getHeardSentencePredicates( );
//        
//        if ( heardSentences.size( ) == 0 ) {
//            return result;
//        } // if
//        Handle listLink = agent->getAtomSpace( ).getOutgoing( heardSentences.back( ), 1 );
//        Handle sentenceNode = agent->getAtomSpace( ).getOutgoing( listLink, 0 );
//
//        if ( !AtomSpaceUtil::isPredicateTrue( agent->getAtomSpace( ),
//                 "is_question", sentenceNode ) ||
//             AtomSpaceUtil::isPredicateTrue( 
//                agent->getAtomSpace( ), "was_answered", sentenceNode )) {
//            // latest heard sentence is not a question
//            return result;
//        } // if
//
//        result.heardSentence = 
//            this->langComp->getTextFromSentencePredicate( heardSentences.back( ) );
//
//        // define a new sentence to be sent to the remote chat bot
//        DialogController::Result innerResult;
//        innerResult = NetworkDialogController::processSentence( elapsedTime );
//
//        if ( innerResult.status && innerResult.heardSentence == result.heardSentence ) {
//            result.status = true;
//            result.sentence = innerResult.sentence;
//        } // if
//
//        return result;
//    }
//    
//};

