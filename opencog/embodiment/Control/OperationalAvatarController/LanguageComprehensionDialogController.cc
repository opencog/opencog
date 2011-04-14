/*
 * opencog/embodiment/Control/OperationalAvatarController/LanguageComprehensionDialogController.cc
 *
 * Copyright (C) 2010 Novamente LLC
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
#include <opencog/embodiment/Control/EmbodimentConfig.h>

#include <opencog/atomspace/SimpleTruthValue.h>

#include <boost/bind.hpp>
#include <boost/asio/ip/tcp.hpp>

using namespace OperationalAvatarController;

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
