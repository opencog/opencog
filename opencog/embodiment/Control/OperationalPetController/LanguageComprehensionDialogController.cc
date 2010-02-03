/*
 * opencog/embodiment/Control/OperationalPetController/LanguageComprehensionDialogController.cc
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

#include <opencog/embodiment/Control/OperationalPetController/LanguageComprehension.h>
#include <opencog/embodiment/Control/EmbodimentConfig.h>

#include <boost/bind.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/thread/xtime.hpp>


using namespace OperationalPetController;

boost::mutex LanguageComprehension::DialogController::lock;

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

    this->dialogControllers.push_back( dialogController );
}

void LanguageComprehension::updateDialogControllers( long elapsedTime, bool wait )
{
    init();
    std::list<DialogController* >::iterator it;
    for( it = this->dialogControllers.begin( ); it != this->dialogControllers.end( ); ++it ) {
        (*it)->update( elapsedTime, wait );
    } // for
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

class QuestionAnsweringDialogController : public LanguageComprehension::DialogController 
{
public:
    QuestionAnsweringDialogController( LanguageComprehension* langComp ) :
        LanguageComprehension::DialogController::DialogController( "QuestionAnswering", langComp ) 
    { }

    virtual ~QuestionAnsweringDialogController( void ) 
    { }

    void retrieveAnswer( void )
    {
#ifdef HAVE_GUILE
        logger().debug( "QuestionAnsweringDialogController::%s - waiting for freeing the global lock", 
                        __FUNCTION__ );
        this->processingAnswer = true;
        boost::mutex::scoped_lock locking( lock );
        logger().debug( "QuestionAnsweringDialogController::%s - ok", 
                        __FUNCTION__ );


        // if there already something to be said, ignore this call
        if ( AtomSpaceUtil::isPredicateTrue( agent->getAtomSpace( ), "has_something_to_say", this->agentHandle ) ) {
            this->processingAnswer = false;
            return;
        } // if

        std::string answer = SchemeEval::instance().eval( "(answer-question)");    
        logger().info( "QuestionAnsweringDialogController::%s - (answer-question) answer: %s", __FUNCTION__, answer.c_str() );
        if ( SchemeEval::instance().eval_error() ) {
            logger().error( "QuestionAnsweringDialogController::%s - An error occurred while trying to resolve reference: %s",
                            __FUNCTION__, answer.c_str( ) );
        } // if
        SchemeEval::instance().clear_pending( );

        boost::trim(answer);

        std::string finalSentence;
        if ( answer == "#truth-query" ) {
            // yes/no question
            HandleSeq elements = LanguageComprehension::getActivePredicateArguments( 
                                                                                    agent->getAtomSpace( ), "latestQuestionFrames" );

            if ( elements.size( ) > 0 ) {
                finalSentence = "Yes";
            } else {
                opencog::AtomSpace& as = agent->getAtomSpace( );
                HandleSeq link(2);
                link[0] = as.addNode( PREDICATE_NODE, "unknownTerm" );
                link[1] = Handle::UNDEFINED;
            
                Type types[] = {PREDICATE_NODE, LIST_LINK };
                HandleSeq evalLinks;
                as.getHandleSet( back_inserter(evalLinks),
                                 link, &types[0], NULL, 2, EVALUATION_LINK, false );
                bool unknownTermFound = false;
                unsigned int i;
                for (i = 0; i < evalLinks.size( ); ++i ) {
                    if ( as.getTV( evalLinks[i] ).isNullTv( ) || as.getTV( evalLinks[i] ).getMean( ) == 0 ) {
                        continue;                    
                    } // if
                    as.setTV( evalLinks[i], TruthValue::FALSE_TV() );
                    unknownTermFound = true;
                } // for

                if ( unknownTermFound ) {
                    finalSentence = "Could you be more specific, please?";
                } else {
                    finalSentence = "No";
                } // else            
            } // else
        } else {
            std::string relations = langComp->resolveFrames2Relex( );
            // call nlgen using relations
            finalSentence = relations;
        } // else

        if ( finalSentence.length( ) > 0 ) {
            agent->getCurrentModeHandler( ).setProperty( "customMessage", finalSentence );
            AtomSpaceUtil::setPredicateValue( agent->getAtomSpace( ), "has_something_to_say", TruthValue::TRUE_TV( ),
                                              this->agentHandle ); 
        } // if
        logger().debug("QuestionAnsweringDialogController::%s - retrieved answer: %s",
                       __FUNCTION__, finalSentence.c_str( ) );

        this->processingAnswer = false;
#endif

    }

    void update( long elapsedTime, bool wait ) 
    {        
        if ( isProcessingAnswer( ) ) {
            return;
        } // if

        bool hasSomethingToSay = AtomSpaceUtil::isPredicateTrue( agent->getAtomSpace( ), 
            "has_something_to_say", this->agentHandle );

        bool needUpdate = AtomSpaceUtil::isPredicateTrue( agent->getAtomSpace( ), 
            "hasQuestionToAnswer", this->agentHandle ) && !hasSomethingToSay;
        
        if ( needUpdate ) {
            boost::thread thread( boost::bind( &QuestionAnsweringDialogController::retrieveAnswer, this ) );
            if ( wait ) {
                thread.join( );
            } // if
        } else if ( hasSomethingToSay ) {
            AtomSpaceUtil::setPredicateValue( agent->getAtomSpace( ), "hasQuestionToAnswer", 
                                              TruthValue::FALSE_TV(), agentHandle  );            
        } // if
        
    }
};

class NetworkDialogController : public LanguageComprehension::DialogController 
{
public:
    NetworkDialogController( const std::string& name, LanguageComprehension* langComp, 
                             const std::string& host, const std::string& port, bool persistent = false ) :
        LanguageComprehension::DialogController::DialogController( name, langComp ), 
        connected( false ), persistent( persistent ), socket(service)
    {        
        try {
            logger().debug("NetworkDialogController::%s - opening socket connection",__FUNCTION__);

            boost::asio::ip::tcp::resolver resolver(service);
            boost::asio::ip::tcp::resolver::query query(boost::asio::ip::tcp::v4( ), host, port);
            this->iterator = resolver.resolve(query);
            
            if ( persistent ) {
                this->socket.connect(*iterator);
                logger().debug("NetworkDialogController::%s - socket connection opened with success",__FUNCTION__);
                this->connected = true;
            } // if

        } catch( const std::exception& e){
            logger().error("NetworkDialogController::%s - Failed to open socket. Exception Message: %s",__FUNCTION__,e.what());

        } // catch
        
    }

    virtual ~NetworkDialogController( void ) 
    {
        if ( this->connected ) {
            this->socket.close( );
        } // if
    }

    void retrieveAnswer( void ) 
    {

        logger().debug( "NetworkDialogController::%s - (%s) waiting for freeing the global lock", 
                        __FUNCTION__, getName( ).c_str( ) );
        boost::mutex::scoped_lock locking( lock );
        this->processingAnswer = true;
        logger().debug( "NetworkDialogController::%s - (%s) ok", 
                        __FUNCTION__, getName( ).c_str( ) );

        // if there already something to be said, ignore this call
        if ( AtomSpaceUtil::isPredicateTrue( agent->getAtomSpace( ), "has_something_to_say", this->agentHandle ) ) {
            this->processingAnswer = false;
            return;
        } // if

        try {
            if ( !this->connected ) {
                if ( this->persistent ) {
                    this->processingAnswer = false;
                    return;
                } else {
                    this->socket.connect(*iterator);
                    logger().debug("NetworkDialogController::%s - (%s) socket connection opened with success",
                                   __FUNCTION__, getName( ).c_str( ));
                    this->connected = true;
                } // else
            } // if            
            
            {
                std::stringstream sentence;
                sentence << this->agent->getCurrentModeHandler( ).getPropertyValue( "latestHeardSentence" );
                sentence << std::endl;
                this->socket.send(boost::asio::buffer(sentence.str().c_str(), sentence.str().length( ) ) );
            }
                        
            std::stringstream message;
            do {
                char buffer[1024] = { '\0' };
                this->socket.receive( boost::asio::buffer(buffer, 1024) );
                message << buffer;
            } while( this->socket.available( ) > 0 );
            
            std::string sentence = message.str( );
            boost::trim(sentence);
            if ( sentence.length( ) > 0 ) {
                agent->getCurrentModeHandler( ).setProperty( "customMessage", sentence );
                AtomSpaceUtil::setPredicateValue( agent->getAtomSpace( ), 
                                                  "has_something_to_say", TruthValue::TRUE_TV( ), this->agentHandle );
            } // if
            logger().debug("NetworkDialogController::%s - (%s) retrieved answer: %s",
                           __FUNCTION__, getName( ).c_str( ), sentence.c_str( ) );
            if ( !this->persistent ) {
                this->socket.close( );
                logger().debug("NetworkDialogController::%s - (%s) socket connection closed with success",
                               __FUNCTION__, getName( ).c_str( ));
                this->connected = false;
            } // if
        } catch( const std::exception& e){
            logger().error("NetworkDialogController::%s - (%s) Failed to open socket. Exception Message: %s",
                           __FUNCTION__, getName( ).c_str( ),e.what());

        } // catch

        this->processingAnswer = false;
    }

protected:
    bool isConnected( void )
    {
        return this->connected;
    }

private:
    bool connected;
    bool persistent;
    boost::asio::io_service service;
    boost::asio::ip::tcp::socket socket;
    boost::asio::ip::tcp::resolver::iterator iterator;
};
        
class NetworkQuestionAnswererDialogController : public NetworkDialogController
{

public:
    NetworkQuestionAnswererDialogController( const std::string& name, LanguageComprehension* langComp, 
                                             const std::string& host, const std::string& port, bool persistent = false ) :
        NetworkDialogController::NetworkDialogController( name, langComp, host, port, persistent )
    {
        
    }
    
    void update( long elapsedTime, bool wait ) 
    {
        if ( isProcessingAnswer( ) ) {
            return;
        } // if

        bool hasSomethingToSay = AtomSpaceUtil::isPredicateTrue( agent->getAtomSpace( ), 
            "has_something_to_say", this->agentHandle );

        bool needUpdate = AtomSpaceUtil::isPredicateTrue( agent->getAtomSpace( ), 
            "hasQuestionToAnswer", this->agentHandle ) && !hasSomethingToSay;

        if ( needUpdate ) {
            boost::thread thread( boost::bind( &NetworkDialogController::retrieveAnswer, this ) );
            if ( wait ) {
                thread.join( );
            } // if
        } else if ( hasSomethingToSay ) {
            AtomSpaceUtil::setPredicateValue( agent->getAtomSpace( ), "hasQuestionToAnswer", 
                                              TruthValue::FALSE_TV(), agentHandle  );            
        } // if
    }
    
};

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
