/*
 * opencog/spatial/MapExplorerServer.cc
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

#include <boost/bind.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/thread/xtime.hpp>
#include <boost/functional/hash.hpp>

#include <opencog/util/Logger.h>
#include <opencog/spatial/MapExplorerServer.h>

using namespace opencog::spatial;
using namespace opencog;

MapExplorerServer::MapExplorerServer( const std::string& host, const std::string& port ):
    host(host), port( port ), running( false ), service(NULL), latestMapHash( 0 )
{
    try {        

        boost::asio::ip::tcp::resolver resolver(this->visualDebuggerService);
        boost::asio::ip::tcp::resolver::query query(boost::asio::ip::tcp::v4(), host, port );
        boost::asio::ip::tcp::endpoint endpoint = *resolver.resolve(query);

        this->visualDebuggerAcceptor = new boost::asio::ip::tcp::acceptor(this->visualDebuggerService );
        this->visualDebuggerAcceptor->open( endpoint.protocol( ) );
        this->visualDebuggerAcceptor->set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));
        this->visualDebuggerAcceptor->set_option(boost::asio::ip::tcp::acceptor::keep_alive(true));
        this->visualDebuggerAcceptor->bind( endpoint );

        // max 5 connections
        this->visualDebuggerAcceptor->listen( 5 );
        this->clientMapStringHash.resize( 5 );
        std::fill( this->clientMapStringHash.begin(), this->clientMapStringHash.end(), 0 );


    } catch( std::exception& ex ) {
        logger().error(
            "MapExplorerServer::%s - Unable to listen visual debugger. ip=%s, port=%d. Exception Message: %s", __FUNCTION__,
            host.c_str(), port.c_str( ), ex.what());

        if (this->visualDebuggerAcceptor != NULL) {
            this->visualDebuggerAcceptor->close();
            delete this->visualDebuggerAcceptor;
            this->visualDebuggerAcceptor = NULL;
        } // if
    } // catch

}

MapExplorerServer::~MapExplorerServer( void )
{
    stop( );
    
    if ( this->visualDebuggerAcceptor != NULL ) {
        delete this->visualDebuggerAcceptor;        
    } // if
}

void MapExplorerServer::start( void )
{
    if ( isRunning( ) ) {
        return;
    } // if

    if ( this->visualDebuggerAcceptor == NULL ) {
        logger().error( "MapExplorerServer::%s - Server was not initialized correctly. "
                        "Check the log for error messages", __FUNCTION__ );
        return;
    } // if

    // now, put the visualDebugger service to listen on a given port
    // and wait for connections
    this->running = true;
    this->service = new boost::thread( boost::bind(&MapExplorerServer::serverLoop, this ) );
    logger().debug( "MapExplorerServer::%s - Server initialized and running", __FUNCTION__ );

}

void MapExplorerServer::wait( void )
{
    if ( isRunning( ) ) {
        logger().debug( "MapExplorerServer::%s - Waiting for the serverLoop ending ", __FUNCTION__ );
        this->service->join( );
    } // if
}

void MapExplorerServer::stop( void )
{
    if ( this->running ) {
        logger().debug( "MapExplorerServer::%s - Stopping the service ", __FUNCTION__ );
        this->running = false;
        this->clients.join_all( );
        this->visualDebuggerAcceptor->close( );
        this->service->interrupt( );
        delete this->service;
        this->service = NULL;
    } // if

}

bool MapExplorerServer::isRunning( void )
{
    return this->running;
}

void MapExplorerServer::serverLoop( void ) 
{
    while (this->running) {        
        SocketPtr sock( new boost::asio::ip::tcp::socket( this->visualDebuggerService ) );
        logger().debug( "MapExplorerServer::%s - Accepting new client connection", __FUNCTION__ );

        this->visualDebuggerAcceptor->accept(*sock);
        this->clients.create_thread( boost::bind( &MapExplorerServer::clientLoop, this, sock ) );
    } // while

}

void MapExplorerServer::clientLoop( SocketPtr socket )
{
    unsigned int clientId = clientMapStringHash.size( );

    unsigned int i;
    for( i = 0; clientId == clientMapStringHash.size( ) && i < clientMapStringHash.size( ); ++i ) {
        if ( clientMapStringHash[i] == 0 ) {
            clientId = i;
        } // if
    } // if    

    logger().debug( "MapExplorerServer::%s - New client connected id '%d'", __FUNCTION__, clientId );

    bool clientRunning = true;
    while (this->running && clientRunning ) {
        boost::thread::sleep( boost::get_system_time() + boost::posix_time::milliseconds(250) );

        std::string map;

        { // compare the client map to the latest map
            boost::mutex::scoped_lock lock( this->mapMutex );
            if ( this->latestMapHash != this->clientMapStringHash[clientId] ) {
                this->clientMapStringHash[clientId] = this->latestMapHash;
                map = this->latestMapString;
            } // if
        } // end block

        if ( map.length( ) > 0 ) {
            boost::asio::write(*socket, boost::asio::buffer(map) );
        } // if

        size_t clientMessageSize = socket->available( );
        if ( clientMessageSize > 0 ) { // client sent a message to me
            char buffer[clientMessageSize+1];
            size_t len = socket->read_some(boost::asio::buffer(buffer, clientMessageSize) );
            buffer[len] = '\0';
            if ( std::string( buffer ) == "quit" ) {
                clientRunning = false;
            } // if
        } // if

    } // while
    clientMapStringHash[clientId] = 0;
    logger().debug( "MapExplorerServer::%s - Client connection closed id '%d'", __FUNCTION__, clientId );
}


void MapExplorerServer::sendMap( const spatial::Octree3DMapManager&  map )
{
    boost::mutex::scoped_lock lock( this->mapMutex );
    this->latestMapString = Octree3DMapManager::toString( map );
    this->latestMapHash = boost::hash_value( this->latestMapString );
}
