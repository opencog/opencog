/*
 * opencog/spatial/MapExplorerServer.h
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

#ifndef MAPEXPLORERSERVER_H
#define MAPEXPLORERSERVER_H

#include <opencog/spatial/3DSpaceMap/Octree3DMapManager.h>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>

namespace opencog 
{
/** \addtogroup grp_spatial
 *  @{
 */
    namespace spatial 
    {

        /**
         * MapExplorer is a tool designed to visualize the state of
         * a LocalSpaceMap. This class starts a network service that
         * provides a mechanism for reading a LocalSpaceMap from a
         * given source and transmit it via network to one or more
         * clients. That way, if the source updates its LocalSpaceMap,
         * all the connected clients will receive an updated copy of
         * that.
         */
        class MapExplorerServer 
        {
        public:
            /**
             * This constructor receives a host and a port as arguments
             * and prepare the service to start running
             *
             * @param host ip or host used to bind the service
             * @param port port used to bind the service
             */
            MapExplorerServer( const std::string& host, const std::string& port );
            
            virtual ~MapExplorerServer( void );
            
            /**
             * Start the MapExplorer server. 
             * IMPORTANT: this method will start a thread that will run the service
             * and will let the application continue.
             */
            void start( void );

            /**
             * If you wish to block your program and wait for the server
             * to finish, call this method.
             */
            void wait( void );
            
            /**
             * Just finish the service
             */
            void stop( void );

            /**
             * Check if the service is running or not
             * @return true if yes or no otherwise
             */
            bool isRunning( void );
            
            /**
             * Thread safe sender map method. It can be used to
             * send a new map to the clients connected to the
             * server.
             * 
             * @param map A new LocalSpaceMap
             */
            void sendMap( const spatial::Octree3DMapManager& map );
            
        private:
            typedef boost::shared_ptr<boost::asio::ip::tcp::socket> SocketPtr;
            
            /**
             * Method called by the main thread
             */
            void serverLoop( void );

            /**
             * Method called by each thread created to new clients
             *
             * @param socket a smart pointer the the client socket
             */
            void clientLoop( SocketPtr socket );
            
            std::string host;
            std::string port;
            bool running;
            
            // visual debugger use a TCP connection
            // to visualize the current state of the LocalSpaceMap
            boost::asio::io_service visualDebuggerService;
            boost::asio::ip::tcp::acceptor* visualDebuggerAcceptor;
            
            boost::thread* service;
            boost::thread_group clients;
            boost::mutex mapMutex;

            std::vector<size_t> clientMapStringHash;
            std::string latestMapString;
            size_t latestMapHash;
            
        };
    } // spatial
/** @}*/
} // opencog

#endif // MAPEXPLORERSERVER_H
