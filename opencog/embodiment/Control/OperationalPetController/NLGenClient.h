/*
 * opencog/embodiment/Control/OperationalPetController/NLGenClient.h
 *
 * Copyright (C) 2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Fabricio Silva 
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

#ifndef NLGENCLIENT_H
#define NLGENCLIENT_H

#include <string>
#include <cstdlib>
#include <iostream>
#include <boost/version.hpp>

#define BOOST_MINOR_VERSION (BOOST_VERSION / 100 % 1000)
#define BOOST_ACCEPTED_VERSION 35

#if BOOST_MINOR_VERSION >= BOOST_ACCEPTED_VERSION
#include <boost/asio.hpp>
using boost::asio::ip::tcp;
#endif

namespace OperationalPetController
{
    class NLGenClient {
        public:
            NLGenClient(std::string host, int port){

#if BOOST_MINOR_VERSION >= BOOST_ACCEPTED_VERSION

                this->port = port;
                this->host = host;
                connect();
#endif
            }
            ~NLGenClient(){
#if BOOST_MINOR_VERSION >= BOOST_ACCEPTED_VERSION
                logger().debug("[NLGenClient.%s] - closing socket connection",__FUNCTION__);
                socket->close();
                delete socket;
                logger().debug("[NLGenClient.%s] - socket connection closed with success",__FUNCTION__);
#endif
            }

            bool connect(){
#if BOOST_MINOR_VERSION >= BOOST_ACCEPTED_VERSION
                
                if(connected){
                    return true;                
                }
               try{
                    logger().debug("[NLGenClient.%s] - opening socket connection",__FUNCTION__);
                     
                    tcp::resolver resolver(io_service);
                    tcp::resolver::query query(tcp::v4(), host, boost::lexical_cast<std::string>(port));
                    tcp::resolver::iterator iterator = resolver.resolve(query);
                    socket = new tcp::socket(io_service);
                    socket->connect(*iterator);
                    logger().debug("[NLGenClient.%s] - socket connection opened with success",__FUNCTION__);
                    connected = true;
                }catch(std::exception& e){
                    logger().error("[NLGenClient.%s] - Failed to open socket. Exception Message: %s",__FUNCTION__,e.what());
                    connected = false;
                }
               return connected;
#else
                logger().debug("[NLGenClient.%s] - Failed to connect to the socket server. BOOST Version must be equals or greater than %d but is %d",__FUNCTION__,BOOST_ACCEPTED_VERSION, BOOST_MINOR_VERSION);
#endif
            }

            std::string send(std::string text){
#if BOOST_MINOR_VERSION >= BOOST_ACCEPTED_VERSION
                logger().debug("[NLGenClient.%s] - sending text %s",__FUNCTION__,text.c_str());

                if(!connect()){//if it is not connect, try to connect
                    logger().debug("[NLGenClient.%s] - it was not possible to send the text to nlgen because the socket is closed",__FUNCTION__);
                    return "";
                }

                size_t request_length = text.length();
                boost::asio::write(*socket, boost::asio::buffer(text.c_str(), request_length));
                char reply[1024];//TODO max of sentence size is 1024.....
                size_t reply_length = socket->read_some(boost::asio::buffer(reply));
                //size_t reply_length = boost::asio::read(*socket,boost::asio::buffer(reply));
                std::string nlgenResult = std::string(reply); 
                logger().debug("[NLGenClient.%s] - read %d characters",__FUNCTION__,reply_length);
                return nlgenResult.substr(0,reply_length-1);
#else
                logger().debug("[NLGenClient.%s] - Failed to send text to the socket server. BOOST Version must be equals or greater than %d but is %d",__FUNCTION__,BOOST_ACCEPTED_VERSION,BOOST_MINOR_VERSION);
                return "";
#endif
            }

        private:
#if BOOST_MINOR_VERSION >= BOOST_ACCEPTED_VERSION
            boost::asio::io_service io_service;
            tcp::socket *socket;
            std::string host; 
            int port;
            bool connected;
#endif
    };
};

#endif //NLGENCLIENT_H

