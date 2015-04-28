/*
 * @file opencog/embodiment/Control/OperationalAvatarController/Plaza.h
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2011-03-25
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

#ifdef HAVE_ZMQ

#ifndef PLAZA_H
#define PLAZA_H

#include <string>
#include <stdlib.h>

#include <lib/zmq/zmq.hpp>

namespace opencog { namespace oac {

/**
 * @class
 *
 * @brief Customized device for delivering messages via zeromq 
 *
 * @note 
 *
 * Plaza is a Spanish word related to "field" which describes an open urban 
 * public space, such as a city square (see also http://en.wikipedia.org/wiki/Plaza).
 * 
 * Each OAC holds a Plaza which provides a public field that all the mind agents 
 * can publish their specific messages there with respective sockets. While subscribers,
 * such as a visualization monitor can easily get these messages within Plaza. 
 * 
 * Technically speaking, each mind agent that wants to send messages should have its own 
 * socket and publish its messages to a specific endpoint. All the mind agents share the
 * same context but not the socket and endpoint, because socket is not thread safe and
 * endpoint is of exclusive resource. 
 *
 * Then the Plaza within OAC would subscribe all the messages from all the mind agents 
 * and then forward them to a single endpoint. 
 *
 * Finally subscribers can access all the messages published by the mind agents via either 
 * the unified endpoint provided by the Plaza or the endpoint provided by the specific 
 * mind agent. The first approach seems more elegant while if efficiency really matters,
 * you can also choose the latter one. 
 *
 * Within mind agents: 
 *                                            _______
 * mind agent 1 (socket 1) => end point 1 => |       |
 * mind agent 2 (socket 2) => end point 2 => | Plaza | => Unified end point P
 * mind agent 3 (socket 3) => end point 3 => |_______| 
 * ...             ...        ...               
 * 
 * For subscribers (either approach works): 
 *
 * Approach 1 (more elegant, recommend): 
 *  _______                             --------------------------------------
 * |       |                           | => subscriber a (socket a, filter a) |
 * | Plaza | => Unified end point P => | => subscriber b (socket b, filter b) | 
 * |_______|                           | ...                                  |
 *                                      --------------------------------------
 *                                                        
 * Approach 2 (only when efficiency matters):
 *
 * end point 1 => subscriber c (socket c)
 * end point 2 => subscriber d (socket d) 
 * ...
 *
 */
class Plaza
{
public:

    /**
     * Constructor
     *
     * @param ip            All the messages of mind agents would be forwarded to this ip
     * @param publishPort   All the messages of mind agents would be forwarded to this port 
     *
     * @note Plaza use TCP protocol to publish messages
     */
    Plaza (const std::string & ip, const std::string & publishPort); 
    ~Plaza ();

    /**
     * Add publisher to the plaza given its publish end point
     *
     * @publishEndPoint The publisher send its messages to this end point, which would be 
     *                  subscribed by plaza
     */
    void addPublisher (const std::string & publishEndPoint);

    /**
     * Get all the messages from front end and forward them to the back end 
     */
    void forwardMessages ();

    /**
     * Publish a string via the given socket with more following messages
     *
     * @note Before publishing any strings, you should create a socket with ZMQ_PUB
     *       and bind it to a specific end point, any protocol works
     */
    static bool publishStringMore (zmq::socket_t & socket, const std::string & str);

    /**
     * Publish a string via the given socket with no following messages
     *
     * @note Before publishing any strings, you should create a socket with ZMQ_PUB
     *       and bind it to a specific end point, any protocol works
     */
    static bool publishString (zmq::socket_t & socket, const std::string & str);    

    /**
     * A bunch of simple getters
     */
    zmq::context_t & getZmqContext () {
        return * this->zmqContext; 
    }

    const std::string & getIP () {
        return this->ip;
    }

    const std::string & getPublishPort () {
        return this->publishPort; 
    }

    /**
     * Return the end point that subscribers can connect to 
     */
    const std::string & getUnifiedEndPoint () {
        return this->unifiedEndPoint; 
    }

    /**
     * Handy converters between message and other data type
     */
    static void messageToString (zmq::message_t & message, std::string & str) {
        str = std::string (static_cast<char *>(message.data()), message.size());
    }

    static void stringToMessage (const std::string & str, zmq::message_t & message) {
        message.rebuild(str.size());
        memcpy(message.data(), str.data(), str.size());
    }

private:
    zmq::context_t * zmqContext; 
    const std::string ip;
    const std::string publishPort;      
    std::string unifiedEndPoint; 

    zmq::socket_t * pFrontend;  // Connect with mind agents 
    zmq::socket_t * pBackend;   // Bind to unifiedEndPoint, provide the public endpoint for subscribers
}; // class 

} } // namespace opencog::oac

#endif // PLAZA_H

#endif // HAVE_ZMQ
