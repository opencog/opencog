/*
 * @file opencog/embodiment/Control/OperationalAvatarController/Plaza.cc
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2011-03-24
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

#include "Plaza.h"
#include <iostream>

using namespace OperationalAvatarController;

Plaza::Plaza (const std::string & ip, const std::string & publishPort) : ip(ip), publishPort(publishPort)
{
    // Initialize i/o context, which is thread safe and should be shared by all the mind agents
    this->zmqContext = new zmq::context_t(1); 

    this->pFrontend = new zmq::socket_t(*zmqContext, ZMQ_SUB); 
    this->pBackend = new zmq::socket_t(*zmqContext, ZMQ_PUB);

    // Subscribe on all the messages from all the mind agents
    this->pFrontend->setsockopt(ZMQ_SUBSCRIBE, "", 0); 

    // Provide the unified endpoint for all the subscribers 
    this->unifiedEndPoint = "tcp://" + ip + ":" + publishPort; 
    this->pBackend->bind(unifiedEndPoint.c_str());

    std::cout<<"ZeroMQ publish end point is: "<<unifiedEndPoint<<std::endl; 
}

Plaza::~Plaza ()
{
    delete this->pFrontend;
    delete this->pBackend; 
    // Delete context never succeed, it blocks here
//    delete this->zmqContext; 
}

void Plaza::addPublisher (const std::string & publishEndPoint)
{
    this->pFrontend->connect(publishEndPoint.c_str()); 
}

void Plaza::forwardMessages ()
{
    int64_t more;  // Indicate whether more message is followed
    size_t more_size = sizeof (more);

    // Get all the messages from front end and forward them to the back end
    while (1) {
        zmq::message_t message;

        if ( !this->pFrontend->recv(&message, ZMQ_NOBLOCK) )
            break; 

        this->pFrontend->getsockopt(ZMQ_RCVMORE, &more, &more_size);
        this->pBackend->send(message, more? ZMQ_SNDMORE: 0);
    }
}

bool Plaza::publishStringMore (zmq::socket_t & socket, const std::string & str)
{
    zmq::message_t message( str.size() );
    memcpy(message.data(), str.data(), str.size());
    return socket.send(message, ZMQ_SNDMORE);
}

bool Plaza::publishString (zmq::socket_t & socket, const std::string & str) 
{
    zmq::message_t message( str.size() );
    memcpy(message.data(), str.data(), str.size());
    return socket.send(message, 0);
}

#endif // HAVE_ZMQ
