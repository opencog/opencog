/*
 * opencog/atomspace/ZMQServer.cc
 *
 * Copyright (C) 2008-2010 OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Erwin Joosten
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

#include "ZMQServer.h"

using namespace opencog;

ZMQServer::ZMQServer(AtomSpace* atomSpace1, string networkAdress)
{
	zmqContext= new zmq::context_t(1);
	atomSpace = atomSpace1;
	zmqServerThread = boost::thread(boost::bind(&ZMQServer::zmqLoop, this, networkAdress));
}

ZMQServer::~ZMQServer()
{
    /** TODO send signal to exit gracefully@n
     * zmqServerThread.join();
     */
};

void ZMQServer::zmqLoop(string networkAddress)
{
    zmq::socket_t zmqServerSocket (*zmqContext, ZMQ_REP);
    zmqServerSocket.bind (networkAddress.c_str());

    while (true)
    {
        //  Wait for next request from client
        zmq::message_t request;
        zmqServerSocket.recv (&request);
        //TODO check for exit signal
 	 	//TODO check for errors, log and retry?
        ZMQRequestMessage requestMessage;
		requestMessage.ParseFromArray(request.data(), request.size());

		ZMQReplyMessage replyMessage;
		switch(requestMessage.function())
		{
			case ZMQgetAtom:
			{
				AtomPtr atom = atomSpace->cloneAtom(
				        Handle(requestMessage.handle()));
				ProtocolBufferSerializer::serialize(*atom, replyMessage.mutable_atom());
				break;
			}
			//TODO add other functions
			default:
				assert(!"Invalid ZMQ function");
		}

		// Send reply back to client
		string strReply = replyMessage.SerializeAsString();
		zmq::message_t reply (strReply.size());
		memcpy ((void *) reply.data (), strReply.c_str(), strReply.size()); //TODO in place init
		zmqServerSocket.send (reply);
    }
}

