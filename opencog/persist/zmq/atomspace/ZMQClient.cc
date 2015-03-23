/*
 * opencog/atomspace/ZMQClient.cc
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

#include "ZMQClient.h"
#include <lib/zmq/zmq.hpp>
#include <opencog/atomspace/ZMQMessages.pb.h>

using namespace opencog;

ZMQClient::ZMQClient(string networkAddress)
{
	zmqContext= new zmq::context_t(1);
    zmqClientSocket = new zmq::socket_t(*zmqContext, ZMQ_REQ);
    zmqClientSocket->connect (networkAddress.c_str());
}

ZMQClient::~ZMQClient()
{
	delete zmqContext;
	delete zmqClientSocket;
};

void ZMQClient::SendMessage(ZMQRequestMessage& requestMessage,
        ZMQReplyMessage& replyMessage)
{
    // Send request to server
    string strRequest = requestMessage.SerializeAsString();
    zmq::message_t request(strRequest.size());
    memcpy((void *) request.data (), strRequest.c_str(),
            strRequest.size()); //TODO use copyless init from data
    zmqClientSocket->send(request);

    // Wait for reply
    zmq::message_t reply;
    zmqClientSocket->recv(&reply);
    replyMessage.ParseFromArray(reply.data(), reply.size());
}

AtomPtr ZMQClient::getAtom(Handle& h)
{
    ZMQRequestMessage req;
    ZMQReplyMessage rep;

    req.set_function(ZMQgetAtom);
    req.set_handle(h.value());
    SendMessage(req, rep);
    Atom* atom = ProtocolBufferSerializer::deserialize(rep.atom());
    return AtomPtr(atom);
};



