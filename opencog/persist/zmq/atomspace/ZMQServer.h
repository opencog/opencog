/*
 * opencog/atomspace/ZMQServer.h
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

#ifndef _OPENCOG_ZMQ_SERVER_H
#define _OPENCOG_ZMQ_SERVER_H

#include <boost/thread.hpp>

#include "AtomSpace.h"
#include "Handle.h"
#include "types.h"
#include <lib/zmq/zmq.hpp>
#include <opencog/atomspace/ZMQMessages.pb.h>
#include <string>

using namespace std;

/*
TODO: remove text from this file, as an exact copy is in wiki
http://wiki.opencog.org/w/ZeroMQ

Setup:
	Install zeroMQ (should already be installed)
 		http://www.zeromq.org/intro:get-the-software
 		eclipse: add zmq to libraries
 	Install protocol buffers (should already be installed)
 		http://code.google.com/p/protobuf/downloads/list
 		./configure
 		make
 		sudo make install
 		eclipse: add protobuf to libraries
 	Compile proto file (if changed)
		protoc --cpp_out=. AtomSpaceMessages.proto

How to compile:
     Add new files and libs to opencog/atomspace/CMakeLists.txt
         add ZMQMessages.pb.cc, ZMQServer.cc, ZMQClient.cc, ProtocolBufferSerializer.cc
                 to ADD_LIBRARY (atomspace SHARED
         add protobuf to SET(ATOMSPACE_LINK_LIBS
         add ZMQMessages.pb.h, ZMQServer.h, ZMQClient.h, ProtocolBufferSerializer.h
                 to INSTALL (FILES
         run cmake .. in bin folder
    Add  -DZMQ_EXPERIMENT to CXX_DEFINES in the flags.make file in the following folders:
        /home/erwin/ochack/bin/opencog/atomspace/CMakeFiles/atomspace.dir
        /home/erwin/ochack/bin/opencog/server/CMakeFiles/cogserver.dir
        /home/erwin/ochack/bin/opencog/server/CMakeFiles/server.dir

How to use:
    Link with AtomSpace
    Create a ZMQClient instance and call its methods
        ZMQClient* zmqClient = new ZMQClient(); //defaults to localhost
        zmqClient->getAtom(h);
    If the cogserver is running on a different PC
        ZMQClient* zmqClient = new ZMQClient("tcp://168.0.0.21:5555");

How it works:
	ZeroMQ allows you to connect a client process to the atomspace managed by
	the server (normally the cogserver). Use cases include deploying an agent
	on a separate server, easier debugging of agents, connecting components
	that are written in different languages (any language that supports
	protocolbuffers (see http://code.google.com/p/protobuf/wiki/ThirdPartyAddOns)
	can use a simple message based wrapper to talk to the server. Of course
	implementing the full atomspace OO interface takes a lot more work. )
	and connecting tools that need high performance access to the atomspace
	(e.g. the atomspace visualizer).
	You can connect multiple clients to one server. The clients can be anywhere
	in the world as long as	the server is reachable via TCP/IP.

	Server:
		if server is enabled run zmqloop
		zmqloop
			check for RequestMessage
			switch(function)
				case getAtom:
					call getAtom with arguments from RequestMessage
					store return value in ReplyMessage
				case getName:
					etc...
			send ReplyMessage
	Client:
		async queue works the same as usual but if client is enabled call server instead of Run
			copy ASRequest arguments and function number to RequestMessage
			send RequestMessage to server
			wait for ReplyMessage
			copy ReplyMessage to ASRequest

Problem solving:
	It doesn't matter if you start the server or client first but if the server
	crashes or is restarted	you have to restart the client(s) as well.
	If you get error "Address already in use" then another instance of the
	server is already running

TODO:
    -febcorpus.scm doesn't load
	-clean exit of atomspaceasync without using sleep or a busy loop
		send ctrl+C signal to the thread and break when NULL message received
			char *reply = zstr_recv (client);
			if (!reply)
				if(exitSeverloop)
					logger.info "Serverloop exitting"
					break;
				else
					logger.error "Serverloop quit unexpectedly"
	-what if dowork throws an exception on the server? catch exceptions, put them in reply message and rethrow at client
		-try to handle communications exceptions locally
 */


class AtomSpaceAsyncUTest;

namespace opencog {
/** \addtogroup grp_atomspace
 *  @{
 */

class ZMQServer {
    zmq::context_t* zmqContext;
    zmq::socket_t *zmqClientSocket;
    boost::thread zmqServerThread;
    AtomSpace* atomSpace;

    void zmqLoop(string networkAddress);

public: 
    ZMQServer(AtomSpace* atomSpace1,
    		string networkAddress = "tcp://*:5555"); //"ipc:///tmp/AtomSpaceZMQ.ipc"
    ~ZMQServer();
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_ZMQ_SERVER_H
