/*
 * opencog/atomspace/ZMQClient.h
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

#ifndef _OPENCOG_ZMQ_CLIENT_H
#define _OPENCOG_ZMQ_CLIENT_H

#include "opencog/atomspace/Atom.h"
#include "opencog/atomspace/Handle.h"
#include "types.h"
#include <lib/zmq/zmq.hpp>
#include <opencog/atomspace/ZMQMessages.pb.h>
#include <string>
#include <boost/thread.hpp>

using namespace std;

namespace opencog {
/** \addtogroup grp_atomspace
 *  @{
 */

class ZMQClient {
    zmq::socket_t* zmqClientSocket;
    zmq::context_t* zmqContext;

    void SendMessage(ZMQRequestMessage& requestMessage,
            ZMQReplyMessage& replyMessage);

public:
    ZMQClient(string networkAddres="tcp://127.0.0.1:5555"); //"ipc:///tmp/AtomSpaceZMQ.ipc"
    ~ZMQClient();

    AtomPtr getAtom(Handle& h);
    
    /** @todo add other functions as needed */
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_ZMQ_CLIENT_H
