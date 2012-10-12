/*
 * opencog/embodiment/Learning/LearningServer/LearningServer.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Shujing Ke rainkekekeke@gmail.com
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

//#ifdef HAVE_ZMQ

#include "LearningServer.h"
#include <opencog/util/Config.h>


using namespace opencog::learningserver;
using namespace opencog;

// new learning server for the Unity3D embodiment. not for the pet park.
LearningServer::LearningServer()
{
    this->zmqContext = new zmq::context_t(1);

    this->zmqPuller = new zmq::socket_t(*zmqContext, ZMQ_PULL);
    this->zmqPusher = new zmq::socket_t(*zmqContext, ZMQ_PUSH);

    this->pullIP = config().get("LEARNING_SERVER_PULL_IP");
    this->pullPort = config().get("LEARNING_SERVER_PULL_PORT");

    this->pushIP = config().get("LEARNING_SERVER_PUSH_IP");
    this->pushPort = config().get("LEARNING_SERVER_PUSH_PORT");

    string pullAddress = "tcp://" + this->pullIP + ":" + this->pullPort;
    string pushAddress = "tcp://" + this->pushIP + ":" + this->pushPort;
    this->zmqPuller->connect(pullAddress.c_str());
    this->zmqPusher->connect(pushAddress.c_str());
}

void LearningServer::run()
{
    while(1)
    {
        zmq::message_t message;
        this->zmqPuller->recv(&message);

    }
}

void LearningServer::createLearningTaskThread()
{

}

