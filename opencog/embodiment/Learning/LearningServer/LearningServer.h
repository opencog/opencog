/*
 * opencog/embodiment/Learning/LearningServer/LearningServer.h
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

#ifndef LEARNING_SERVER_H
#define LEARNING_SERVER_H

#ifdef HAVE_ZMQ

#include <lib/zmq/zmq.hpp>
#include <string>

namespace opencog { namespace learningserver {

class LearningServer
{
public:
    LearningServer();
    ~LearningServer(){}

    void run();

protected:
    std::string pullIP;
    std::string pullPort;

    std::string pushIP;
    std::string pushPort;

    zmq::context_t * zmqContext;
    zmq::socket_t * zmqPuller;
    zmq::socket_t * zmqPusher;

    void createLearningTaskThread();

};
}} // opencog::learningserver

#endif // HAVE_ZMQ

#endif // LEARNING_SERVER_H
