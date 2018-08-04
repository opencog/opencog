/*
 * opencog/cogserver/server/Request.cc
 *
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Gustavo Gama <gama@vettalabs.com>
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

#include <opencog/util/exceptions.h>
#include <opencog/util/Logger.h>
#include <opencog/util/oc_assert.h>

#include <opencog/cogserver/server/ConsoleSocket.h>

#include "Request.h"

using namespace opencog;

Request::Request(CogServer& cs) :
    _console(nullptr), _cogserver(cs)
{
}

Request::~Request()
{
    logger().debug("[Request] destructor");
    if (_console)
    {
        _console->OnRequestComplete();
        _console->put();  // dec use count we are done with it.
    }
}

void Request::set_console(ConsoleSocket* con)
{
    // The "exit" request causes the console to be destroyed,
    // rendering the _console pointer invalid. However, generic
    // code will try to call Request::send() afterwards. So
    // prevent the invalid reference by zeroing he pointer.
    if (nullptr == con)
    {
        _console->put();  // dec use count -- we are done with socket.
        _console = nullptr;
        return;
    }

    OC_ASSERT(nullptr == _console, "Setting console twice!");

    logger().debug("[Request] setting socket: %p", con);
    con->get();  // inc use count -- we plan to use the socket
    _console = con;
}

void Request::send(const std::string& msg) const
{
    // The _console might be zero for the exit request, because the
    // exit command destroys the socket, and then tries to send a
    // reply on the socket it just destroyed.
    if (_console) _console->SendResult(msg);
}

void Request::setParameters(const std::list<std::string>& params)
{
    _parameters.assign(params.begin(), params.end());
}

void Request::addParameter(const std::string& param)
{
    _parameters.push_back(param);
}
