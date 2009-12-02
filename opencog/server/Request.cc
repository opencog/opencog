/*
 * opencog/server/Request.cc
 *
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
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

#include "Request.h"

#include <opencog/server/IHasMimeType.h>
#include <opencog/server/IRPCSocket.h>
#include <opencog/util/Logger.h>
#include <opencog/util/exceptions.h>

using namespace opencog;

Request::Request() : _socket(NULL)
{
}

Request::~Request()
{
    logger().debug("[Request] destructor");
    if (_socket) _socket->OnRequestComplete();
}

void Request::setSocket(ConsoleSocket* s)
{
    logger().debug("[Request] setting socket: %p", s);
    if (NULL == _socket) {
        _socket = s;
        _mimeType = _socket->mimeType();
    } else
        throw RuntimeException(TRACE_INFO,
            "Bad idea to try to set the socket more than once.");
}

void Request::send(const std::string& msg) const
{
    if (_socket) _socket->Send(msg);
}

void Request::setParameters(const std::list<std::string>& params)
{
    _parameters.assign(params.begin(), params.end());
}

void Request::addParameter(const std::string& param)
{
    _parameters.push_back(param);
}
