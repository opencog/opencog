/*
 * opencog/server/Request.cc
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

#include "Request.h"

#include <opencog/util/Logger.h>
#include <opencog/util/exceptions.h>

using namespace opencog;

Request::Request(CogServer& cs) :
    _cogserver(cs), _requestResult(NULL), _mimeType("text/plain")
{
}

Request::~Request()
{
    logger().debug("[Request] destructor");
    if (_requestResult) _requestResult->OnRequestComplete();
}

void Request::setRequestResult(RequestResult* rr)
{
    logger().debug("[Request] setting requestResult: %p", rr);
    if (NULL == _requestResult) {
        _requestResult = rr;
        _mimeType = _requestResult->mimeType();
    } else if (NULL == rr) {
        _requestResult = NULL;  // used by exit/quit commands to not send any result.
    } else
        throw RuntimeException(TRACE_INFO,
            "Bad idea to try to set the RequestResult more than once.");
}

void Request::send(const std::string& msg) const
{
    if (_requestResult) _requestResult->SendResult(msg);
}

void Request::setParameters(const std::list<std::string>& params)
{
    _parameters.assign(params.begin(), params.end());
}

void Request::addParameter(const std::string& param)
{
    _parameters.push_back(param);
}
