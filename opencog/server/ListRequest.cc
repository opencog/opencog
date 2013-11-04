/*
 * opencog/server/ListRequest.cc
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

#include "ListRequest.h"

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/types.h>
#include <opencog/server/CogServer.h>

using namespace opencog;

ListRequest::ListRequest(CogServer& cs) : Request(cs)
{
}

ListRequest::~ListRequest()
{
    logger().debug("[ListRequest] destructor");
}

bool ListRequest::syntaxError()
{
    _error << "invalid syntax" << std::endl;
    sendError();
    return false;
}

bool ListRequest::execute()
{
    std::string name = "";
    Type type = NOTYPE;
    Handle handle = Handle::UNDEFINED;
    bool subtypes = false;
    AtomSpace& as = _cogserver.getAtomSpace();
    std::ostringstream err;

    std::list<std::string>::const_iterator it;
    for (it = _parameters.begin(); it != _parameters.end(); ++it) {
        if (*it == "-h") { // filter by handle
            ++it;
            if (it == _parameters.end()) return syntaxError();
            UUID uuid = strtol((*it).c_str(), NULL, 0);
            handle = Handle(uuid);
            if (!as.isValidHandle(handle)) {
                _error << "invalid handle" << std::endl;
                sendError();
                return false;
            }
            _handles.push_back(handle);
        } else if (*it == "-n")  { // filter by name
            ++it;
            if (it == _parameters.end()) return syntaxError();
            name.assign(*it);
        } else if (*it == "-t") { // filter by type, excluding subtypes
            ++it;
            if (it == _parameters.end()) return syntaxError();
            type = classserver().getType((*it).c_str());
            if (type == NOTYPE) {
                _error << "invalid type" << std::endl;
                sendError();
                return false;
            }
        } else if (*it == "-T") { // filter by type, including subtypes
            ++it;
            if (it == _parameters.end()) return syntaxError();
            type = classserver().getType((*it).c_str());
            if (type == NOTYPE) {
                _error << "invalid type" << std::endl;
                sendError();
                return false;
            }
            subtypes = true;
        }
    }
    if (name != "" && type != NOTYPE) { // filter by name & type
        as.getHandlesByName
            (std::back_inserter(_handles), name.c_str(), type, subtypes);
    } else if (name != "") {     // filter by name
        as.getHandlesByName(std::back_inserter(_handles), name.c_str(), ATOM, true);
    } else if (type != NOTYPE) { // filter by type
        as.getHandlesByType(std::back_inserter(_handles), type, subtypes);
    } else {
        as.getHandlesByType(back_inserter(_handles), ATOM, true);
    }
    sendOutput();
    return true;
}

void ListRequest::sendOutput()
{
    std::ostringstream oss;

    if (_mimeType == "text/plain") {
        AtomSpace& as = _cogserver.getAtomSpace();
        std::vector<Handle>::const_iterator it; 
        for (it = _handles.begin(); it != _handles.end(); ++it) {
            oss << as.atomAsString(*it) << std::endl;
        }
    } else throw RuntimeException(TRACE_INFO, "Unsupported mime-type: %s",
            _mimeType.c_str());

    send(oss.str());
}

void ListRequest::sendError()
{
    if (_mimeType != "text/plain")
        throw RuntimeException(TRACE_INFO, "Unsupported mime-type: %s",
                _mimeType.c_str());
    send(_error.str());
}
