/*
 * opencog/cogserver/server/ShutdownRequest.cc
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

#include <sstream>

#include <opencog/util/oc_assert.h>
#include <opencog/cogserver/server/CogServer.h>
#include <opencog/cogserver/server/ConsoleSocket.h>

#include "ShutdownRequest.h"

using namespace opencog;

ShutdownRequest::ShutdownRequest(CogServer& cs) : Request(cs)
{
}

ShutdownRequest::~ShutdownRequest()
{
}

bool ShutdownRequest::execute()
{
    std::ostringstream oss;
    oss << "Shutting down cogserver" << std::endl;
    send(oss.str());

    _cogserver.stop();

    ConsoleSocket* con = get_console();
    OC_ASSERT(con, "Bad request state");
    set_console(nullptr);
    con->Exit();

    return true;
}
