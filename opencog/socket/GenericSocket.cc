/*
 * opencog/socket/GenericSocket.cc
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

#include "GenericSocket.h"

#include <string>
#include <sstream>

#include <opencog/socket/GenericModule.h>
#include <opencog/server/CogServer.h>
#include <opencog/util/Logger.h>

using namespace opencog;

GenericSocket::GenericSocket(ISocketHandler &handler)
    : TcpSocket(handler)
{
    logger().debug("[GenericSocket] constructor (t: 0x%x)", pthread_self());
    SetLineProtocol(true);
    _shell = NULL;
#ifdef OLD_WAY_MAYBE_OBSOLETE
    CogServer& cogserver = static_cast<CogServer&>(server());

    // XXX unclear ... 
    // what is the point of this lookup from id?
    GenericModule* module =
            static_cast<GenericModule*>(cogserver.getModule(GenericModule::id());
    if (module) _shell = module->shell();
    else logger().error("[GenericSocket] constructor: module \"%s\" has not been loaded");
#endif
}

void GenericSocket::DeclareShell(GenericShell *s)
{
	_shell = s;
}

GenericSocket::~GenericSocket()
{
    logger().debug("[GenericSocket] destructor (t: 0x%x)", pthread_self());
}

void GenericSocket::OnAccept()
{   
    logger().debug("[GenericSocket] OnAccept (t: 0x%x)", pthread_self());
    if (Detach() == false) {
        logger().error("Unable to detach socket.");
        return;
    }
}

void GenericSocket::OnDetached()
{
    logger().debug("[GenericSocket] OnDetached (t: 0x%x)", pthread_self());
    SetNonblocking(true);

    Send(_shell->normal_prompt);
}

void GenericSocket::OnLine(const std::string& line)
{
    logger().debug("[GenericSocket] OnLine [%s]", line.c_str());
    _shell->eval(line, *this);
}
