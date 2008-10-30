/*
 * opencog/guile/SchemeSocket.cc
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

#include "SchemeSocket.h"

#include <string>
#include <sstream>

#include <opencog/guile/SchemeModule.h>
#include <opencog/guile/SchemeSmob.h>
#include <opencog/server/CogServer.h>
#include <opencog/util/Logger.h>

using namespace opencog;

SchemeSocket::SchemeSocket(ISocketHandler &handler)
    : TcpSocket(handler)
{
    logger().debug("[SchemeSocket] constructor (t: 0x%x)", pthread_self());
    SetLineProtocol(true);
    CogServer& cogserver = static_cast<CogServer&>(server());
    SchemeModule* module =
            static_cast<SchemeModule*>(cogserver.getModule(SchemeModule::id()));
    if (module) _shell = module->shell();
    else logger().error("[SchemeSocket] constructor: module \"%s\" has not been loaded");
}

SchemeSocket::~SchemeSocket()
{
    logger().debug("[SchemeSocket] destructor (t: 0x%x)", pthread_self());
}

void SchemeSocket::OnAccept()
{   
    logger().debug("[SchemeSocket] OnAccept (t: 0x%x)", pthread_self());
    if (Detach() == false) {
        logger().error("Unable to detach socket.");
        return;
    }
}

void SchemeSocket::OnDetached()
{
    logger().debug("[SchemeSocket] OnDetached (t: 0x%x)", pthread_self());
    SetNonblocking(true);

    /* init guile on this thread */
    scm_init_guile();
    SchemeSmob::init();

    Send(_shell->normal_prompt);
}

void SchemeSocket::OnLine(const std::string& line)
{
    logger().debug("[SchemeSocket] OnLine [%s]", line.c_str());
    _shell->eval(line, *this);
}
