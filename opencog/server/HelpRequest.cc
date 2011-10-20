/*
 * opencog/server/HelpRequest.cc
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

#include "HelpRequest.h"

#include <iomanip>
#include <sstream>

#include <opencog/server/CogServer.h>
#include <opencog/server/Request.h>

#include <opencog/util/Config.h>
#include <opencog/util/ansi.h>

using namespace opencog;

HelpRequest::HelpRequest()
{
}

HelpRequest::~HelpRequest()
{
}

bool HelpRequest::execute()
{
    std::ostringstream oss;
    CogServer& cogserver = static_cast<CogServer&>(server());

    if (_parameters.empty()) {
        std::list<const char*> commands = cogserver.requestIds();

        size_t maxl = 0;
        std::list<const char*>::const_iterator it;
        for (it = commands.begin(); it != commands.end(); ++it) {
            size_t len = strlen(*it);
            if (len > maxl) maxl = len;
        }

        oss << "Available commands:" << std::endl;
        for (it = commands.begin(); it != commands.end(); ++it) {
            // Skip hidden commands
            if (cogserver.requestInfo(*it).hidden) continue;
            std::string cmdname(*it);
            std::string ansi_cmdname;
            ansi_green(ansi_cmdname); ansi_bright(ansi_cmdname);
            ansi_cmdname.append(cmdname);
            ansi_off(ansi_cmdname);
            ansi_green(ansi_cmdname);
            ansi_cmdname.append(":");
            ansi_off(ansi_cmdname);
            size_t cmd_length = strlen(cmdname.c_str());
            size_t ansi_code_length = strlen(ansi_cmdname.c_str()) - cmd_length;
            oss << "  " << std::setw(maxl+ansi_code_length+2) << std::left << ansi_cmdname
                << cogserver.requestInfo(*it).description << std::endl;
        }
    } else if (_parameters.size() == 1) {
        const RequestClassInfo& cci = cogserver.requestInfo(_parameters.front());
        if (cci.help != "")
            oss << cci.help << std::endl;
    } else {
        oss << info().help << std::endl;
    }

    if (_mimeType == "text/plain")
        send(oss.str());

    return true;
}
