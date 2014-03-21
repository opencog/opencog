/*
 * opencog/embodiment/Control/Procedure/ComboPAIExecutable.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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

#include "ComboShellServer.h"
#include <opencog/util/files.h>
#include <opencog/embodiment/Control/EmbodimentConfig.h>

using namespace opencog;
using namespace opencog::messaging;

int main(int argc, char** argv)
{
    string OAC_ID("OAC_npc");              // Network element ID of the OAC
    if (argc != 2) {
        cout << "OAC ID omitted, 'OAC_npc' will be used" << endl
             << "Usage recall:" << endl
             << argv[0] << " [OAC_ID]" << endl;
    } else {
        OAC_ID = argv[1];
    }

    // set up the system for talking to the router
    config(opencog::control::EmbodimentConfig::embodimentCreateInstance, true);

    // if exists load file with configuration parameters
    // IMPORTANT: this file should be the same for all executables that create
    // a EmbodimentConfig object.

    if (fileExists(config().get("CONFIG_FILE").c_str())) {
        config().load(config().get("CONFIG_FILE").c_str());
    }

    server(ComboShellServer::createInstance);
    ComboShellServer& css = static_cast<ComboShellServer&>(server());
    css.init(OAC_ID);
    css.serverLoop();
    return 0;
}
