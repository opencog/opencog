/*
 * opencog/embodiment/Control/MessagingSystem/DummyServerExecutable.cc
 *
 * Copyright (C) 2007-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Andre Senna
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
#include <SystemParameters.h>
#include "DummyServer.h"
#include <opencog/util/files.h>

using namespace MessagingSystem;

int main(int argc, char *argv[])
{

    Control::SystemParameters parameters;

    // if exists load file with configuration parameters
    // IMPORTANT: this file should be the same for all executables that create
    // a systemParameter object.
    if (fileExists(parameters.get("CONFIG_FILE").c_str())) {
        parameters.loadFromFile(parameters.get("CONFIG_FILE"));
    }

    DummyServer *server = new DummyServer(parameters, argv[1], argv[2], atoi(argv[3]));
    server->serverLoop();
    return 0;
}

