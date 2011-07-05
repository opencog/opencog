/*
 * opencog/embodiment/Control/OperationalAvatarController/OACTester.cc
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

#include "OAC.h"
#include <opencog/embodiment/Control/EmbodimentConfig.h>
#include <opencog/embodiment/Control/MessagingSystem/StringMessage.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

#include <opencog/util/Logger.h>

using namespace std;
using namespace opencog;
using namespace opencog::oac;
using opencog::messaging::StringMessage;

int main()
{
    config(opencog::control::EmbodimentConfig::embodimentCreateInstance, true);

    StringMessage *msg = new StringMessage("", "", "");
    int length;
    char * buffer;

    ifstream is;
    is.open ("arquivo.xml", ios::binary );

    // get length of file:
    is.seekg (0, ios::end);
    length = is.tellg();
    is.seekg (0, ios::beg);


    // allocate memory:
    buffer = new char [length];

    // read data as a block:
    is.read (buffer, length);
    is.close();
    string xml = buffer;

    server(OAC::createInstance);
    OAC& opc = static_cast<OAC&>(server());
    opc.init("teste-opc", "127.0.0.1", 4000, "5000", "1", "2", "pet", "neutral");
    msg->setMessage(xml);
    opc.processNextMessage(msg);

    delete[] buffer;
    // TODO: how to delete opc now?
    //delete opc;
    return 0;
}

