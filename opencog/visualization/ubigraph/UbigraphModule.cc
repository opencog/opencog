/*
 * opencog/visualization/ubigraph/UbigraphModule.cc
 *
 * Copyright (C) 2008-2009 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Jared Wigmore <jared.wigmore@gmail.com>
 * Adapted from DottyModule (which is by Trent Waddington <trent.waddington@gmail.com>)
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

#include <queue>
#include <sstream>
#include <string>

// To parse and check ascii IP address optionally given by user
#include <arpa/inet.h>

#include <opencog/util/Logger.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/server/CogServer.h>
#include <opencog/util/mt19937ar.h>


#include "UbigraphModule.h"

using namespace opencog;

DECLARE_MODULE(UbigraphModule);

UbigraphModule::UbigraphModule(CogServer& cs) : Module(cs)
{
    logger().info("[UbigraphModule] constructor");
    do_ubigraphUpdate_register();    
    do_ubigraphRandomSTI_register();    
    do_ubigraph_register();    
}

UbigraphModule::~UbigraphModule()
{
    logger().info("[UbigraphModule] destructor");
    do_ubigraphUpdate_unregister();    
    do_ubigraphRandomSTI_unregister();    
    do_ubigraph_unregister();
}

void UbigraphModule::init()
{
    logger().info("[UbigraphModule] init");
}

std::string UbigraphModule::do_ubigraph(Request *dummy, std::list<std::string> args)
{
    std::string serverIP = "";
    int port = 0;
    while (!args.empty()) {
        if (args.front().compare(0,2,"--") == 0) {
            if (args.front() == "--with-incoming")
                g.withIncoming = true;
            if (args.front() == "--compact")
                g.compact = true;
        } else {
            struct sockaddr_in sa;
            // Should be an IP address and port
            if ( args.front().find(':') == std::string::npos )
                return "IP and port formatting error. Expects x.x.x.x:port";
            serverIP = args.front().substr(0,args.front().find(':'));
            int result = inet_pton(AF_INET, serverIP.c_str(), &(sa.sin_addr));
            if (result == 0) {
                // Not a valid IP address
                serverIP = "";
                port = 0;
                return "IP address invalid.";
            }
            std::istringstream myStream(args.front().substr(args.front().find(':')+1));
            if (!(myStream >> port)) {
                // conversion of port string to int failed
                serverIP = "";
                port = 0;
                return "Port invalid.";
            }
        }
        args.pop_front();
    }
    if (port == 0)
        g.init();
    else
        g.init(serverIP, port);
    if (g.isConnected()) {
        g.watchSignals();
        g.graph();
    } else {
        return "Failed to connect to " + g.getServerString();
    }
    return "Now connected to Ubigraph server.";
}

std::string UbigraphModule::do_ubigraphUpdate(Request *dummy, std::list<std::string> args)
{
    g.updateSizeOfType(NODE, Ubigrapher::STI, 15.0f);
    return "";
}

std::string UbigraphModule::do_ubigraphRandomSTI(Request *dummy, std::list<std::string> args)
{
    HandleSeq hs;
    MT19937RandGen rng(1);
    std::back_insert_iterator< HandleSeq > out_hi(hs);
    int nNodes = 2;

    if (!args.empty()) nNodes = atoi(args.front().c_str());
    _cogserver.getAtomSpace().getHandleSet(out_hi, NODE, true);
    if (hs.size() == 0) return "";
    while (nNodes > 0) {
        _cogserver.getAtomSpace().setSTI(hs[rng.randint(hs.size())], 1000);
        nNodes--;
    }
    _cogserver.getAtomSpace().updateMaxSTI(1000);

    return "";
}

