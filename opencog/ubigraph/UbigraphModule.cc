/*
 * opencog/ubigraph/UbigraphModule.cc
 *
 * Copyright (C) 2008-2009 by Singularity Institute for Artificial Intelligence
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

#include <opencog/util/Logger.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/server/CogServer.h>

#include "UbigraphModule.h"

using namespace opencog;

DECLARE_MODULE(UbigraphModule);

UbigraphModule::UbigraphModule() : Module()
{
    logger().info("[UbigraphModule] constructor");
    do_ubigraph_register();    
}

UbigraphModule::~UbigraphModule()
{
    logger().info("[UbigraphModule] destructor");
    do_ubigraph_unregister();
}

void UbigraphModule::init()
{
    logger().info("[UbigraphModule] init");
}

std::string UbigraphModule::do_ubigraph(Request *dummy, std::list<std::string> args)
{
    while (!args.empty()) {
        if (args.front() == "with-incoming")
            g.withIncoming = true;
        if (args.front() == "--compact")
            g.compact = true;
        args.pop_front();
    }
    g.graph();
    return "";
}

