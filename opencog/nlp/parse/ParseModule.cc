/*
 * opencog/nlp/ParseModule.cc
 *
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by Trent Waddington <trent.waddington@gmail.com>
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

#ifdef HAVE_LINK_GRAMMAR

#include <queue>
#include <sstream>
#include <string>

#include "ParseModule.h"
#include <opencog/util/Logger.h>
#include <opencog/atomspace/utils.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/server/CogServer.h>
#include <opencog/nlp/wsd/atom_types.h>

using namespace opencog;

DECLARE_MODULE(ParseModule);

ParseModule::ParseModule() : Module()
{
    logger().info("[ParseModule] constructor");
    do_hear_register();
    CogServer& cogserver = static_cast<CogServer&>(server());
    cogserver.registerAgent(ParsingAgent::info().id, &parsingFactory);
}

ParseModule::~ParseModule() {
    logger().info("[ParseModule] destructor");
    do_hear_unregister();
    CogServer& cogserver = static_cast<CogServer&>(server());
    cogserver.unregisterAgent(ParsingAgent::info().id);
}

void ParseModule::init()
{
    logger().info("[ParseModule] init");
}

std::string ParseModule::do_hear(Request *dummy, std::list<std::string> args)
{
    AtomSpace *space = CogServer::getAtomSpace(); 
    std::string str;
    for (std::list<std::string>::iterator it = args.begin(); it != args.end(); it++)
    {
        if (it != args.begin())
            str += " ";
        str += *it;
    }

    Handle hSentence = space->addNode(SENTENCE_NODE, str);

    return hSentence.str(); 
}

#endif // HAVE_LINK_GRAMMAR
