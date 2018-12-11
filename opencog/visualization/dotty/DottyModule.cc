/*
 * opencog/dotty/DottyModule.cc
 *
 * Copyright (C) 2008 by OpenCog Foundation
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

#include <queue>
#include <sstream>
#include <string>

#include <boost/pointer_cast.hpp>

#include <opencog/atoms/atom_types/NameServer.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/base/Link.h>
#include <opencog/atoms/base/Node.h>
#include <opencog/cogserver/server/CogServer.h>
#include <opencog/util/Logger.h>

#include "DottyModule.h"

using namespace opencog;

DECLARE_MODULE(DottyModule);

class DottyGrapher
{
public:
    DottyGrapher(AtomSpace *space) : space(space), withIncoming(false), compact(false) { answer = ""; }
    AtomSpace *space;
    std::string answer;
    bool withIncoming;
    bool compact;

    /**
     * Outputs a dotty node for an atom.
     */
    bool do_nodes(const Handle& h)
    {
        AtomPtr a(h);

        if (compact)
        {
            // don't make nodes for binary links with no incoming
            LinkPtr l(LinkCast(a));
            if (l and l->getOutgoingSet().size() == 2 and
                     h->getIncomingSetSize() == 0)
                return false;
        }

        std::ostringstream ost;
        ost << h.value() << " [";
        if (!nameserver().isNode(a->get_type()))
            ost << "shape=\"diamond\" ";
        ost << "label=\"[" << nameserver().getTypeName(a->get_type()) << "]";
        if (nameserver().isNode(a->get_type())) {
            NodePtr n(NodeCast(a));
            ost << " " << n->get_name();
        } //else {
            // TODO: anything to output for links?
            //LinkPtr l = boost::dynamic_pointer_cast<Link>(a);
        //}
        ost << "\"];\n";
        answer += ost.str();
        return false;
    }

    /**
     * Outputs dotty links for an atom's outgoing connections.
     */
    bool do_links(const Handle& h)
    {
        AtomPtr a(h);
        std::ostringstream ost;

        //const Link *l = dynamic_cast<const Link *>(a);
        LinkPtr l(LinkCast(a));
        if (l)
        {
            const HandleSeq &out = l->getOutgoingSet();

            if (compact and out.size() == 2 and h->getIncomingSetSize() == 0)
            {
	            ost << out[0].value() << " -> " << out[1].value() << " [label=\""
                    << nameserver().getTypeName(a->get_type()) << "\"];\n";
                answer += ost.str();
                return false;
            }

            for (size_t i = 0; i < out.size(); i++) {
	            ost << h.value() << "->" << out[i].value()
	                << " [label=\"" << i << "\"];\n";
            }
        }

        if (withIncoming) {
            HandleSeq hs;
            h->getIncomingSet(back_inserter(hs));
            int i = 0;
            for (const Handle& h : hs) {
	            ost << h.value() << "->" << h.value()
	                << " [style=\"dotted\" label=\"" << i << "\"];\n";
                i++;
            }
        }

        answer += ost.str();
        return false;
    }

    void graph()
    {
        answer += "\ndigraph OpenCog {\n";
        space->foreach_handle_of_type((Type)ATOM, &DottyGrapher::do_nodes, this, true);
        space->foreach_handle_of_type((Type)ATOM, &DottyGrapher::do_links, this, true);
        answer += "}\n";
    }

};

DottyModule::DottyModule(CogServer& cs) : Module(cs)
{
    logger().info("[DottyModule] constructor");
    do_dotty_register();
}

DottyModule::~DottyModule()
{
    logger().info("[DottyModule] destructor");
    do_dotty_unregister();
}

void DottyModule::init()
{
    logger().info("[DottyModule] init");
}

std::string DottyModule::do_dotty(Request *dummy, std::list<std::string> args)
{
    AtomSpace *space = &CogServer::getAtomSpace();
    DottyGrapher g(space);
    while (!args.empty()) {
        if (args.front() == "with-incoming")
            g.withIncoming = true;
        if (args.front() == "--compact")
            g.compact = true;
        args.pop_front();
    }
    g.graph();
    return g.answer;
}

