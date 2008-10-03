/*
 * src/modules/dotty.cc
 *
 * Copyright (C) 2008 by Trent Waddington <trent.waddington@gmail.com>
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

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/server/CogServer.h>
#include <opencog/util/platform.h>

using namespace opencog;

class DottyGrapher
{
public:
    DottyGrapher(AtomSpace *space) : space(space), withIncoming(false) { answer = ""; }
    AtomSpace *space;
    std::string answer;
    bool withIncoming;

    /**
     * Outputs a dotty node for an atom.
     */
    bool do_nodes(const Atom *a)
    {
        std::ostringstream ost;
        ost << TLB::getHandle(a) << " [";
        if (!space->isNode(a->getType()))
            ost << "shape=\"diamond\" ";
        ost << "label=\"[" << ClassServer::getTypeName(a->getType()) << "]";
        if (space->isNode(a->getType())) {
            Node *n = (Node*)a;
            ost << " " << n->getName();
        } else {
            Link *l = (Link*)a;
            l = l; // TODO: anything to output for links?
        }
        ost << "\"];\n";
        answer += ost.str();
        return false;
    }

    /**
     * Outputs dotty links for an atom's outgoing connections.
     */
    bool do_links(const Atom *a)
    {
        Handle h = TLB::getHandle(a);
        std::ostringstream ost;
        const std::vector<Handle> &out = a->getOutgoingSet();
        for (size_t i = 0; i < out.size(); i++) {
            ost << h << "->" << out[i] << " [label=\"" << i << "\"];\n";
        }

        if (withIncoming) {
            HandleEntry *he = a->getIncomingSet();
            int i = 0;
            while (he) {
                ost << h << "->" << he->handle << " [style=\"dotted\" label=\"" << i << "\"];\n";
                he = he->next;
                i++;
            }
        }

        answer += ost.str();
        return false;
    }

    void graph()
    {
        answer += "\ndigraph OpenCog {\n";
        space->getAtomTable().foreach_atom(&DottyGrapher::do_nodes, this);
        space->getAtomTable().foreach_atom(&DottyGrapher::do_links, this);
        answer += "}\n";
    }

};

extern "C" std::string cmd_dotty(std::queue<std::string> &args)
{
    AtomSpace *space = CogServer::getAtomSpace();
    DottyGrapher g(space);
    while (!args.empty()) {
        if (args.front() == "with-incoming")
            g.withIncoming = true;
        args.pop();
    }
    g.graph();
    return g.answer;
}
