/*
 * opencog/ubigraph/UbigraphModule.cc
 *
 * Copyright (C) 2009 by Singularity Institute for Artificial Intelligence
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

#include "UbigraphModule.h"
#include <opencog/util/Logger.h>
#include <opencog/atomspace/utils.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/server/CogServer.h>

extern "C" {
    #include <UbigraphAPI.h>
}

using namespace opencog;

DECLARE_MODULE(UbigraphModule);

std::string initials(std::string s)
{
    std::string ret;
    foreach (char c,  s) {
        if (/*isUpperCase(c)*/ Isox(c) == c) {
            ret += c;
        }
    }
    return ret;
}

class Ubigrapher
{
public:
    // TODO register handlers for the add/remove atom events
    Ubigrapher(AtomSpace *space) : space(space), withIncoming(false), compact(true)
    {
        compactLabels = true;
        ubigraph_clear();
    }
    AtomSpace *space;
    bool withIncoming;
    bool compact;
    //! Makes much more compact labels. Currently uses the initials of the typename.
    bool compactLabels;

    /**
     * Outputs a ubigraph node for an atom.
     */
    bool do_nodes(Handle h)
    {
        Atom *a = TLB::getAtom(h);

        if (compact)
        {
            // don't make nodes for binary links with no incoming
            Link *l = dynamic_cast<Link*>(a);
            if (l && l->getOutgoingSet().size() == 2 &&
                     l->getIncomingSet() == NULL)
                return false;
        }

//        std::ostringstream ost;
//        ost << h.value() << " [";
        int id = (int)h.value();
        int status = ubigraph_new_vertex_w_id(id);
        if (space->isNode(a->getType()))
            ubigraph_set_vertex_attribute(id, "shape", "sphere");
        else {
            //            ost << "shape=\"diamond\" ";
            ubigraph_set_vertex_attribute(id, "shape", "octahedron");
            ubigraph_set_vertex_attribute(id, "color", "#ff0000");
        }

        std::ostringstream ost;
        //        ost << "label=\"[" << ClassServer::getTypeName(a->getType()) << "]";
        std::string type = ClassServer::getTypeName(a->getType());
        if (compactLabels) {
            ost << initials(type);
        } else {
            ost << type;
        }
        
        if (space->isNode(a->getType())) {
            Node *n = (Node*)a;
            ost << " " << n->getName();
        } else {
            Link *l = (Link*)a;
            l = l; // TODO: anything to output for links?
        }
        ubigraph_set_vertex_attribute(id, "label", ost.str().c_str());
//        ost << "\"];\n";
//        answer += ost.str();*/
        return false;
    }

    /**
     * Outputs ubigraph links for an atom's outgoing connections.
     */
    bool do_links(Handle h)
    {
        Atom *a = TLB::getAtom(h);
//        std::ostringstream ost;

        const Link *l = dynamic_cast<const Link *>(a);
        if (l)
        {
            const std::vector<Handle> &out = l->getOutgoingSet();
            
//            int id = ;// make IDs based on the type and outgoing set, in case
//            // it's later necessary to change this edge
//            int status = ubigraph_new_edge_w_id(id,x,y);

            if (compact && out.size() == 2 && l->getIncomingSet() == NULL)
            {
//                ost << out[0] << " -> " << out[1] << " [label=\""
//                    << ClassServer::getTypeName(a->getType()) << "\"];\n";
//                answer += ost.str();
                int id = ubigraph_new_edge(out[0].value(),out[1].value());
                
                std::string type = ClassServer::getTypeName(a->getType());
                std::ostringstream ost;
                if (compactLabels) {
                    ost << initials(type);
                } else {
                    ost << type;
                }
                ubigraph_set_edge_attribute(id, "label", ost.str().c_str());                
                
                ubigraph_set_edge_attribute(id, "arrow", "true");
                // Makes it easier to see the direction of the arrows (cones),
                // but hides the type labels
//                ubigraph_set_edge_attribute(id, "arrow_radius", "1.5");
                ubigraph_set_edge_attribute(id, "arrow_length", "2.0");
                return false;
            }

            for (size_t i = 0; i < out.size(); i++) {
                int id = ubigraph_new_edge(h.value(),out[i].value());
                ubigraph_set_edge_attribute(id, "label", toString(i).c_str());
                ubigraph_set_edge_attribute(id, "arrow", "true");
                // Makes it easier to see the direction of the arrows (cones),
                // but hides the number labels
//                ubigraph_set_edge_attribute(id, "arrow_radius", "1.5");
                ubigraph_set_edge_attribute(id, "arrow_length", "2.0");

//                ost << h << "->" << out[i] << " [label=\"" << i << "\"];\n";
            }
        }

/*        if (withIncoming) {
            HandleEntry *he = a->getIncomingSet();
            int i = 0;
            while (he) {
//                ost << h << "->" << he->handle << " [style=\"dotted\" label=\"" << i << "\"];\n";
                he = he->next;
                i++;
            }
        }*/

//        answer += ost.str();
        return false;
    }

    void graph()
    {
        space->foreach_handle_of_type((Type)ATOM, &Ubigrapher::do_nodes, this, true);
        space->foreach_handle_of_type((Type)ATOM, &Ubigrapher::do_links, this, true);
    }

};

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
    AtomSpace *space = CogServer::getAtomSpace();
    Ubigrapher g(space);
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

