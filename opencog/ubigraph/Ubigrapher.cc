/*
 * opencog/ubigraph/Ubigrapher.cc
 *
 * Copyright (C) 2008-2009 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by Jared Wigmore <jared.wigmore@gmail.com>
 * Adapted from code in DottyModule (which is by Trent Waddington <trent.waddington@gmail.com>)
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

#include <sstream>
#include <string>
#include <tr1/functional>
using namespace std::tr1::placeholders;

#include <opencog/util/Logger.h>
#include <opencog/atomspace/utils.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/server/CogServer.h>

extern "C" {
    #include <UbigraphAPI.h>
}
#include "Ubigrapher.h"

namespace opencog
{

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

Ubigrapher::Ubigrapher() : withIncoming(false), compact(false)
{
    space = CogServer::getAtomSpace();

    compactLabels = true;

    space->addAtomSignal().connect(std::tr1::bind(&Ubigrapher::add_vertex, this, _1));
    space->addAtomSignal().connect(std::tr1::bind(&Ubigrapher::add_edges, this, _1));
    space->removeAtomSignal().connect(std::tr1::bind(&Ubigrapher::remove_vertex, this, _1));
    space->removeAtomSignal().connect(std::tr1::bind(&Ubigrapher::remove_edges, this, _1));

    ubigraph_clear();

    // Set the styles for various types of edges and vertexes in the graph        
    nodeStyle = ubigraph_new_vertex_style(0);
    ubigraph_set_vertex_style_attribute(nodeStyle, "shape", "sphere");
    
    linkStyle = ubigraph_new_vertex_style(0);
    ubigraph_set_vertex_style_attribute(linkStyle, "shape", "octahedron");
    ubigraph_set_vertex_style_attribute(linkStyle, "color", "#ff0000");

    outgoingStyle = ubigraph_new_edge_style(0);
    ubigraph_set_edge_style_attribute(outgoingStyle, "arrow", "true");
    // Makes it easier to see the direction of the arrows (cones),
    // but hides the number/type labels
//      ubigraph_set_edge_style_attribute(outgoingStyle, "arrow_radius", "1.5");
    ubigraph_set_edge_style_attribute(outgoingStyle, "arrow_length", "2.0");
    
    compactLinkStyle = ubigraph_new_edge_style(outgoingStyle);
}

bool Ubigrapher::add_vertex(Handle h)
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

    int id = (int)h.value();
    int status = ubigraph_new_vertex_w_id(id);
    if (space->isNode(a->getType()))
        ubigraph_change_vertex_style(id, nodeStyle);
    else {
        ubigraph_change_vertex_style(id, linkStyle);
    }

    std::ostringstream ost;
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
    return false;
}

/**
 * Outputs ubigraph links for an atom's outgoing connections.
 */
bool Ubigrapher::add_edges(Handle h)
{
    Atom *a = TLB::getAtom(h);

    const Link *l = dynamic_cast<const Link *>(a);
    if (l)
    {
        const std::vector<Handle> &out = l->getOutgoingSet();
        
//            int id = ;// make IDs based on the type and outgoing set, in case
//            // it's later necessary to change this edge
//            int status = ubigraph_new_edge_w_id(id,x,y);

        if (compact && out.size() == 2 && l->getIncomingSet() == NULL)
        {
            int id = h.value();
            int status = ubigraph_new_edge_w_id(id, out[0].value(),out[1].value());
            
            std::string type = ClassServer::getTypeName(a->getType());
            std::ostringstream ost;
            if (compactLabels) {
                ost << initials(type);
            } else {
                ost << type;
            }
            ubigraph_change_edge_style(id, compactLinkStyle);
            ubigraph_set_edge_attribute(id, "label", ost.str().c_str());
            return false;
        }

        for (size_t i = 0; i < out.size(); i++) {
            int id = ubigraph_new_edge(h.value(),out[i].value());
            ubigraph_change_edge_style(id, outgoingStyle);
            ubigraph_set_edge_attribute(id, "label", toString(i).c_str());
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
    return false;
}

/**
 * Removes the ubigraph node for an atom.
 */
bool Ubigrapher::remove_vertex(Handle h)
{
    Atom *a = TLB::getAtom(h);

    if (compact)
    {
        // Won't have made a node for a binary link with no incoming
        Link *l = dynamic_cast<Link*>(a);
        if (l && l->getOutgoingSet().size() == 2 &&
                 l->getIncomingSet() == NULL)
            return false;
    }

    int id = (int)h.value();
    int status = ubigraph_remove_vertex(id);

    return false;
}

bool Ubigrapher::remove_edges(Handle h)
{
    Atom *a = TLB::getAtom(h);

    // This method is only relevant to binary Links with no incoming.
    // Any other atoms will be represented by vertexes, and the edges
    // to them will be automatically deleted by ubigraph when the
    // vertexes are deleted.
    if (compact)
    {
        Link *l = dynamic_cast<Link*>(a);
        if (l && l->getOutgoingSet().size() == 2 &&
                 l->getIncomingSet() == NULL)
        {                     
            int id = h.value();
            int status = ubigraph_remove_edge(id);
        }
    }
    return false;
}

void Ubigrapher::graph()
{
    space->foreach_handle_of_type((Type)ATOM, &Ubigrapher::add_vertex, this, true);
    space->foreach_handle_of_type((Type)ATOM, &Ubigrapher::add_edges, this, true);
}


} // namespace opencog
