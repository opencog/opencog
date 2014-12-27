/*
 * opencog/visualization/ubigraph/Ubigrapher.cc
 *
 * Copyright (C) 2008-2009 by OpenCog Foundation
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
#include <iomanip>
#include <string>
#include <ctype.h>
#include <limits.h>
#include <unistd.h>

#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/server/CogServer.h>
#include <opencog/util/foreach.h>
#include <opencog/util/Logger.h>

using namespace std;

extern "C" {
    #include "UbigraphAPI.h"
    #include <xmlrpc.h>
}
#include "Ubigrapher.h"

extern const char* ubigraph_url;

namespace opencog
{

std::string initials(std::string s)
{
    std::string ret;
    foreach (char c,  s) {
        if (toupper(c) == c) {
            ret += c;
        }
    }
    return ret;
}

Ubigrapher::Ubigrapher()
    : pushDelay(1), connected(false), space(CogServer::getAtomSpace()),
      withIncoming(false), compact(false)
{
    serverIP = "";
    serverPort = 0;

    compactLabels = true;
    labelsOn = true;

}

void Ubigrapher::init(std::string server, int port)
{
    std::ostringstream os;
    serverIP = server;
    serverPort = port;
    connected = false;
    listening = false;
    os << "http://" << serverIP << ":" << serverPort << "/RPC2";
    serverString = os.str();
    logger().info("Ubigrapher will connect to " + serverString);
    ubigraph_url = serverString.c_str();

    if (ubigraph_clear() == UBIGRAPH_SUCCESS) {
        connected = true;
        setStyles();
    }
}

void Ubigrapher::watchSignals()
{
    if (isConnected()) {
        if (!listening) {
            c_add = space.addAtomSignal(
                    boost::bind(&Ubigrapher::handleAddSignal, this, _1));
            c_remove = space.removeAtomSignal(
                    boost::bind(&Ubigrapher::atomRemoveSignal, this, _1));
            assert(c_add.connected() && c_remove.connected());
            listening = true;
        } else {
            logger().error("[Ubigrapher] Couldn't watch signals, already watching!");
        }
    } else {
        logger().error("[Ubigrapher] Not connected, so won't watch signals!");
    }
}

void Ubigrapher::unwatchSignals()
{
    if (listening) {
        c_add.disconnect();
        c_remove.disconnect();
        assert(!(c_add.connected() || c_remove.connected()));
        listening = false;
    } else {
        logger().error("[Ubigrapher] Couldn't unwatch signals, none connected!");
    }
}

void Ubigrapher::setStyles()
{
    if (!isConnected()) return;
    //cout << "Ubigrapher setStyles" << endl;
    // Set the styles for various types of edges and vertexes in the graph
    nodeStyle = ubigraph_new_vertex_style(0);
    ubigraph_set_vertex_style_attribute(nodeStyle, "shape", "sphere");
    
    linkStyle = ubigraph_new_vertex_style(0);
    ubigraph_set_vertex_style_attribute(linkStyle, "shape", "octahedron");
    ubigraph_set_vertex_style_attribute(linkStyle, "color", "#ff0000");

    outgoingStyle = ubigraph_new_edge_style(0);
    compactLinkStyle = ubigraph_new_edge_style(outgoingStyle);

    outgoingStyleDirected = ubigraph_new_edge_style(outgoingStyle);
    ubigraph_set_edge_style_attribute(outgoingStyleDirected, "arrow", "true");
    // Makes it easier to see the direction of the arrows (cones),
    // but hides the number/type labels
    //ubigraph_set_edge_style_attribute(outgoingStyle, "arrow_radius", "1.5");
    ubigraph_set_edge_style_attribute(outgoingStyleDirected, "arrow_length",
            "2.0");

    compactLinkStyleDirected = ubigraph_new_edge_style(compactLinkStyle);
    ubigraph_set_edge_style_attribute(compactLinkStyleDirected, "arrow",
            "true");
    ubigraph_set_edge_style_attribute(compactLinkStyleDirected, "arrow_length",
            "2.0");
    
}

bool Ubigrapher::handleAddSignal(Handle h)
{
    // XXX This is an error waiting to happen. Signals handling adds must be
    // thread safe as they are called from the AtomSpace event loop
    if (!isConnected()) return false;
    usleep(pushDelay);
    if (classserver().isA(h->getType(),NODE))
        return addVertex(h);
    else {
        if (compact) {
            // don't make nodes for binary links with no incoming
            LinkPtr l(LinkCast(h));
            if (l and l->getOutgoingSet().size() == 2 and
                     space.getIncoming(h).size() == 0)
                return addEdges(h);
        }
        return (addVertex(h) || addEdges(h));
    }
}

bool Ubigrapher::atomRemoveSignal(AtomPtr a)
{
    // XXX This is an error waiting to happen. Signals handling adds must be
    // thread safe as they are called from the AtomSpace event loop
    if (!isConnected()) return false;
    usleep(pushDelay);
    Handle h = a->getHandle();
    if (classserver().isA(a->getType(),NODE))
        return removeVertex(h);
    else {
        if (compact) {
            // don't make nodes for binary links with no incoming
            LinkPtr l(LinkCast(a));
            if (l and l->getOutgoingSet().size() == 2 and
                     space.getIncoming(h).size() == 0)
                return removeEdges(h);
        }
        return removeVertex(h);
    }
}

void Ubigrapher::updateSizeOfHandle(Handle h, property_t p, float multiplier, float baseline)
{
    if (!isConnected()) return;
    float scaler = 0.0f;
    std::ostringstream ost;
    switch (p) {
    case NONE:
        break;
    case TV_STRENGTH:
        scaler = space.getMean(h) * multiplier;
        break;
    case STI:
        scaler = space.getNormalisedZeroToOneSTI(h,false,true)
            * multiplier;
    }
    ost << baseline + scaler;
    LinkPtr l(LinkCast(h));
    if (l) {
        const std::vector<Handle> &out = l->getOutgoingSet();
        if (compact and out.size() == 2 and space.getIncoming(h).size() == 0) {
            ubigraph_set_edge_attribute(h.value(), "width", ost.str().c_str());
        } else
            ubigraph_set_vertex_attribute(h.value(), "size", ost.str().c_str());
    } else {
        ubigraph_set_vertex_attribute(h.value(), "size", ost.str().c_str());
    }

}

void Ubigrapher::updateSizeOfType(Type t, property_t p, float multiplier, float baseline)
{
    if (!isConnected()) return;
    HandleSeq hs;
    std::back_insert_iterator< HandleSeq > out_hi(hs);

    // Get all atoms (and subtypes) of type t
    space.getHandlesByType(out_hi, t, true);
    // For each, get prop, scale... and 
    foreach (Handle h, hs) {
        updateSizeOfHandle(h, p, multiplier, baseline);
    }

}

void Ubigrapher::updateColourOfHandle(Handle h, property_t p, unsigned char startRGB[3],
        unsigned char endRGB[3], float hard)
{
    if (!isConnected()) return;
    unsigned char val[3];
    float scaler = 0.0f;
    int j;
    std::ostringstream ost;
    unsigned char diff[3];
    float multiplierForTV = 10.0f;

    // Find the range that colour changes over for each component
    for (j = 0; j < 3; j++)
        diff[j] = endRGB[j] - startRGB[j];

    ost << "#";
    switch (p) {
    case NONE:
        scaler = 1.0f;
        break;
    case TV_STRENGTH:
        scaler = space.getMean(h);
        break;
    case STI:
        scaler = space.getNormalisedZeroToOneSTI(h,false,true);
    }
    if (hard == 0.0f) {
        if (p == TV_STRENGTH) scaler *= multiplierForTV;
        for (j = 0; j < 3; j++) val[j] = startRGB[j];
        if (scaler > 1.0f) scaler = 1.0f;
        for (j=0; j < 3; j++) {
            val[j] += (unsigned char) (scaler * diff[j]);
            ost << hex << setfill('0') << setw(2) << int(val[j]);
        }
    } else {
        if (scaler < hard) {
            for (j=0; j < 3; j++)
                ost << hex << setfill('0') << setw(2) << int(startRGB[j]);
        } else {
            for (j=0; j < 3; j++)
                ost << hex << setfill('0') << setw(2) << int(endRGB[j]);
        }
    }

    LinkPtr l(LinkCast(h));
    if (l) {
        const std::vector<Handle> &out = l->getOutgoingSet();
        if (compact && out.size() == 2 and space.getIncoming(h).size() == 0) {
            //ubigraph_set_edge_attribute(h.value(), "color", "#ffffff");
            ubigraph_set_edge_attribute(h.value(), "color", ost.str().c_str());
            //ubigraph_set_edge_attribute(h.value(), "width", "2");
            ubigraph_set_edge_attribute(h.value(), "stroke", "solid");
        } else
            ubigraph_set_vertex_attribute(h.value(), "color", ost.str().c_str());
    } else {
        ubigraph_set_vertex_attribute(h.value(), "color", ost.str().c_str());
    }
}

void Ubigrapher::updateColourOfType(Type t, property_t p, unsigned char startRGB[3],
        unsigned char endRGB[3], float hard)
{
    if (!isConnected()) return;
    // Ubigraph doesn't display color properly when set for individual
    // links. Instead, we have to use a style for each base color and change the
    // brightness.
    HandleSeq hs;
    std::back_insert_iterator< HandleSeq > out_hi(hs);
    
    // Get all atoms (and subtypes) of type t
    space.getHandlesByType(out_hi, t, true);
    // For each, get prop, scale... and 
    foreach (Handle h, hs) {
        updateColourOfHandle(h, p, startRGB, endRGB, hard);
    }

}

void Ubigrapher::applyStyleToType(Type t, int style)
{
    if (!isConnected()) return;
    HandleSeq hs;
    std::back_insert_iterator< HandleSeq > out_hi(hs);
    // Get all atoms (and subtypes) of type t
    space.getHandlesByType(out_hi, t, true);
    applyStyleToHandleSeq(hs, style);
}

void Ubigrapher::applyStyleToTypeGreaterThan(Type t, int style, property_t p, float limit)
{
    if (!isConnected()) return;
    HandleSeq hs;
    std::back_insert_iterator< HandleSeq > out_hi(hs);

    // Get all atoms (and subtypes) of type t
    space.getHandlesByType(out_hi, t, true);
    // For each, get prop, scale... and 
    foreach (Handle h, hs) {
        bool okToApply = true;
        switch (p) {
        case NONE:
            break;
        case TV_STRENGTH:
            if (space.getMean(h) < limit) okToApply = false;
            break;
        case STI:
            if (space.getNormalisedZeroToOneSTI(h,false,true) < limit) okToApply = false;
        }
        if (okToApply) {
            LinkPtr l(LinkCast(h));
            if (l) {
                const std::vector<Handle> &out = l->getOutgoingSet();
                if (compact && out.size() == 2 and space.getIncoming(h).size() == 0) {
                    ubigraph_change_edge_style(h.value(), style);
                } else
                    ubigraph_change_vertex_style(h.value(), style);
            } else 
                ubigraph_change_vertex_style(h.value(), style);
        }
    }
}

void Ubigrapher::applyStyleToHandleSeq(HandleSeq hs, int style)
{
    if (!isConnected()) return;
    // For each, get prop, scale... and 
    foreach (Handle h, hs) {
        LinkPtr l(LinkCast(h));
        if (l) {
            const std::vector<Handle> &out = l->getOutgoingSet();
            if (compact and out.size() == 2 and space.getIncoming(h).size() == 0) {
                ubigraph_change_edge_style(h.value(), style);
            } else
                ubigraph_change_vertex_style(h.value(), style);
        } else 
            ubigraph_change_vertex_style(h.value(), style);
    }
}
bool Ubigrapher::addVertex(Handle h)
{
	// Policy: don't display PLN's FWVariableNodes, because it's annoying
	// if (classserver().isA(space.getType(h), FW_VARIABLE_NODE)) return false;

    if (!isConnected()) return false;
    bool isNode = classserver().isA(h->getType(),NODE);

    int id = (int) h.value();

    if (isNode) {
        int status = ubigraph_new_vertex_w_id(id);
        if (status)
            logger().error("Status was %d", status);
        ubigraph_change_vertex_style(id, nodeStyle);
    } else {
        LinkPtr l(LinkCast(h));
        if (l and compact and l->getOutgoingSet().size() == 2 and space.getIncoming(h).size() == 0)
            return false;
        int status = ubigraph_new_vertex_w_id(id);
        if (status)
            logger().error("Status was %d", status);
        ubigraph_change_vertex_style(id, linkStyle);
    }

    if (labelsOn) {
        std::ostringstream ost;
        std::string type = classserver().getTypeName(h->getType());
        if (compactLabels) {
            ost << initials(type);
        } else {
            ost << type;
        }
        
        if (isNode) {
            NodePtr n(NodeCast(h));
            ost << " " << n->getName();
        } /*else {
            LinkPtr l(LinkCast(a));
            l = l; // TODO: anything to output for links?
        }*/
        ost << ":" << space.getMean(h);
        ubigraph_set_vertex_attribute(id, "label", ost.str().c_str());
    }
    return false;
}

/**
 * Outputs ubigraph links for an atom's outgoing connections.
 */
bool Ubigrapher::addEdges(Handle h)
{
    if (!isConnected()) return false;

    usleep(pushDelay);
    LinkPtr l(LinkCast(h));
    if (l)
    {
        const std::vector<Handle> &out = l->getOutgoingSet();
        
//      int id = ;// make IDs based on the type and outgoing set, in case
//      // it's later necessary to change this edge
//      int status = ubigraph_new_edge_w_id(id,x,y);

        if (compact and out.size() == 2 and space.getIncoming(h).size() == 0)
        {
            int id = h.value();
            int status = ubigraph_new_edge_w_id(id, out[0].value(),out[1].value());
            if (status)
                logger().error("Status was %d", status);
            
            int style = compactLinkStyle;
            if (classserver().isA(h->getType(), ORDERED_LINK))
                style = compactLinkStyleDirected;
            ubigraph_change_edge_style(id, style);
            if (labelsOn) {
                std::string type = classserver().getTypeName(h->getType());
                std::ostringstream ost;
                if (compactLabels) {
                    ost << initials(type);
                } else {
                    ost << type;
                }
                ost << ":" << space.getMean(h);
                ubigraph_set_edge_attribute(id, "label", ost.str().c_str());
            }
            return false;
        } else {
            int style = outgoingStyle;
            if (classserver().isA(h->getType(), ORDERED_LINK))
                style = outgoingStyleDirected;
            for (size_t i = 0; i < out.size(); i++) {
                int id = ubigraph_new_edge(h.value(), out[i].value());
                ubigraph_change_edge_style(id, style);
                //ubigraph_set_edge_attribute(id, "label", toString(i).c_str());
            }
        }
    }

    return false;
}

/**
 * Removes the ubigraph node for an atom.
 */
bool Ubigrapher::removeVertex(Handle h)
{
    if (!isConnected()) return false;

    if (compact)
    {
        // Won't have made a node for a binary link with no incoming
        LinkPtr l(LinkCast(h));
        if (l and l->getOutgoingSet().size() == 2 
              and space.getIncoming(h).size() == 0)
            return false;
    }

    int id = (int) h.value();
    int status = ubigraph_remove_vertex(id);
    if (status)
        logger().error("Status was %d", status);

    return false;
}

bool Ubigrapher::removeEdges(Handle h)
{
    if (!isConnected()) return false;

    // This method is only relevant to binary Links with no incoming.
    // Any other atoms will be represented by vertexes, and the edges
    // to them will be automatically deleted by ubigraph when the
    // vertexes are deleted.
    if (compact)
    {
        LinkPtr l(LinkCast(h));
        if (l and l->getOutgoingSet().size() == 2 
              and space.getIncoming(h).size() == 0)
        {                     
            int id = h.value();
            int status = ubigraph_remove_edge(id);
            if (status)
                logger().error("Status was %d", status);
        }
    }
    return false;
}

void Ubigrapher::graph()
{
    if (!isConnected()) return;
    ubigraph_clear();
    setStyles();
    space.foreach_handle_of_type((Type)ATOM, &Ubigrapher::addVertex, this, true);
    space.foreach_handle_of_type((Type)ATOM, &Ubigrapher::addEdges, this, true);
}


} // namespace opencog
