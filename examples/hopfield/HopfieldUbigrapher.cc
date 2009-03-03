/*
 * examples/hopfield/HopfieldUbigrapher.h
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

extern "C" {
    #include <UbigraphAPI.h>
}

#include <opencog/atomspace/Handle.h>

#include "HopfieldUbigrapher.h"

namespace opencog
{

HopfieldUbigrapher::HopfieldUbigrapher() : Ubigrapher()
{
    patternStyle = patternErrStyle = notPatternStyle =
        keyNodeStyle = activeKeyNodeStyle = newEdgeStyle = 0;
    compact = true;
    labelsOn = false;
    setStyles();
    watchSignals();
}

void HopfieldUbigrapher::setAsKeyNode(Handle kn)
{ ubigraph_change_vertex_style(kn.value(), keyNodeStyle); }

void HopfieldUbigrapher::setAsNewRandomLink(Handle kn)
{ ubigraph_change_edge_style(kn.value(), newEdgeStyle); }

void HopfieldUbigrapher::setText(string s)
{
    static int labelVertex = 0;

    if (labelVertex == 0) {
        int pseudoEdge;
        if (groundNode == Handle::UNDEFINED) return;
        labelVertex = ubigraph_new_vertex();
//        ubigraph_set_vertex_attribute(labelVertex, "visible", "false");
        pseudoEdge = ubigraph_new_edge(labelVertex, groundNode.value());
        ubigraph_set_edge_attribute(pseudoEdge, "strength", "0.01");
        ubigraph_set_edge_attribute(pseudoEdge, "visible", "false");
        ubigraph_set_edge_attribute(pseudoEdge, "oriented", "true");
    }
    ubigraph_set_vertex_attribute(labelVertex, "label", s.c_str());
}

void HopfieldUbigrapher::setStyles()
{
    //cout << "H Ubigrapher setStyles" << endl;
    //Ubigrapher::setStyles();

    ubigraph_set_edge_style_attribute(0, "strength", "0.005");
    newEdgeStyle = ubigraph_new_edge_style(0);
    ubigraph_set_edge_style_attribute(newEdgeStyle, "stroke", "dashed");

    patternStyle = ubigraph_new_vertex_style(nodeStyle);
    patternErrStyle = ubigraph_new_vertex_style(nodeStyle);
    notPatternStyle = ubigraph_new_vertex_style(nodeStyle);

    nodeStyle = notPatternStyle;

    keyNodeStyle = ubigraph_new_vertex_style(notPatternStyle);
    activeKeyNodeStyle = ubigraph_new_vertex_style(patternStyle);

    // pattern style: green spheres
    ubigraph_set_vertex_style_attribute(patternStyle, "color", "#80ff32");

    // pattern error style: red spheres
    ubigraph_set_vertex_style_attribute(patternErrStyle, "color", "#ff8032");

    // pattern error style: grey spheres
    ubigraph_set_vertex_style_attribute(notPatternStyle, "color", "#606060");

    // key node style: octahedron
    ubigraph_set_vertex_style_attribute(keyNodeStyle, "shape", "octahedron");

    // active key node style: octahedron
    ubigraph_set_vertex_style_attribute(activeKeyNodeStyle, "shape", "octahedron");
}


} // namespace opencog

