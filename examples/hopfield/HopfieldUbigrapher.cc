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
    #include <opencog/visualization/ubigraph/UbigraphAPI.h>
}

#include <opencog/atoms/base/Handle.h>

#include "HopfieldUbigrapher.h"

namespace opencog
{

HopfieldUbigrapher::HopfieldUbigrapher() : Ubigrapher()
{
    // styles for pattern nodes:
    patternStyle = patternAddErrStyle = patternMissErrStyle = notPatternStyle = 0;
    // styles for key nodes:
    keyNodeStyle = keyNodeActiveStyle = 0;
    // styles for randomly added links
    randomLinkStyle = 0;
    labelVertex = 0;

    compact = true;   // collapse links
    labelsOn = false; // don't show labels
    setStyles();
    watchSignals();
}

void HopfieldUbigrapher::setAsPatternNode(Handle kn)
{ ubigraph_change_vertex_style(kn.value(), notPatternStyle); }

void HopfieldUbigrapher::setAsKeyNode(Handle kn)
{ ubigraph_change_vertex_style(kn.value(), keyNodeStyle); }

void HopfieldUbigrapher::setAsActiveKeyNode(Handle kn)
{ ubigraph_change_vertex_style(kn.value(), keyNodeActiveStyle); }

void HopfieldUbigrapher::setAsNewRandomLink(Handle kn)
{ ubigraph_change_edge_style(kn.value(), randomLinkStyle); }

void HopfieldUbigrapher::setGroundNode(Handle h)
{
    if (h == Handle::UNDEFINED) return;
    groundNode = h;
    if (labelVertex) {
        // remove existing edge 
        ubigraph_remove_edge(labelEdge);
    } else {
        labelVertex = ubigraph_new_vertex();
        ubigraph_set_vertex_attribute(labelVertex, "size", "1");
    }
    
    labelEdge = ubigraph_new_edge(labelVertex, groundNode.value());
    ubigraph_set_edge_attribute(labelEdge, "strength", "0.01");
    ubigraph_set_edge_attribute(labelEdge, "visible", "false");
    ubigraph_set_edge_attribute(labelEdge, "oriented", "true");

}

void HopfieldUbigrapher::setText(string s)
{
    if (showText && labelVertex)
        ubigraph_set_vertex_attribute(labelVertex, "label", s.c_str());
}

void HopfieldUbigrapher::setStyles()
{
    //cout << "H Ubigrapher setStyles" << endl;
    //Ubigrapher::setStyles();

    // When first added, links are red and have low strength
    ubigraph_set_edge_style_attribute(compactLinkStyle, "strength", "0.005");
    ubigraph_set_edge_style_attribute(compactLinkStyle, "color", "#ff5530");
    // When first added, links are red and have low strength
    //ubigraph_set_edge_style_attribute(compactLinkStyleDirected, "strength", "0.005");
    //ubigraph_set_edge_style_attribute(compactLinkStyleDirected, "color", "#ff5530");

    // Random links are dashed and gray
    randomLinkStyle = ubigraph_new_edge_style(compactLinkStyle);
    ubigraph_set_edge_style_attribute(randomLinkStyle, "stroke", "dashed");
    ubigraph_set_edge_style_attribute(randomLinkStyle, "color", "#aaaaaa");

    // PATTERN node styles
    // normal node style: grey spheres
    notPatternStyle = ubigraph_new_vertex_style(nodeStyle);
    ubigraph_set_vertex_style_attribute(notPatternStyle, "color", "#606060");
    nodeStyle = notPatternStyle;

    // pattern node style: green spheres
    patternStyle = ubigraph_new_vertex_style(notPatternStyle);
    ubigraph_set_vertex_style_attribute(patternStyle, "color", "#80ff32");

    // pattern positive error style: red spheres
    patternAddErrStyle = ubigraph_new_vertex_style(notPatternStyle);
    ubigraph_set_vertex_style_attribute(patternAddErrStyle, "color", "#ff8032");

    // pattern negative error style: blue spheres
    patternMissErrStyle = ubigraph_new_vertex_style(notPatternStyle);
    ubigraph_set_vertex_style_attribute(patternMissErrStyle, "color", "#3280ff");

    // KEY node styles
    // key node style: octahedron
    keyNodeStyle = ubigraph_new_vertex_style(notPatternStyle);
    ubigraph_set_vertex_style_attribute(keyNodeStyle, "shape", "octahedron");

    // active key node style: octahedron
    keyNodeActiveStyle = ubigraph_new_vertex_style(patternStyle);
    ubigraph_set_vertex_style_attribute(keyNodeActiveStyle, "shape", "octahedron");
    ubigraph_set_vertex_style_attribute(keyNodeActiveStyle, "color", "#63f1ba");

}

void HopfieldUbigrapher::showDiff(HandleSeq hs, Pattern current, Pattern original)
{
    // So hs is the grid, current is the binarised STIs (including keyNodes and
    // with the appropriate mask), original is the original pattern.
    for (uint i = 0; i < hs.size(); i++) {
        string vColor = "#606060";
        if (current.isMasked(i)) {
            if (current[i]) vColor = "#63f1ba";
        } else if (current[i] && original[i]) {
            vColor = "#80ff32";
        } else if (current[i]) {
            vColor = "#ff8032";
        } else if (original[i]) {
            vColor = "#3280ff";
        }
        ubigraph_set_vertex_attribute(hs[i].value(), "color", vColor.c_str());

    }


}

} // namespace opencog

