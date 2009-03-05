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
    patternStyle = patternAddErrStyle = patternMissErrStyle = notPatternStyle =
        keyNodeStyle = activeKeyNodeStyle = randomLinkStyle = 0;
    compact = true;
    labelsOn = false;
    setStyles();
    watchSignals();
}

void HopfieldUbigrapher::setAsKeyNode(Handle kn)
{ ubigraph_change_vertex_style(kn.value(), keyNodeStyle); }

void HopfieldUbigrapher::setAsNewRandomLink(Handle kn)
{ ubigraph_change_edge_style(kn.value(), randomLinkStyle); }

void HopfieldUbigrapher::setText(string s)
{
#if 0
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
#endif
}

void HopfieldUbigrapher::setStyles()
{
    //cout << "H Ubigrapher setStyles" << endl;
    //Ubigrapher::setStyles();

    // When first added, links are red and have low strength
    ubigraph_set_edge_style_attribute(compactLinkStyle, "strength", "0.005");
    ubigraph_set_edge_style_attribute(compactLinkStyle, "color", "#ff5530");

    // Random links are dashed and gray
    randomLinkStyle = ubigraph_new_edge_style(compactLinkStyle);
    ubigraph_set_edge_style_attribute(randomLinkStyle, "stroke", "dashed");
    ubigraph_set_edge_style_attribute(randomLinkStyle, "color", "#aaaaaa");

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

    // key node style: octahedron
    keyNodeStyle = ubigraph_new_vertex_style(notPatternStyle);
    ubigraph_set_vertex_style_attribute(keyNodeStyle, "shape", "octahedron");

    // active key node style: octahedron
    activeKeyNodeStyle = ubigraph_new_vertex_style(patternStyle);
    ubigraph_set_vertex_style_attribute(activeKeyNodeStyle, "shape", "octahedron");
}

void HopfieldUbigrapher::showDiff(HandleSeq hs, Pattern current, Pattern original)
{
    for (uint i = 0; i < hs.size(); i++) {
        string vColor = "#606060";
        if (current[i] && original[i]) {
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

