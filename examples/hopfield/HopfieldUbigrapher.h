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

#include <opencog/visualization/ubigraph/Ubigrapher.h>

#include "Pattern.h"

#ifndef _OPENCOG_HOPFIELD_UBIGRAPHER
#define _OPENCOG_HOPFIELD_UBIGRAPHER

using namespace std;
 
namespace opencog
{


class HopfieldUbigrapher : public Ubigrapher
{
    //! Ubigraph ids for label vertex and edge
    int labelVertex, labelEdge;
    //! For linking general text to
    Handle groundNode; 

protected:
    void setStyles();
public:

    HopfieldUbigrapher();
//    ~HopfieldUbigrapher();

    //! Whether to show the labels ground node.
    //! (Also need to setGroundNode() )
    bool showText;

    //! Display a label that isn't attached to any of the
    //! Hopfield network nodes, but is instead attached to
    //! an invisible Vertex and edge that attach to the
    //! first grid node.
    void setText(string s);

    //! Change node style to a key node
    void setAsKeyNode(Handle kn);
    void setAsActiveKeyNode(Handle kn);
    //! Change node style to a pattern node 
    void setAsPatternNode(Handle kn);
    //! Change link style to a freshly added random link
    //! (this isn't the default link style because HebbianUpdatingAgent
    //! sometimes swaps between link types to/from Inverse Hebbian Link)
    void setAsNewRandomLink(Handle kn);
    //! Set which node is the ground node for linking the text Vertex to.
    void setGroundNode(Handle h) ;

    //! Display difference between encoded
    //! \arg show agreement in green
    //! \arg show missing in blue
    //! \arg show extra in red
    void showDiff(HandleSeq h, Pattern cur, Pattern original);

    //! Ubigraph styles
    int patternStyle, patternAddErrStyle, patternMissErrStyle, notPatternStyle;
    int keyNodeActiveStyle, keyNodeStyle;
    int randomLinkStyle;

};

} // namespace opencog

#endif // _OPENCOG_HOPFIELD_UBIGRAPHER
