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

#include <opencog/ubigraph/Ubigrapher.h>

#ifndef _OPENCOG_HOPFIELD_UBIGRAPHER
#define _OPENCOG_HOPFIELD_UBIGRAPHER
 
namespace opencog
{


class HopfieldUbigrapher : public Ubigrapher
{
    Handle groundNode; //! For linking general text to
protected:
    void setStyles();
public:

    HopfieldUbigrapher();
//    ~HopfieldUbigrapher();

    void setText(string s);
    void setAsKeyNode(Handle kn);
    void setAsNewRandomLink(Handle kn);
    void setGroundNode(Handle h) {groundNode = h;};

    //! Ubigraph styles
    int patternStyle, patternErrStyle, notPatternStyle,
        keyNodeStyle, activeKeyNodeStyle, newEdgeStyle;

};

} // namespace opencog

#endif // _OPENCOG_HOPFIELD_UBIGRAPHER
