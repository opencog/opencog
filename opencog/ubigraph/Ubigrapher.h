/*
 * opencog/ubigraph/Ubigrapher.h
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

#include <time.h>

#include <opencog/atomspace/AtomSpace.h>

namespace opencog
{

class Ubigrapher
{
private:
    /// Store when last update was
    struct timeval lastPush;
    /// Limit requests to occur only once per push delay
    long pushDelay;

public:
    Ubigrapher();

    AtomSpace *space;
    bool withIncoming;
    bool compact;
    //! Makes much more compact labels. Currently uses the initials of the typename.
    bool compactLabels;

    //! Styles corresponding to different types of Atom,
    //! except outgoingStyle which is for the edges that collectively represent outgoing sets
    int nodeStyle, linkStyle, compactLinkStyle, outgoingStyle;

    /**
     * Outputs a ubigraph node for an atom.
     */
    bool add_vertex(Handle h);
    /**
     * Outputs ubigraph links for an atom's outgoing connections.
     */
    bool add_edges(Handle h);
    /**
     * Removes the ubigraph node for an atom.
     */
    bool remove_vertex(Handle h);
    /**
     * Removes the ubigraph link for an atom.
     */
    bool remove_edges(Handle h);

    void graph();
};

} // namespace opencog
