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

/** Ubigrapher - draws Atoms to the Ubigraph visualisation server.
 *
 * \url http://http://ubietylab.net/ubigraph/
 *
 * @warning Ubigrapher can listen for signals, but if multiple threads send
 * signals, then this could confuse the server/connection, and this file should
 * be modified to check that another signal isn't already being processed.
 */
class Ubigrapher
{
private:
    /// Store when last update was
    struct timeval lastPush;
    /// Limit requests to occur only once per push delay
    long pushDelay;

    /**
     * Outputs a ubigraph node for an atom.
     */
    bool addVertex(Handle h);
    /**
     * Outputs ubigraph links for an atom's outgoing connections.
     */
    bool addEdges(Handle h);
    /**
     * Removes the ubigraph node for an atom.
     */
    bool removeVertex(Handle h);
    /**
     * Removes the ubigraph link for an atom.
     */
    bool removeEdges(Handle h);

    bool handleAddSignal(Handle h); //! Signal handler for atom adds.
    bool handleRemoveSignal(Handle h); //! Signal handler for atom removals.

    bool listening; //! Whether the Ubigrapher is listening for AtomSpace signals.
    boost::signals::connection c_add; //! Connection to add atom signals
    boost::signals::connection c_remove; //! Connection to remove atom signals
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

    //! Set the various default styles
    void setStyles();

    void watchSignals();
    void unwatchSignals();

    void graph();
};

} // namespace opencog
