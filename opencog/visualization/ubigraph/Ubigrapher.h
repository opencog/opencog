/*
 * opencog/visualization/ubigraph/Ubigrapher.h
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

#include <time.h>

#include <opencog/atomspace/AtomSpace.h>

#include "UbigraphAPI.h"

#ifndef _OPENCOG_UBIGRAPHER
#define _OPENCOG_UBIGRAPHER

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
    bool addVertex(const Handle& h);
    /**
     * Outputs ubigraph links for an atom's outgoing connections.
     */
    bool addEdges(const Handle& h);
    /**
     * Removes the ubigraph node for an atom.
     */
    bool removeVertex(Handle h);
    /**
     * Removes the ubigraph link for an atom.
     */
    bool removeEdges(Handle h);

    bool handleAddSignal(Handle); //! Signal handler for atom adds.
    bool atomRemoveSignal(AtomPtr); //! Signal handler for atom removals.

    bool listening; //! Whether the Ubigrapher is listening for AtomSpace signals.
    boost::signals2::connection c_add; //! Connection to add atom signals
    boost::signals2::connection c_remove; //! Connection to remove atom signals

    /** Whether the Ubigrapher has successfully connected to an Ubigraph
     * server.
     */
    bool connected;

    std::string serverString; //!< Complete XMLRPC server string

protected:
    //! Initialise the various styles
    virtual void setStyles();
    
public:
    Ubigrapher();
    virtual ~Ubigrapher() {}
    void init(std::string server = "localhost", int port = UBIGRAPH_DEFAULT_PORT);

    std::string serverIP;
    int serverPort;

    bool isConnected() { return connected; };
    std::string getServerString() { return serverString; };

    AtomSpace& space;

    bool withIncoming;
    //! Whether to squash arity 2 links with no incoming set into a single edge.
    bool compact;
    //! Whether nodes/links are given labels at all
    bool labelsOn;
    //! Makes much more compact labels. Currently uses the initials of the typename.
    bool compactLabels;

    //! Styles corresponding to different types of Atom,
    //! except outgoingStyle which is for the edges that collectively represent outgoing sets
    int nodeStyle, linkStyle,
        compactLinkStyle, compactLinkStyleDirected,
        outgoingStyle, outgoingStyleDirected;

    // Hack styles to account for Ubigraph color attribute bug
    //int redStyle, blueStyle, greenStyle;
    //! property type for controlling how styles and attributes are applied.
    typedef enum { NONE = 0, TV_STRENGTH, STI } property_t;

    //! Update the colors of Handle h based on property p
    void updateColourOfHandle(Handle h, property_t p, unsigned char startRGB[3],
            unsigned char endRGB[3], float hard = 0.0f);
    //! Update the colors of atoms with type t, based on property p
    void updateColourOfType(Type t, property_t p, unsigned char startRGB[3],
            unsigned char endRGB[3], float hard = 0.0f);
    //! Update the size of all atoms of type t, based on prop p
    void updateSizeOfType(Type t, property_t p, float multiplier = 1.0f, float baseline=0.5f);
    //! Update the size of Handle based on prop p
    void updateSizeOfHandle(Handle hs, property_t p, float multiplier = 1.0f, float baseline=0.5f);
    //! Apply a given style to all atoms of type t, if normalised p is greater
    //! than limit.
    void applyStyleToTypeGreaterThan(Type t, int style, property_t p, float limit = 0.0f);
    //! Apply a given style to all atoms of type t.
    void applyStyleToType(Type t, int style);

    void applyStyleToHandleSeq(HandleSeq hs, int style);

    //! Start watching the AtomSpace add/remove atom signals.
    void watchSignals();
    //! Stop watching the AtomSpace add/remove atom signals.
    void unwatchSignals();

    //! Redraw the ubigraph, based on the current AtomSpace.
    void graph();
};

} // namespace opencog

#endif // _OPENCOG_UBIGRAPHER
