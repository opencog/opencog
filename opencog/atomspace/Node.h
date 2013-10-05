/*
 * opencog/atomspace/Node.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
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

#ifndef _OPENCOG_NODE_H
#define _OPENCOG_NODE_H

#include <opencog/atomspace/Atom.h>
#ifdef ZMQ_EXPERIMENT
#include "ProtocolBufferSerializer.h"
#endif

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/**
 * This is a subclass of Atom. It represents the most basic kind of
 * pattern known to the OpenCog system.
 */
class Node : public Atom
{
    friend class AtomSpaceImpl;  // needs acces to clone()
#ifdef ZMQ_EXPERIMENT
    friend class ProtocolBufferSerializer;
#endif

private:

    // properties
    std::string name;

#ifdef ZMQ_EXPERIMENT
    Node() {};
#endif
    void init(const std::string&) throw (InvalidParamException, AssertionException);

    /** @todo cloning atoms is a fundamental violation oft he architecture. */
    virtual AtomPtr clone() const;
public:

    /**
     * Constructor for this class.
     *
     * @param Node type
     * @param Node name A reference to a std::string with the name of
     *                  the node.  Use empty string for unamed node.
     * @param Node truthvalue A reference to a TruthValue object.
     */
    Node(Type t, const std::string& s,
         const TruthValue& tv = TruthValue::NULL_TV(),
         const AttentionValue& av = AttentionValue::DEFAULT_AV())
        : Atom(t,tv,av) {
        init(s);
    }

    /** Copy constructor, does not copy atom table membership! */
    Node(const Node &n) 
        : Atom(n.getType(),n.getTruthValue(),n.getAttentionValue()) {
        init(n.name);
    }

    /**
     * Gets the name of the node.
     *
     * @return The name of the node.
     */
    const std::string& getName() const;

    /**
     * Returns a string representation of the node.
     *
     * @return A string representation of the node.
     */
    std::string toString() const;
    std::string toShortString() const;

    /**
     * Returns whether a given atom is equal to the current node.
     * @param Atom to be tested.
     * @return true if they are equal, false otherwise.
     */
    virtual bool operator==(const Atom&) const;

    /**
     * Returns whether a given atom is different from the current node.
     * @param Atom to be tested.
     * @return true if they are different, false otherwise.
     */
    virtual bool operator!=(const Atom&) const;
};

// XXX temporary hack ... 
#define createNode std::make_shared<Node>

/** @}*/
} // namespace opencog

#endif // _OPENCOG_NODE_H
