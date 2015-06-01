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

#include <opencog/util/oc_assert.h>
#include <opencog/atomspace/Atom.h>

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
protected:
    // properties
    std::string _name;
    void init(const std::string&) throw (InvalidParamException, AssertionException);

    Node(const Node &l) : Atom(0)
    { OC_ASSERT(false, "Node: bad use of copy ctor"); }

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
         TruthValuePtr tv = TruthValue::DEFAULT_TV(),
         AttentionValuePtr av = AttentionValue::DEFAULT_AV())
        : Atom(t,tv,av)
    {
        init(s);
    }

    /**
     * Copy constructor, does not copy atom table membership!
     * Cannot be const, because the get() functions can't be,
     * because thread-safe locking required in the gets.
     */
    Node(Node &n)
        : Atom(n.getType(), n.getTruthValue(), n.getAttentionValue())
    {
        init(n._name);
    }

    /**
     * Gets the name of the node.
     *
     * @return The name of the node.
     */
    inline const std::string& getName() const { return _name; }

    /**
     * Returns a string representation of the node.
     *
     * @return A string representation of the node.
     */
    std::string toString(std::string indent = "");
    std::string toShortString(std::string indent = "");

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

typedef std::shared_ptr<Node> NodePtr;
static inline NodePtr NodeCast(const Handle& h)
    { AtomPtr a(h); return std::dynamic_pointer_cast<Node>(a); }
static inline NodePtr NodeCast(const AtomPtr& a)
    { return std::dynamic_pointer_cast<Node>(a); }

// XXX temporary hack ...
#define createNode std::make_shared<Node>

/** @}*/
} // namespace opencog

#endif // _OPENCOG_NODE_H
