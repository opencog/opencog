/*
 * src/AtomSpace/Link.h
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

#ifndef LINK_H
#define LINK_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "Atom.h"
#include "Trail.h"
#include "types.h"

#include <string>

namespace opencog
{

/**
 * Nodes in OpenCog are connected to each other by links. Each link embodies
 * one of the basic inter-node relationships. Links do not necessarily
 * describe a binary relationship between two entities. Links may describe
 * relationships between more than two entities at once. Finally, links
 * describe relationships not only between nodes, but also higher-order
 * relationships between links, and between nodes and links.
 */
class Link : public Atom
{
    friend class SavingLoading;
    friend class AtomTable;
    friend class NMXmlParser;
    friend class Atom;

private:
    Trail* trail;
    void init(void) throw (InvalidParamException);

#ifdef PUT_OUTGOING_SET_IN_LINKS
    // Adds a new handle to the outgoing set. Note that this is
    // used only in the NativeParser friend class, and, due to
    // performance issues, it should not be used anywhere else...
    void addOutgoingAtom(Handle h);
#endif /* PUT_OUTGOING_SET_IN_LINKS */

protected:

#ifdef PUT_OUTGOING_SET_IN_LINKS
    // Array that does not change during atom lifespan.
    std::vector<Handle> outgoing;

    /**
      * Sets the outgoing set of the atom
      * This method can be called only if the atom is not inserted
      * in an AtomTable yet.
      * Otherwise, it throws a RuntimeException.
      */
    void setOutgoingSet(const std::vector<Handle>& o)
    throw (RuntimeException);
#endif /* PUT_OUTGOING_SET_IN_LINKS */

public:

    /**
     * Constructor for this class.
     *
     * @param Link type.
     * @param Outgoing set, which is an array of the atom handles
     *        referenced by this link (both sources and targets).
     * @param Link truthvalue, which will be cloned before being
     *        stored in this Link.
     */
    Link(Type, const std::vector<Handle>&,
         const TruthValue& = TruthValue::NULL_TV());

    /**
     * Destructor for this class.
     */
    ~Link() throw ();

#ifdef PUT_OUTGOING_SET_IN_LINKS
    inline Arity getArity() const {
        return outgoing.size();
    }

    /**
     * Returns a const reference to the array containing this
     * atom's outgoing set.
     *
     * @return A const reference to this atom's outgoing set.
     */
    inline const std::vector<Handle>& getOutgoingSet() const {
        return outgoing;
    }
    /**
     * Returns a specific atom in the outgoing set (using the TLB).
     *
     * @param The position of the atom in the array.
     * @return A specific atom in the outgoing set (using the TLB).
     */
    Atom * getOutgoingAtom(int) const throw (RuntimeException);

    /**
     * Builds the target type index structure according to the types of
     * elements in the outgoing set of the given atom.
     *
     * @return A pointer to target types array built.
     * NOTE: The argument size gets the size of the returned array.
     */
    Type* buildTargetIndexTypes(int *size);

    /**
     * Returns the position of a certain type on the reduced array of
     * target types of an atom.
     *
     * @param The type which will be searched in the reduced target types
     * index array.
     * @return The position of the given type in the reduced array of
     * target types.
     */
    int locateTargetIndexTypes(Type) const;

    /**
     * Returns the number of different target types of an atom.
     *
     * @return The number of different target types of an atom.
     */
    int getTargetTypeIndexSize() const;

#endif /* PUT_OUTGOING_SET_IN_LINKS */

    /**
     * Returns the trail of the link.
     *
     * @return Trail of the link.
     */
    Trail* getTrail();

    /**
     * Sets a trail for the link.
     *
     * @param Trail to be set.
     */
    void setTrail(Trail *);

    /**
     * Returns the weight value of the link.
     *
     * @return Weight value of the link.
     */
    float getWeight();

    /**
     * Returns a string representation of the link.
     *
     * @return A string representation of the link.
     */
    std::string toString() const;

    /**
     * Returns a short string representation of the link.
     *
     * @return A short string representation of the link.
     */
    std::string toShortString() const;

    /**
     * Returns whether a given handle is a source of this link.
     *
     * @param Handle to be checked for being a link source.
     * @return Whether a given handle is a source of this link.
     */
    bool isSource(Handle) throw (InvalidParamException);

    /**
     * Returns whether the element in a given position in the
     * outgoing set of this link is a source.
     *
     * @param Position in the outgoing set.
     * @return Whether the element in a given position in the
     *         outgoing set of this link is a source.
     */
    bool isSource(int) throw (IndexErrorException, InvalidParamException);

    /**
     * Returns whether a given handle is a target of this link.
     *
     * @param Handle to be checked for being a link target.
     * @return Whether a given handle is a target of this link.
     */
    bool isTarget(Handle) throw (InvalidParamException);

    /**
     * Returns whether the element in a given position in the
     * outgoing set of this link is a target.
     *
     * @param Position in the outgoing set.
     * @return Whether the element in a given position in the
     *         outgoing set of this link is a target.
     */
    bool isTarget(int) throw (IndexErrorException, InvalidParamException);

    /**
     * Returns whether a given atom is equal to the current link.
     * @param Link to be tested.
     * @return true if they are equal, false otherwise.
     */
    virtual bool equals(const Atom *) const;

    /**
    * Returns the hashCode of the Link.
    * @return a integer value as the hashCode of the Link.
    */
    virtual int hashCode(void) const;
};

}

#endif
