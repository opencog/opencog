/*
 * opencog/atomspace/Link.h
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

#ifndef _OPENCOG_LINK_H
#define _OPENCOG_LINK_H

#include <string>

#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/atomspace/Trail.h>
#include <opencog/atomspace/types.h>

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
    // This is dodgy, but it needs access to getOutgoingAtom
    friend class HandleEntry;

private:
    Trail* trail;
    void init(const std::vector<Handle>&) throw (InvalidParamException);

    // Adds a new handle to the outgoing set. Note that this is
    // used only in the NativeParser friend class, and, due to
    // performance issues, it should not be used anywhere else...
    void addOutgoingAtom(Handle h);

protected:

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

    /**
     * Returns a specific atom in the outgoing set (using the TLB).
     *
     * @param The position of the atom in the array.
     * @return A specific atom in the outgoing set (using the TLB).
     */
    inline Atom * getOutgoingAtom(int pos) const
    {
        return TLB::getAtom(getOutgoingHandle(pos));
    }

public:

    /**
     * Constructor for this class.
     *
     * @param Link type.
     * @param Outgoing set, which is an array of the atom handles
     *        referenced by this link.
     * @param Link truthvalue, which will be cloned before being
     *        stored in this Link.
     */
    Link(Type t, const std::vector<Handle>& oset,
         const TruthValue& tv = TruthValue::NULL_TV())
        : Atom(t, tv)
    {
        init(oset);
    }

    Link(Type t, Handle& h,
         const TruthValue& tv = TruthValue::NULL_TV()) 
        : Atom(t, tv)
    {
        std::vector<Handle> oset;
        oset.push_back(h);
        init(oset);
    }

    Link(Type t, Handle& ha, Handle &hb,
         const TruthValue& tv = TruthValue::NULL_TV()) 
        : Atom(t, tv)
    {
        std::vector<Handle> oset;
        oset.push_back(ha);
        oset.push_back(hb);
        init(oset);
    }

    Link(Type t, Handle& ha, Handle &hb, Handle &hc,
         const TruthValue& tv = TruthValue::NULL_TV()) 
        : Atom(t, tv)
    {
        std::vector<Handle> oset;
        oset.push_back(ha);
        oset.push_back(hb);
        oset.push_back(hc);
        init(oset);
    }
    Link(Type t, Handle& ha, Handle &hb, Handle &hc, Handle &hd,
         const TruthValue& tv = TruthValue::NULL_TV()) 
        : Atom(t, tv)
    {
        std::vector<Handle> oset;
        oset.push_back(ha);
        oset.push_back(hb);
        oset.push_back(hc);
        oset.push_back(hd);
        init(oset);
    }

    /** Copy constructor, does NOT copy atom table membership! */
    Link(const Link &l)
        : Atom(l.getType(), l.getTruthValue(), l.getAttentionValue())
    {
        init(l.getOutgoingSet());
    }

    /**
     * Destructor for this class.
     */
    ~Link();

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
     * Returns a specific Handle in the outgoing set.
     *
     * @param The position of the handle in the array.
     * @return A specific handle in the outgoing set.
     */
    inline Handle getOutgoingHandle(int pos) const throw (RuntimeException)
    {
        // Checks for a valid position
        if ((0 <= pos) && (pos < getArity())) {
            return outgoing[pos];
        } else {
            throw RuntimeException(TRACE_INFO, "invalid outgoing set index %d", pos);
        }
    }

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
     * Note that the TV is only represented by its
     * mean and count so if it is a compositeTV only
     * the primaryTV is printed.
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
    bool isSource(Handle) const throw (InvalidParamException);

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
     * @param Atom to be tested.
     * @return true if they are equal, false otherwise.
     */
    virtual bool operator==(const Atom&) const;

    /**
     * Returns whether a given atom is different from the current link.
     * @param Atom to be tested.
     * @return true if they are different, false otherwise.
     */
    virtual bool operator!=(const Atom&) const;

    /**
     * Returns the hashCode of the link.
     * @return an unsigned integer value as the hashCode of the link.
     */
    virtual size_t hashCode(void) const;

    virtual Atom* clone() const;
};

} // namespace opencog

#endif // _OPENCOG_LINK_H
