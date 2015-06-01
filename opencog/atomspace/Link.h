/*
 * opencog/atomspace/Link.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008-2010 OpenCog Foundation
 * All Rights Reserved
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

#include <opencog/util/oc_assert.h>
#include <opencog/atomspace/Atom.h>

namespace opencog
{
class AtomTable;
/** \addtogroup grp_atomspace
 *  @{
 */

//! arity of Links, represented as short integer (16 bits)
typedef unsigned short Arity;

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
    friend class AtomTable;

private:
    void init(const HandleSeq&) throw (InvalidParamException);
    void resort(void);

    Link(const Link &l) : Atom(0)
    { OC_ASSERT(false, "Link: bad use of copy ctor"); }

protected:

    //! Array holding actual outgoing set of the link.
    //! Should not change during atom lifespan.
    HandleSeq _outgoing;

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
    Link(Type t, const HandleSeq& oset,
         TruthValuePtr tv = TruthValue::DEFAULT_TV(),
         AttentionValuePtr av = AttentionValue::DEFAULT_AV())
        : Atom(t, tv, av)
    {
        init(oset);
    }

    Link(Type t, Handle& h,
         TruthValuePtr tv = TruthValue::DEFAULT_TV(),
         AttentionValuePtr av = AttentionValue::DEFAULT_AV())
        : Atom(t, tv, av)
    {
        HandleSeq oset;
        oset.push_back(h);
        init(oset);
    }

    Link(Type t, Handle& ha, Handle &hb,
         TruthValuePtr tv = TruthValue::DEFAULT_TV(),
         AttentionValuePtr av = AttentionValue::DEFAULT_AV())
        : Atom(t, tv, av)
    {
        HandleSeq oset;
        oset.push_back(ha);
        oset.push_back(hb);
        init(oset);
    }

    Link(Type t, Handle& ha, Handle &hb, Handle &hc,
         TruthValuePtr tv = TruthValue::DEFAULT_TV(),
         AttentionValuePtr av = AttentionValue::DEFAULT_AV())
        : Atom(t, tv, av)
    {
        HandleSeq oset;
        oset.push_back(ha);
        oset.push_back(hb);
        oset.push_back(hc);
        init(oset);
    }
    Link(Type t, Handle& ha, Handle &hb, Handle &hc, Handle &hd,
         TruthValuePtr tv = TruthValue::DEFAULT_TV(),
         AttentionValuePtr av = AttentionValue::DEFAULT_AV())
        : Atom(t, tv, av)
    {
        HandleSeq oset;
        oset.push_back(ha);
        oset.push_back(hb);
        oset.push_back(hc);
        oset.push_back(hd);
        init(oset);
    }

    /**
     * Copy constructor, does NOT copy atom table membership!
     * Cannot be const, because the get() functions can't be,
     * because thread-safe locking required in the gets. */
    Link(Link &l)
        : Atom(l.getType(), l.getTruthValue(), l.getAttentionValue())
    {
        init(l.getOutgoingSet());
    }

    /**
     * Destructor for this class.
     */
    ~Link();

    inline Arity getArity() const {
        return _outgoing.size();
    }

    /**
     * Returns a const reference to the array containing this
     * atom's outgoing set.
     *
     * @return A const reference to this atom's outgoing set.
     */
    inline const HandleSeq& getOutgoingSet() const
    {
        return _outgoing;
    }
    /**
     * Returns a specific Handle in the outgoing set.
     *
     * @param The position of the handle in the array.
     * @return A specific handle in the outgoing set.
     */
    inline Handle getOutgoingAtom(Arity pos) const throw (RuntimeException)
    {
        // Checks for a valid position
        if (pos < getArity()) {
            return _outgoing[pos];
        } else {
            throw RuntimeException(TRACE_INFO, "invalid outgoing set index %d", pos);
        }
    }

    /**
     * Returns a string representation of the link.
     *
     * @return A string representation of the link.
     */
    std::string toString(std::string indent = "");

    /**
     * Returns a short string representation of the link.
     * Note that the TV is only represented by its
     * mean and count so if it is a compositeTV only
     * the primaryTV is printed.
     *
     * @return A short string representation of the link.
     */
    std::string toShortString(std::string indent = "");

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
    bool isSource(size_t) const throw (IndexErrorException, InvalidParamException);

    /**
     * Returns whether a given handle is a target of this link.
     *
     * @param Handle to be checked for being a link target.
     * @return Whether a given handle is a target of this link.
     */
    bool isTarget(Handle) const throw (InvalidParamException);

    /**
     * Returns whether the element in a given position in the
     * outgoing set of this link is a target.
     *
     * @param Position in the outgoing set.
     * @return Whether the element in a given position in the
     *         outgoing set of this link is a target.
     */
    bool isTarget(size_t) const throw (IndexErrorException, InvalidParamException);

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
};

static inline LinkPtr LinkCast(const Handle& h)
    { AtomPtr a(h); return std::dynamic_pointer_cast<Link>(a); }
static inline LinkPtr LinkCast(const AtomPtr& a)
    { return std::dynamic_pointer_cast<Link>(a); }

// XXX temporary hack ...
#define createLink std::make_shared<Link>

/** @}*/
} // namespace opencog

#endif // _OPENCOG_LINK_H
