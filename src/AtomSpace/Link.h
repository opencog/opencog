/**
 * Link.h
 *
 * Copyright(c) 2001 Thiago Maia, Andre Senna
 * All rights reserved.
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

    private:
        Trail* trail;
        void init(void) throw (InvalidParamException);

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
        std::string toString();

        /**
         * Returns a short string representation of the link.
         *
         * @return A short string representation of the link.
         */
        std::string toShortString();

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

};

#endif
