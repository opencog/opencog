/*
 * opencog/atomspace/BackingStore.h
 *
 * Implements an interface class for storage providers.
 *
 * Copyright (C) 2009, 2013 Linas Vepstas <linasvepstas@gmail.com>
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

#ifndef _OPENCOG_BACKING_STORE_H
#define _OPENCOG_BACKING_STORE_H

#include <set>

#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/**
 * This class provides a simple, generic interface for dynamically
 * storing/retreiving atoms from disk or other remote location or
 * process. This class focuses on "on-demand" atom retreival,
 * rather than on bulk-save/restore (although perhaps that should
 * be provided as well.)  
 */
class BackingStore
{
	public:
		virtual ~BackingStore() {}

		/** 
		 * Return a pointer to a link of the indicated type and outset,
		 * if it exists; else return NULL.
		 */
		virtual LinkPtr getLink(Type, const HandleSeq&) const = 0;

		/** 
		 * Return a pointer to a node of the indicated type and name,
		 * if it exists; else return NULL.
		 */
		virtual NodePtr getNode(Type, const char *) const = 0;

		/** 
		 * Return a pointer to an Atom associated with the given
		 * handle, if it exists; else return NULL.
		 */
		virtual AtomPtr getAtom(Handle) const = 0;

		/**
		 * Return a vector containing the handles of the entire incoming
		 * set of the indicated handle. 
		 */
		virtual HandleSeq getIncomingSet(Handle) const = 0;

		/**
		 * Recursively store the atom and anything in it's outgoing set.
		 * If the atom is already in storage, this will update it's 
		 * truth value, etc. 
		 */
		virtual void storeAtom(Handle) = 0;

		/**
		 * Read-write synchronization barrier.
		 * All writes will be completed before this routine returns.
		 * This allows the backend to implement asynchronous writes,
		 * while still providing some control to those who need it.
		 * (Mostly the unit tests, at this time.)
		 */
		virtual void barrier() = 0;

		/**
		 * Returns true if the backing store will ignore this type.
		 * This is used for performance optimization, as asking the
		 * backend to retreive an atom can take a long time. If an atom
		 * is of this given type, it will not be fetched.
		 */
		virtual bool ignoreType(Type t) const {
			 return (_ignored_types.end() != _ignored_types.find(t));
		}

		/**
		 * Returns true if the backing store will ignore this atom,
		 * either because it is of an ignorable type, or is a link
		 * which contains an atom that is of an ignorable type.
		 */
		virtual bool ignoreAtom(Handle) const;

		/**
		 * The set of ignored atom types.
		 */
		std::set<Type> _ignored_types;
};

/** @}*/
} //namespace opencog

#endif // _OPENCOG_BACKING_STORE_H
