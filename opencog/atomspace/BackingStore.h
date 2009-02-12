/*
 * opencog/atomspace/BackingStore.h
 *
 * Implements an interface class for storage providers.
 *
 * Copyright (C) 2009 Linas Vepstas <linasvepstas@gmail.com>
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

namespace opencog
{

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
		/** 
		 * Return a handle to a link of the indicated type and outset,
		 * if it exists; else return an undefined handle.  */
		virtual Handle getHandle(Type, const std::vector<Handle>&) const = 0;

		/** 
		 * Return a handle to a node of the indicated type and name,
		 * if it exists; else return an undefined handle.  */
		virtual Handle getHandle(Type, const char *) const = 0;
};

} //namespace opencog

#endif // _OPENCOG_BACKING_STORE_H
