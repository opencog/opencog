/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
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

#ifndef _ATOMLOOKUPPROVIDER_H
#define _ATOMLOOKUPPROVIDER_H

#include <boost/smart_ptr.hpp>

#include <set>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/types.h>

#ifdef WIN32
#include <string>
#endif

using namespace opencog;

class AtomLookupProvider
{
public:
    /** If name is non-empty, then return the set of all nodes with type T and
     * that name.
     * Else, return the set of all links with type T. Whether to include
     * subclasses too is optional.  NOTE! atoms with confidence < 0.0000001 are
     * not returned!
     */
    virtual boost::shared_ptr<set<Handle> > getHandleSet(Type T, const string&
            name, bool subclass = false) =0 ;

    /** Return the set of all nodes with type T and the name str
     * NOTE! atoms with confidence < 0.0000001 are not returned!
     */
    virtual Handle getHandle(Type t,const std::string& str) =0;

    /** return the set of all links with type T and the given outgoing set
      * NOTE! atoms with confidence < 0.0000001 are not returned!
      */
    virtual Handle getHandle(Type t,const HandleSeq& outgoing) =0;
	virtual ~AtomLookupProvider() { };
};

#endif // _ATOMLOOKUPPROVIDER_H
