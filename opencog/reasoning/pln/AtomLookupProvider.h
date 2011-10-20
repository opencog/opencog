/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
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

namespace opencog {
namespace pln {

// PLN atom handle, which can also wrapper atom types. 
typedef unsigned int pHandle;
typedef std::vector<pHandle> pHandleSeq;
// pHandle ranges: 0 <= h < NOTYPE (atom types); h > NOTYPE (pln atom handles)
const static unsigned int mapOffset = NOTYPE+1;
#define PHANDLE_UNDEFINED NOTYPE

class AtomLookupProvider
{

public:
    /** If name is non-empty, then return the set of all nodes with type T and
     * that name.
     * Else, return the set of all links with type T. Whether to include
     * subclasses too is optional.  NOTE! atoms with confidence < 0.0000001 are
     * not returned!
     */
    virtual boost::shared_ptr<std::set<pHandle> > getHandleSet(Type T,
const std::string& name, bool subclass = false) =0 ;

    /** Return pHandle with type T and the name str
     */
    virtual pHandle getHandle(Type t,const std::string& str) =0;

    /** return pHanlde with type T and the given outgoing set
      */
    virtual pHandle getHandle(Type t,const pHandleSeq& outgoing) =0;
    virtual ~AtomLookupProvider() { };
};

}} //~namespace opencog::pln

#endif // _ATOMLOOKUPPROVIDER_H
