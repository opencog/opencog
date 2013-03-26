/*
 * opencog/atomspace/IncomingIndex.h
 *
 * Copyright (C) 2008,2009,2013 Linas Vepstas <linasvepstas@gmail.com>
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

#ifndef _OPENCOG_INCOMING_INDEX_H
#define _OPENCOG_INCOMING_INDEX_H

#include <set>
#include <vector>

#include <opencog/atomspace/HandleIndex.h>
#include <opencog/atomspace/types.h>

namespace opencog
{

/**
 * Implements a Handle index array of RB-trees (C++ set)
 * Given a Handle, this returns the incoming set of that handle.
 */
class IncomingIndex
{
    private:
        HandleIndex idx;
    public:
        IncomingIndex(void);
        void insertAtom(const Atom* a);
        void removeAtom(const Atom* a);
        void remove(bool (*)(Handle));
        void resize();

        const HandleSeq& getIncomingSet(Handle) const;
};

} //namespace opencog

#endif // _OPENCOG_INCOMING_INDEX_H
