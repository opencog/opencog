/*
 * opencog/atomspace/LinkIndex.h
 *
 * Copyright (C) 2008,2009 Linas Vepstas <linasvepstas@gmail.com>
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

#ifndef _OPENCOG_LINK_INDEX_H
#define _OPENCOG_LINK_INDEX_H

#include <set>
#include <vector>

#include <opencog/atomspace/HandleSeqIndex.h>
#include <opencog/atomspace/types.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/**
 * Implements an (type, HandleSeq) index array of RB-trees (C++ set)
 * That is, given both a type, and a HandleSeq, it returns a single,
 * unique Handle associated with that pair.  In other words, it returns
 * the single, unique Link which is that pair.
 */
class LinkIndex
{
    private:
        std::vector<HandleSeqIndex> idx;
    public:
        LinkIndex(void);
        void insertAtom(const Atom* a);
        void removeAtom(const Atom* a);
        void remove(bool (*)(Handle));
        void resize();

        Handle getHandle(Type type, const HandleSeq&) const;
        UnorderedHandleSet getHandleSet(Type type, const HandleSeq &, bool subclass) const;
};

/** @}*/
} //namespace opencog

#endif // _OPENCOG_LINK_INDEX_H
