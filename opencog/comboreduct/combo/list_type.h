/*
 * opencog/comboreduct/combo/list_type.h
 *
 * Copyright (C) 2002-2008, 2012 Novamente LLC
 * All Rights Reserved
 *
 * Written by Nil Geisweiller, Linas Vepstas
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
#ifndef _COMBO_LIST_TYPE_H
#define _COMBO_LIST_TYPE_H

#include <ostream>
#include <boost/operators.hpp>
#include "vertex.h"

namespace opencog { namespace combo {

/// list_t is really just a combo tree. :-)
class list_t
    : public list_base,
      boost::less_than_comparable<list_t>, // generate >, <= and >= given <
      boost::equality_comparable<list_t>   // generate != given ==
{
public:
    list_t(const combo_tree &tr)
    {
        _tr = tr;
    }

    const combo_tree& get_tree() const
    {
        return _tr;
    }

    bool operator==(const list_t& m) const
    {
        return _tr == m._tr;
    }

    /// Lexicographic ordering on trees ... ??? does this work?
    bool operator<(const list_t& m) const
    {
        OC_ASSERT(false, "list_t comparison NOT YET IMPLEMENTED");
        return true;
    }

protected:
    friend class list_ptr;
    virtual list_base* copy() const
    {
        return new list_t(_tr);
    }

private:
    combo_tree _tr;

};

inline bool is_list_type(const vertex& v)
{
    return boost::get<list_ptr>(&v);
}

inline const list_t& get_list_type(const vertex& v)
{
    const list_ptr* p = boost::get<list_ptr>(&v);
    return *(dynamic_cast<const list_t*>(p->get()));
}

inline const combo_tree& get_list_tree(const vertex& v)
{
    return get_list_type(v).get_tree();
}

std::ostream& operator<<(std::ostream&, const opencog::combo::list_t&);


} // ~namespace combo
} // ~namespace opencog

#endif

