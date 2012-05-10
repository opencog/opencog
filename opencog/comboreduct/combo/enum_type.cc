/*
 * opencog/comboreduct/combo/message.cc
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
#include "enum_type.h"

namespace opencog { namespace combo {
using namespace std;

// Global table of string-to-int, so that the operator==()
// can run efficiently (for scoring) without a string-compare.
unsigned enum_t::enum_issued = 0;
map<string, unsigned> enum_t::enum_map;
boost::shared_mutex enum_t::id_mutex;

unsigned enum_t::get_id(const string& token)
{
    typedef boost::shared_mutex mutex;
    typedef boost::unique_lock<mutex> unique_lock;

    unique_lock lock(id_mutex);

    map<string, unsigned>::iterator entry = enum_map.find(token);
    if (entry == enum_map.end()) {
       enum_issued ++;
       entry = enum_map.insert(pair<string, unsigned>(token, enum_issued)).first;
    }
    return (unsigned) entry->second;
}

ostream& operator<<(ostream& out, const combo::enum_t& m)
{
    return out << combo::enum_t::prefix() 
               << "(" << m.getId() << "):"
               << '\"' << m.getContent() << '\"';
}

} // ~namespace combo
} // ~namespace opencog

