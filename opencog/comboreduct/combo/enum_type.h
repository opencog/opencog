/*
 * opencog/comboreduct/combo/enum_type.h
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
#ifndef _COMBO_ENUM_TYPE_H
#define _COMBO_ENUM_TYPE_H

#include <opencog/util/exceptions.h>

#include "type_tree_def.h"

#define COMBO_ENUM_TYPE_PREFIX "message:"

namespace opencog { namespace combo {

// enum_t is essentially a string but is coded as a different type
// than definite_object (or message), because it semantically denotes
// something else.
class enum_t
{
private:
    unsigned id;
    std::string _content;

    // Global table of string-to-int, so that the operator==()
    // can run efficiently (for scoring) without a string-compare.
    static unsigned enum_issued;
    static std::map<std::string, unsigned> enum_map;

    unsigned get_id(const std::string& token)
    {
        // XXX Need a thread lock here.
        map<string, unsigned>::iterator entry = enum_map.find(token);
        if (entry == enum_map.end()) {
           enum_issued ++;
           entry = enum_map.insert(pair<string, unsigned>(token, enum_issued)).first;
        }
        return (unsigned) entry->second;
    }

public:
    enum_t(const std::string &m)
    {
        _content = m;
        id = get_id (_content);
    }

    std::string getContent() const {
        return _content;
    }

    bool operator==(enum_t m) const {
        return id == m.id;
    }
    bool operator!=(enum_t m) const {
        return id != m.id;
    }
    bool operator<(enum_t m) const {
        return _content < m.getContent();
    }
    bool operator<=(enum_t m) const {
        return _content <= m.getContent();
    }
    bool operator>(enum_t m) const {
        return _content > m.getContent();
    }
    bool operator>=(enum_t m) const {
        return _content >= m.getContent();
    }
    
    static std::string prefix() {
        return COMBO_ENUM_TYPE_PREFIX;
    }
};

std::ostream& operator<<(std::ostream&, const opencog::combo::enum_t&);

} // ~namespace combo
} // ~namespace opencog

#endif

