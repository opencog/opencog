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

#include <map>
#include <ostream>
#include <string>
#include <boost/thread.hpp>
#include <boost/operators.hpp>

#define COMBO_ENUM_TYPE_PREFIX "enum_type:"

namespace opencog { namespace combo {

// enum_t is essentially a string but is coded as a different type
// than definite_object (or message), because it semantically denotes
// something else.
class enum_t
    : boost::less_than_comparable<enum_t>, // generate >, <= and >= given <
      boost::equality_comparable<enum_t> // generate != given ==
{
private:
    unsigned id;
    std::string _content;

    // Global table of string-to-int, so that the operator==()
    // can run efficiently (for scoring) without a string-compare.
    static unsigned enum_issued;
    static std::map<std::string, unsigned> enum_map;
    static boost::shared_mutex id_mutex;

protected:
    // Issue a unique id number.
    unsigned get_id(const std::string& token);

    enum_t(const std::string &m, unsigned i)
    {
        _content = m;
        id = i;
    }

public:
    enum_t(const std::string &m)
    {
        _content = m;
        id = get_id (_content);
    }

    unsigned getId() const {
        return id;
    }

    std::string getContent() const {
        return _content;
    }

    bool operator==(const enum_t& m) const {
        return id == m.id;
    }

    bool operator<(const enum_t& m) const {
        /// XXX Why are we not using id?
        return _content < m.getContent();
    }
    
    static std::string prefix() {
        return COMBO_ENUM_TYPE_PREFIX;
    }
    static const enum_t& get_random_enum();
    static size_t size() {
        return enum_map.size();
    }
    static const enum_t& invalid_enum() {
        static enum_t bad("", -1);
        return bad;
    }
};

std::ostream& operator<<(std::ostream&, const opencog::combo::enum_t&);

} // ~namespace combo
} // ~namespace opencog

#endif

