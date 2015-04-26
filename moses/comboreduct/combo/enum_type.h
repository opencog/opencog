/*
 * moses/comboreduct/combo/enum_type.h
 *
 * Copyright (C) 2002-2008, 2012 Novamente LLC
 * Copyright (C) 2012 Poulin Holdings LLC
 * All Rights Reserved
 *
 * Written by Nil Geisweiller, Linas Vepstas
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://moses.org/wiki/Licenses
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

namespace moses { namespace combo {

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
    // Issue a unique id number for each unique string.
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
        // Compare Id's, not strings, because that just faster.
        return id == m.id;
    }

    /// Lexicographic ordering on the enum string names.
    bool operator<(const enum_t& m) const {
        return _content < m.getContent();
    }

    /// This is used by enum_str_to_vertex() to identify enums.
    static std::string prefix() {
        return "enum:";
    }

    /// Return some random enum out of the pool of all of them.
    static enum_t get_random_enum();

    /// Return the total number of enum types that have been issued.
    static size_t size();

    /// Return an enum that is gaurenteed to be not equal to any other.
    static const enum_t& invalid_enum() {
        static enum_t bad("", -1);
        return bad;
    }
};

std::ostream& operator<<(std::ostream&, const moses::combo::enum_t&);

} // ~namespace combo
} // ~namespace moses

#endif

