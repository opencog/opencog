/*
 * opencog/atomspace/Handle.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
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

#ifndef _OPENCOG_HANDLE_H
#define _OPENCOG_HANDLE_H

#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <tr1/unordered_set>

namespace opencog
{

class Handle
{

private:

    unsigned long _value;

public:

    static const Handle UNDEFINED;

    Handle(const Handle& h) : _value(h._value) {};
    explicit Handle() : _value(UNDEFINED._value) {};
    explicit Handle(const unsigned long h) : _value(h) {};
    ~Handle() {}

    inline unsigned long value(void) const {
        return _value;
    }

    inline std::string str(void) const {
        std::ostringstream oss;
        oss << _value;
        return oss.str();
    }

    inline Handle& operator=(const Handle& h) {
        _value = h._value;
        return *this;
    }

    inline bool operator==(const Handle& h) const { return _value == h._value; }
    inline bool operator!=(const Handle& h) const { return _value != h._value; }
    inline bool operator< (const Handle& h) const { return _value <  h._value; }
    inline bool operator> (const Handle& h) const { return _value >  h._value; }
    inline bool operator<=(const Handle& h) const { return _value <= h._value; }
    inline bool operator>=(const Handle& h) const { return _value >= h._value; }

};

typedef std::vector<Handle> HandleSeq;
typedef std::vector<HandleSeq> HandleSeqSeq;

} // namespace opencog

namespace std { namespace tr1 {
    template<>
    struct hash<opencog::Handle> : public std::unary_function<const opencog::Handle&, std::size_t> {
        std::size_t operator()(const opencog::Handle& h) const
        { return static_cast<std::size_t>(h.value()); }
    };
}} //namespace std::tr1


inline std::ostream& operator<<(std::ostream& out, const opencog::Handle& h) {
    out << h.value();
    return out;
}

#endif // _OPENCOG_HANDLE_H
