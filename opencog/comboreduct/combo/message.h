/*
 * opencog/comboreduct/combo/message.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
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
#ifndef _COMBO_MESSAGE_H
#define _COMBO_MESSAGE_H

#include <ostream>
#include <set>
#include <string>
#include <boost/operators.hpp>

namespace opencog { namespace combo {

// message is essentially a string but is coded as a different type
// than definite_object, because it semantically denotes something else.
class message
    : boost::less_than_comparable<message>, // generate >, <= and >= given <
      boost::equality_comparable<message>   // generate != given ==
{
private:
    std::string _content;
public:
    message(const std::string& m) {
        _content = m;
    }

    std::string getContent() const {
        return _content;
    }

    /// @todo: this operator is used for scoring, and so the string
    /// compare may make things slow... perhaps we should use a map
    /// the same way that enum_t does.
    bool operator==(const message& m) const {
        return _content == m.getContent();
    }

    bool operator<(const message& m) const {
        return _content < m.getContent();
    }
    
    /// This is used by message_str_to_vertex() to identify a message.
    static std::string prefix() {
        return "message:";
    }
};

typedef std::set<message> message_set;
typedef message_set::iterator message_set_it;
typedef message_set::const_iterator message_set_const_it;

std::ostream& operator<<(std::ostream&, const opencog::combo::message&);

} // ~namespace combo
} // ~namespace opencog

#endif

