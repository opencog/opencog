/*
 * opencog/comboreduct/combo/message.cc
 *
 * Copyright (C) 2012 Poulin Holdings
 * All Rights Reserved
 *
 * Written by Linas Vepstas
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
#include "list_type.h"

namespace opencog { namespace combo {
using namespace std;

ostream& operator<<(ostream& out, const combo::list_t& m)
{
    OC_ASSERT(*m.get_tree().begin() == id::list);
    // return out << m.get_tree();
    return out << "list printing NOT YET IMPLEMENTED!";
}

} // ~namespace combo
} // ~namespace opencog

