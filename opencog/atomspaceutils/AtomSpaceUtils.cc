/*
 * AtomSpaceUtils.cc
 *
 * Copyright (C) 2014 OpenCog Foundation
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

#include <opencog/atomspace/atom_types.h>
#include <opencog/atomspace/Link.h>
#include "AtomSpaceUtils.h"

namespace opencog {

Handle addPrefixedNode(AtomSpace& as, Type t, const std::string& prefix)
{
    static const char alphanum[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";
    static const int len = 8;
    std::string name;
    Handle result;
    // Keep trying random suffixes until a non-existant name is generated.
    do {
        name = prefix;
        for (int i = 0; i < len; ++i) {
            name += alphanum[rand() % (sizeof(alphanum) - 1)];
        }
        result = as.getHandle(t, name);
    } while (as.isValidHandle(result));

    return as.addNode(t, name);
}

} // namespace opencog
