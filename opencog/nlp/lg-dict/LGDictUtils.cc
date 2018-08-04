/*
 * LGDictUtils.cc
 *
 * Copyright (C) 2015 OpenCog Foundation
 *
 * Author: William Ma <https://github.com/williampma>
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

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/base/Link.h>
#include <opencog/atoms/base/Node.h>
#include <opencog/nlp/types/atom_types.h>

#include "LGDictUtils.h"

using namespace opencog::nlp;
using namespace opencog;


namespace opencog
{
namespace nlp
{

Handle lg_conn_get_type(const Handle& hConn)
{
    return hConn->getOutgoingSet()[0];
}

Handle lg_conn_get_dir(const Handle& hConn)
{
    return hConn->getOutgoingSet()[1];
}

/**
 * Check if two connectors' type matches.

XXX FIXME -- this currently fails to correctly handle the head-tail
indicators on link types.  Perhaps it would be better to just call
the same matchinng routines the link-grammar uses?
 *
 * @param hConn1   the first LGConnector
 * @param hConn2   the second LGConnector
 * @return         true if the type matches
 */
bool lg_conn_type_match(const Handle& hConn1, const Handle& hConn2)
{
    if (hConn1->get_type() != LG_CONNECTOR or
        hConn2->get_type() != LG_CONNECTOR)
        return false;

    // Convert the types to string
    std::string type1 = lg_conn_get_type(hConn1)->get_name();
    std::string type2 = lg_conn_get_type(hConn2)->get_name();
    uint i1 = 0;
    uint i2 = 0;

    // check header
    if (islower((int) type1[i1]))
        i1++;
    if (islower((int) type2[i2]))
        i2++;

    if (i1 > 0 && i2 > 0 && type1[0] == type2[0])
        return false;

    while (i1 < type1.length() and i2 < type2.length())
    {
        if (isupper((int) type1[i1]) or isupper((int) type2[i2]))
        {
            if (type1[i1] != type2[i2])
                return false;

            i1++;
            i2++;
            continue;
        }

        if (type1[i1] != '*' and type2[i2] != '*' and type1[i1] != type2[i2])
            return false;

        i1++;
        i2++;
    }

    return true;
}

/**
 * Check if two connectors can be linked.
 *
 * The two connectors must have different directions, but does not matter
 * which one is + and which one is -.
 *
 * @param hConn1   the first LGConnector
 * @param hConn2   the second LGConnector
 * @return         true if linkable
 */
bool lg_conn_linkable(const Handle& hConn1, const Handle& hConn2)
{
    return lg_conn_type_match(hConn1, hConn2) and
        lg_conn_get_dir(hConn1) != lg_conn_get_dir(hConn2);
}

}
}
