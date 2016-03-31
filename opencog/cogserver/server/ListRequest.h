/*
 * opencog/cogserver/server/ListRequest.h
 *
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Gustavo Gama <gama@vettalabs.com>
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

#ifndef _OPENCOG_LIST_REQUEST_H
#define _OPENCOG_LIST_REQUEST_H

#include <sstream>
#include <string>
#include <vector>

#include <opencog/atoms/base/Handle.h>
#include <opencog/cogserver/server/Request.h>
#include <opencog/cogserver/server/RequestClassInfo.h>

namespace opencog
{
/** \addtogroup grp_server
 *  @{
 */

class ListRequest : public Request
{

protected:

    HandleSeq _handles;
    std::ostringstream  _error;

    void sendOutput(void);
    void sendError (void);
    bool syntaxError(void);

public:

    static inline const RequestClassInfo& info() {
        static const RequestClassInfo _cci(
            "list",
            "List atoms in the atomtable",
            "Usage: list [[-h <handle>] | [-n <name>] [[-t|-T] <type>]] [-m <num>] \n\n"
            "List atoms in the atomtable. Optional flags are:\n"
            "   -h <handle>: list the atom identified by the specified handle\n"
            "   -n <name>:   list the nodes identified by the specified name\n"
            "   -t <name>:   list the atoms of the specified type\n"
            "   -T <name>:   list the atoms of the specified type (including subtypes)\n"
            "   -m <num>:    list the nodes up to the specified size"
        );
        return _cci;
    }

    ListRequest(CogServer&);
    virtual ~ListRequest();
    virtual bool execute(void);
    virtual bool isShell(void) {return info().is_shell;}
};

/** @}*/
} // namespace 

#endif // _OPENCOG_LIST_REQUEST_H
