/*
 * opencog/server/ExitRequest.h
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

#ifndef _OPENCOG_EXIT_REQUEST_H
#define _OPENCOG_EXIT_REQUEST_H

#include <vector>
#include <string>

#include <opencog/atomspace/types.h>
#include <opencog/server/Request.h>

namespace opencog
{

class ExitRequest : public Request
{

public:

    static inline const RequestClassInfo& info() {
        static const RequestClassInfo _cci(
            "exit",
            "Close the shell connection",
            "Usage: exit\n\n"
            "Close the shell TCP/IP connection."
        );
        return _cci;
    }

    ExitRequest();
    virtual ~ExitRequest();
    virtual bool execute(void);
    virtual bool isShell(void) {return info().is_shell;}
};

} // namespace 

#endif // _OPENCOG_EXIT_REQUEST_H
