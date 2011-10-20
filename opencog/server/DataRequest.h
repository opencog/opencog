/*
 * src/server/DataRequest.h
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

#ifndef _OPENCOG_DATA_REQUEST_H
#define _OPENCOG_DATA_REQUEST_H

#include <vector>
#include <string>

#include <opencog/atomspace/types.h>
#include <opencog/server/Request.h>
#include <opencog/server/RequestResult.h>

namespace opencog
{

class DataRequest : public Request
{

public:

    static inline const RequestClassInfo& info() {
        static const RequestClassInfo _cci(
            "data",
            "Load inline xml data into the atomspace",
            "Usage: data<CRLF><inline xml><CRLF><Ctrl-D><CRLF>\n\n"
            "Load inline NMXML data. The command must be followed by a\n"
            "carriage-return-linefeed (CRLF), then the XML data. The end\n"
            "of the data is indicated by a single ASCI EOT character (^D)\n"
            "on a line by itself.",
            true
        );
        return _cci;
    }

    DataRequest();
    virtual ~DataRequest();

    virtual void setRequestResult(RequestResult*);
    virtual bool execute(void);
    virtual bool isShell(void) {return info().is_shell;}
};

} // namespace 

#endif // _OPENCOG_DATA_REQUEST_H
