/*
 * opencog/server/SaveRequest.h
 *
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Jared Wigmore <jared.wigmore@gmail.com>
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

#ifndef _OPENCOG_SAVE_REQUEST_H
#define _OPENCOG_SAVE_REQUEST_H

#include <vector>
#include <string>

#include <opencog/server/Request.h>

namespace opencog
{

class SaveRequest : public Request
{

public:

    static inline const RequestClassInfo& info() {
        static const RequestClassInfo _cci(
            "save",
            "Save the AtomSpace to an NMXML file",
            "Usage: save <filename>\n\n"
            "Save the entire contents of the AtomSpace to a file,\n"
            "written in the NMXML data format."
        );
        return _cci;
    }

    SaveRequest();
    virtual ~SaveRequest();
    virtual bool execute(void);
    virtual bool isShell(void) {return info().is_shell;}
};

} // namespace 

#endif // _OPENCOG_SAVE_REQUEST_H
