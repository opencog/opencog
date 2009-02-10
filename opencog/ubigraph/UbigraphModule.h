/*
 * opencog/ubigraph/UbigraphModule.h
 *
 * Copyright (C) 2009 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by Jared Wigmore <jared.wigmore@gmail.com>
 * Adapted from DottyModule (which is by Trent Waddington <trent.waddington@gmail.com>)
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

#ifndef _OPENCOG_UBIGRAPH_MODULE_H
#define _OPENCOG_UBIGRAPH_MODULE_H

#include <string>

#include <opencog/server/CogServer.h>
#include <opencog/server/Factory.h>
#include <opencog/server/Module.h>
#include <opencog/server/Request.h>

namespace opencog
{

class CogServer;

class UbigraphModule : public Module
{
private:

   DECLARE_CMD_REQUEST(UbigraphModule, "ubigraph", do_ubigraph, 
        "ubigraph prototype command",
        "Usage: ubigraph\n\n"
        "test the ubigraph prototype"
   )

public:

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::UbigraphModule");
        return _ci;
    }
    
    static inline const char* id();

    UbigraphModule();
    ~UbigraphModule();
    void init();

}; // class

} // namespace opencog


#endif // _OPENCOG_UBIGRAPH_MODULE_H

