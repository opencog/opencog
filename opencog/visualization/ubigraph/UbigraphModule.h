/*
 * opencog/visualization/ubigraph/UbigraphModule.h
 *
 * Copyright (C) 2008-2009 by OpenCog Foundation
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

#include "Ubigrapher.h"

namespace opencog
{

class CogServer;

class UbigraphModule : public Module
{
private:

    DECLARE_CMD_REQUEST(UbigraphModule, "ubigraph", do_ubigraph, 
        "Start Ubigraph",
        "Usage: ubigraph [--with-incoming, --compact] [IP:port]\n\n"
        "test the ubigraph prototype",
        false, false
    )

    DECLARE_CMD_REQUEST(UbigraphModule, "ubigraph-update-sti", do_ubigraphUpdate, 
        "update node size in ubigraph based on STI",
        "Usage: ubigraph-update-sti\n\n"
        "Update the nodes, this is a test function for visual demos.\n"
        "The size of all nodes reflect their relative STI values.", 
        false, false
    )

    DECLARE_CMD_REQUEST(UbigraphModule, "ubigraph-random-sti", do_ubigraphRandomSTI, 
        "randomly assign STI",
        "Usage: ubigraph-random-sti\n"
        "Temp function, sets STI of node.\n",
        false, false
    )

    Ubigrapher g;

public:

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::UbigraphModule");
        return _ci;
    }
    
    static const char* id();

    UbigraphModule(CogServer&);
    virtual ~UbigraphModule();
    virtual void init();

}; // class

} // namespace opencog


#endif // _OPENCOG_UBIGRAPH_MODULE_H

