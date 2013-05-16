/*
 * opencog/dotty/DottyModule.h
 *
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Trent Waddington <trent.waddington@gmail.com>
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

#ifndef _OPENCOG_DOTTY_MODULE_H
#define _OPENCOG_DOTTY_MODULE_H

#include <string>

#include <opencog/server/CogServer.h>
#include <opencog/server/Factory.h>
#include <opencog/server/Module.h>
#include <opencog/server/Request.h>

namespace opencog
{

class CogServer;

class DottyModule : public Module
{
private:

    DECLARE_CMD_REQUEST(DottyModule, "dotty", do_dotty, 
       "Dumps a (very big) dotty file of the atomspace.", 
       "Usage: dotty\n\n"
       "Dotty, also known as GraphViz, is a graph visualization package.", 
       false, false);

public:

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::DottyModule");
        return _ci;
    }
    
    static inline const char* id();

    DottyModule();
    ~DottyModule();
    void init();

}; // class

} // namespace opencog


#endif // _OPENCOG_DOTTY_MODULE_H

