/*
 * opencog/reasoning/pln/PLNModule.h
 *
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
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

#ifndef _OPENCOG_PLN_MODULE_H
#define _OPENCOG_PLN_MODULE_H

#include <string>

//#include <opencog/server/Agent.h>
#include <opencog/server/CogServer.h>
#include <opencog/server/Factory.h>
#include <opencog/server/Module.h>
#include <opencog/server/Request.h>

namespace opencog
{

class CogServer;

class PLNModule : public Module
{
private:

    DECLARE_CMD_REQUEST(PLNModule, "pln", do_pln, 
       "Run a PLN command", 
       "Usage: pln <command>\n\n"
       "Run the specified PLN command.")

public:

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::PLNModule");
        return _ci;
    }
    
    static inline const char* id();

    PLNModule();
    ~PLNModule();
    void init();

}; // class

} // namespace opencog


#endif // _OPENCOG_PLN_MODULE_H

