/*
 * opencog/server/BuiltinRequestsModule.h
 *
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
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

#ifndef _OPENCOG_BUILTIN_REQUESTS_MODULE_H
#define _OPENCOG_BUILTIN_REQUESTS_MODULE_H

#include <opencog/server/DataRequest.h>
#include <opencog/server/ExitRequest.h>
#include <opencog/server/Factory.h>
#include <opencog/server/HelpRequest.h>
#include <opencog/server/ListRequest.h>
#include <opencog/server/LoadModuleRequest.h>
#include <opencog/server/LoadRequest.h>
#include <opencog/server/Module.h>
#include <opencog/server/ShutdownRequest.h>
#include <opencog/server/UnloadModuleRequest.h>

namespace opencog
{

class BuiltinRequestsModule : public Module
{

private:

    Factory<ListRequest, Request>         listFactory;
    Factory<LoadRequest, Request>         loadFactory;
    Factory<DataRequest, Request>         dataFactory;
    Factory<HelpRequest, Request>         helpFactory;
    Factory<ExitRequest, Request>         exitFactory;
    Factory<ShutdownRequest, Request>     shutdownFactory;
    Factory<LoadModuleRequest, Request>   loadmoduleFactory;
    Factory<UnloadModuleRequest, Request> unloadmoduleFactory;

public:

    static inline const char* id() {
        static const char* _id = "opencog::BuiltinRequestsModule";
        return _id;
    }

    BuiltinRequestsModule();
    virtual ~BuiltinRequestsModule();
    virtual void init();

}; // class

}  // namespace

#endif // _OPENCOG_BUILTIN_REQUESTS_MODULE_H
