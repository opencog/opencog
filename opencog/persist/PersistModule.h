/*
 * opencog/persist/PersistModule.h
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

#ifndef _OPENCOG_PERSIST_MODULE_H
#define _OPENCOG_PERSIST_MODULE_H

#include <vector>
#include <string>

#include "AtomStorage.h"
#include <opencog/server/CogServer.h>
#include <opencog/server/Factory.h>
#include <opencog/server/Module.h>
#include <opencog/server/Request.h>

// The following two macros provide a simple interface to the module
// subsystem, allowing modules to be created with a minimum of effort 
// and overhead.  The basic design goals of these macros are:
//
// 1) A module implementation is isolated from the detailed inner guts
//    and workings of the module loading mechanism. This allows changes
//    to the module loading mechanism to be made, without also having
//    to refactor each and every module out there.
//
// 2) Individual modules now have a simple, straightforward interface
//    for specifying a command, and what should be done in reaction to
//    that command. In particular, the person creating a new module 
//    does not have to learn how the module system works; they can 
//    focus all thier energies on creating the module.

// Declare a command to the module command processing system.
//
// Arguments:
// cmd:       the name of the command
// cmd_sum:   a string to be printed as a command summary.
// cmd_fmt:   a string illustrating the command and its paramters.
//
// This should probably be implemented as a template, rather than a
// macro, but for now, a macro will serve as a quick-n-dirty solution
// to the problem of automating this stuff.
//
#define DECLARE_REQUEST_EXE(cmd,cmd_sum,cmd_fmt)                      \
   class cmd##Request : public Request {                              \
      public:                                                         \
          static inline const RequestClassInfo& info(void) {          \
              static const RequestClassInfo _cci(#cmd,                \
                                                 cmd_sum, cmd_fmt);   \
              return _cci;                                            \
    }                                                                 \
    cmd##Request(void) {};                                            \
    virtual ~cmd##Request() {};                                       \
    virtual bool execute(void);                                       \
};                                                                    \
                                                                      \
    /* Also declare the factory to manage this request */             \
    Factory<cmd##Request, Request> cmd##Factory;

// Declare a command to the module command processing system.
//
// Similar to DECLARE_REQUEST_EXE, except that the execute() method
// is now a generic wrapper, providing a generic service.
//
// Arguments:
// mod_type:  typename of the class that implements the module.
// do_cmd:    name of the method to call to run the command.
//
#define DECLARE_REQUEST_CB(mod_type,cmd,do_cmd,cmd_sum,cmd_fmt)       \
                                                                      \
   class cmd##Request : public Request {                              \
      public:                                                         \
          static inline const RequestClassInfo& info(void) {          \
              static const RequestClassInfo _cci(#cmd,                \
                                              cmd_sum, cmd_fmt);      \
              return _cci;                                            \
    }                                                                 \
    cmd##Request(void) {};                                            \
    virtual ~cmd##Request() {};                                       \
    virtual bool execute(void) {                                      \
        logger().debug("[Request] execute");                          \
        std::ostringstream oss;                                       \
                                                                      \
        CogServer& cogserver = static_cast<CogServer&>(server());     \
        mod_type* mod =                                               \
            static_cast<mod_type *>(cogserver.getModule(              \
                 "opencog::" #mod_type));                             \
                                                                      \
        std::string rs = mod->do_cmd(_parameters);                    \
        oss << rs << std::endl;                                       \
                                                                      \
        if (_mimeType == "text/plain")                                \
            send(oss.str());                                          \
        return true;                                                  \
    }                                                                 \
};                                                                    \
                                                                      \
    /* Declare the factory to manage this request */                  \
    Factory<cmd##Request, Request> cmd##Factory;                      \
                                                                      \
    /* Declare the method that performs the actual action */          \
    std::string do_cmd(std::list<std::string>);                       \
                                                                      \
    /* Declare routines to register and unregister the factories */   \
    void do_cmd##_register(void) {                                    \
        cogserver().registerRequest(cmd##Request::info().id,          \
                                    & cmd##Factory);                  \
    }                                                                 \
    void do_cmd##_unregister(void) {                                  \
        cogserver().unregisterRequest(cmd##Request::info().id);       \
    }

namespace opencog
{

class PersistModule : public Module
{
private:
    AtomStorage* store;

    DECLARE_REQUEST_CB(PersistModule, sqlclose, do_close, 
             "close the SQL database", "sqlclose")

    DECLARE_REQUEST_CB(PersistModule, sqlload, do_load,
            "load the contents of the SQL database to the atomtable",
            "sqlload")

    DECLARE_REQUEST_CB(PersistModule, sqlopen, do_open,
            "open connection to SQL storage",
            "sqlopen <dbname> <username> <auth>")

    DECLARE_REQUEST_CB(PersistModule, sqlstore, do_store,
            "save the contents of the atomtable on the SQL database",
            "sqlstore")

public:
    const char* id(void);

    PersistModule(void);
    virtual ~PersistModule();

    virtual void init(void);

}; // class

}  // namespace

#endif // _OPENCOG_PERSIST_MODULE_H
