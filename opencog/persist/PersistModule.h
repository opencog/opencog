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

namespace opencog
{

class sqlcloseRequest;
class sqlloadRequest;
class sqlopenRequest;
class sqlstoreRequest;

class PersistModule : public Module
{

private:

    AtomStorage* store;

    Factory<sqlcloseRequest, Request> sqlcloseFactory;
    Factory<sqlloadRequest,  Request> sqlloadFactory;
    Factory<sqlopenRequest,  Request> sqlopenFactory;
    Factory<sqlstoreRequest, Request> sqlstoreFactory;

public:

    const char* id(void);
#if 0
    static inline const char* id() {
        static const char* _id = "opencog::PersistModule";
        return _id;
    }
#endif

    PersistModule();
    virtual ~PersistModule();

    virtual void         init    (void);
    virtual void         setStore(AtomStorage *);
    virtual AtomStorage* getStore(void); // XXX get rid of this

    std::string on_close(std::list<std::string>);
    std::string on_load(std::list<std::string>);

}; // class

// Declare a command to the module command processing system.
//
// Arguments:
// cmd:      the name of the command
// cmd_sum:  a string to be printed as a command summary.
// cmd_fmt:  a string illustrating the command and its paramters.
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
};

// Declare a command to the module command processing system.
//
// Similar to DECLARE_REQUEST_EXE, except that the execute() method
// is now a generic wrapper, providing a generic service.
//
// Arguments:
// mode_name: name of the class that implements the module.
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
};

DECLARE_REQUEST_CB(PersistModule, sqlclose, on_close, 
             "close the SQL database", "sqlclose")

DECLARE_REQUEST_CB(PersistModule, sqlload, on_load,
            "load the contents of the SQL database to the atomtable",
            "sqlload")

DECLARE_REQUEST_EXE(sqlopen, 
            "open connection to SQL storage",
            "sqlopen <dbname> <username> <auth>")

DECLARE_REQUEST_EXE(sqlstore, 
            "save the contents of the atomtable on the SQL database",
            "sqlstore")


}  // namespace

#endif // _OPENCOG_PERSIST_MODULE_H
