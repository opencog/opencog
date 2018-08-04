/*
 * opencog/cogserver/server/Request.h
 *
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Gustavo Gama <gama@vettalabs.com>,
 * Simple-API implementation by Linas Vepstas <linasvepstas@gmail.com>
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


#ifndef _OPENCOG_REQUEST_H
#define _OPENCOG_REQUEST_H

#include <list>
#include <string>

#include <opencog/cogserver/server/Factory.h>

namespace opencog
{
/** \addtogroup grp_server
 *  @{
 */

class CogServer;
class ConsoleSocket;

/**
 * The DECLARE_CMD_REQUEST macro provides a simple, easy-to-use interface
 * to the creation of new modules, while also shielding the module writer
 * from the guts and inner workings of the module loading and command
 * processing system.
 *
 * To be clear: the basic design goals of these macros are:
 *
 * 1) A module implementation that is isolated from the detailed inner
 *    guts and workings of the module loading mechanism. This allows
 *    changes to the module loading and command-request processing 
 *    system to be made, without also having to refactor each and every
 *    module that has ever been created.
 *
 * 2) Individual modules now have a simple, straight-forward interface
 *    for specifying a command, and what should be done in reaction to
 *    that command. In particular, the person creating a new module 
 *    does not have to learn how the command processing system works;
 *    they can focus all their energies on creating the module.
 *
 * 3) A module has only a small number of regular, simple commands that
 *    it needs implemented.  If the module has a large number of
 *    commands, or they have a sophisticated syntax, then using the
 *    class GenericShell will be more suitable.
 */

/**
 * DECLARE_CMD_REQUEST -- Declare command to the command processing system.
 *
 * This wrapper declares a shell command to the loadable module 
 * command processing subsystem. This includes the name of the command,
 * the name of a method to call when the user invokes the command, and
 * the command summary and formatting strings that are printed at the
 * shell prompt.
 *
 * Arguments:
 * - mod_type:Typename of the class that implements the module.
 * - cmd_str: The string name of the command
 * - do_cmd:  Name of the method to call to run the command.
 *            The signature of the method must be as follows (see also
 *            the example, below):
 *            `std::string mod_type::do_cmd(Request *, std::list<std::string>)`
 *            The first arg is the original request; most users will not
 *            need this. The second arg is the parsed command line,
 *            presented as a list of strings.
 * - cmd_sum: A short string to be printed as a command help summary.
 * - cmd_desc:A long help string describing the command in detail.
 * - shell_cmd:A boolean value that indicates the command will create a
 *            shell (REPL loop, e.g. for scheme, python).
 * - hidden:  A boolean value that indicates the command is not listed
 *            when 'help' is used to list commands.
 *
 *
 * Example usage:
 *
 * @code
 * class MyModule: public Module {
 *    private:
 *        DECLARE_CMD_REQUEST(MyModule, "stir-fry", do_stirfry,
 *            "stir-fry:  Make the OpenCog server cook Chinese food.",
 *            "Usage: stir-fry <rice> <sesame-oil> <etc>\n\n"
 *            "This is a very super-intellgent cooking command.\n"
 *            "It even makes coffee!!!",
 *            false, false)
 * };
 *
 * MyModule::MyModule() {       // In the constructor, or during init
 *     do_stirfry_register();   // The command must be registered!
 * }
 *
 * MyModule::~MyModule() {       // In the destructor, or earlier if desired
 *     do_stirfry_unregister();  // The command must be unregistered!
 * }
 *
 * std::string MyModule::do_stirfry(Request *r, std::list<std::string> args) {
 *     if (args.size() != 23)
 *         return "stirfry: Error: 23 ingredients must be specified";
 *
 *     std::string first_ingredient = args.front(); args.pop_front();
 *
 *     // Now do some cooking ...
 *
 *     return "Your meal is now ready to be eaten!";
 * }
 * @endcode
 *
 * The above is all there's to it! Just register and unregister the 
 * commands with the command processing subsystem, implement the "do"
 * routine, and go. A module may declare as many commands as desired.
 * Be sure to register and unregister each command.
 *
 * See also: persist/PersistModule.cc as a working real-life example.
 */
#define DECLARE_CMD_REQUEST(mod_type,cmd_str,do_cmd,                  \
                            cmd_sum,cmd_desc,shell_cmd,hidden)        \
                                                                      \
   class do_cmd##Request : public Request {                           \
      public:                                                         \
          static inline const RequestClassInfo& info(void) {          \
              static const RequestClassInfo _cci(cmd_str,             \
                                                 cmd_sum,             \
                                                 cmd_desc,            \
                                                 shell_cmd,           \
                                                 hidden);             \
              return _cci;                                            \
          }                                                           \
          do_cmd##Request(CogServer& cs) : Request(cs) {};            \
          virtual ~do_cmd##Request() {};                              \
          virtual bool execute(void) {                                \
              logger().debug("[" cmd_str " Request] execute");        \
                                                                      \
              mod_type* mod =                                         \
                  static_cast<mod_type *>(_cogserver.getModule(       \
                       "opencog::" #mod_type));                       \
                                                                      \
              std::string rs = mod->do_cmd(this, _parameters);        \
              send(rs);                                               \
              return true;                                            \
          }                                                           \
          virtual bool isShell(void) {                                \
              return info().is_shell;                                 \
          }                                                           \
    };                                                                \
                                                                      \
    /* Declare the factory to manage this request */                  \
    Factory<do_cmd##Request, Request> do_cmd##Factory;                \
                                                                      \
    /* Declare the method that performs the actual action */          \
    std::string do_cmd(Request *, std::list<std::string>);            \
                                                                      \
    /* Declare routines to register and unregister the factories */   \
    void do_cmd##_register(void) {                                    \
        _cogserver.registerRequest(do_cmd##Request::info().id,        \
                                    & do_cmd##Factory);               \
    }                                                                 \
    void do_cmd##_unregister(void) {                                  \
        _cogserver.unregisterRequest(do_cmd##Request::info().id);     \
    }


/**
 * This class defines the abstract base class that should be extended
 * by all cogserver command requests. It provides common members used
 * by most requests, such as the list of request parameters.
 *
 * A typical derived request only has to override/implement two methods:
 * 'info' and 'execute'.
 *
 * Since requests are registered with the cogserver using the
 * Registry+Factory pattern, request classes must implement a static
 * 'info' method which uniquely identifies its class. Note that the
 * Request class uses an extended 'info' class (RequestClassInfo)
 * which should add a description attribute and some help text about
 * the request's usage.
 *
 * The 'execute' method must be overriden by derived requests and
 * implement the actual request behavior. It should retrieve the set of
 * parameters from the '_parameters' member and use the 'send()' method
 * to send its output (or an error message) back to the client.
 *
 * A typical derived Request declaration and initialization would thus
 * look as follows:
 *
 * @code
 * // MyRequest.h
 * #include <opencog/cogserver/server/Request.h>
 * #include <opencog/cogserver/server/Factory.h>
 * class CogServer;
 * class MyRequest : public opencog::Request {
 *     static inline const RequestClassInfo& info() {
 *         static const RequestClassInfo _cci(
 *             "myrequest",
 *             "description of request 'myrequest'",
 *             "myrequest -a <param1>\n"
 *             "myrequest -d <param2>\n"
 *             "myrequest [<optional param 3>]\n"
 *         );
 *         return _cci;
 *     }
 *
 *     bool execute() {
 *         std::ostringstream oss;
 *         // implement the request's behavior
 *         ...
 *         oss << 'command output';
 *         ...
 *         send(oss.str());
 *         return true;
 *     }
 * }
 *
 * // application/module code
 * #include "MyRequest.h"
 * #include <opencog/cogserver/server/Request.h>
 * #include <opencog/cogserver/server/CogServer.h>
 * ...
 * Factory<MyRequest, Request> factory;
 * CogServer& cogserver = static_cast<CogServer&>(server());
 * cogserver.registerRequest(MyRequest::info().id, &factory); 
 * ...
 * @endcode
 */
class Request
{
private:
    ConsoleSocket*         _console;

protected:
    CogServer&             _cogserver;
    std::list<std::string> _parameters;

public:
    /** Request's constructor */
    Request(CogServer&);

    /** Request's desconstructor */
    virtual ~Request();

    /** Abstract execute method. Should be overriden by a derived request with
     *  the actual request's behavior. Retuns 'true' if the command completed
     *  successfully and 'false' otherwise. */
    virtual bool execute(void) = 0;

    /** Abstract method for telling if the Request if for entering a shell*/
    virtual bool isShell(void) = 0;

    /** Send the command output back to the client. */
    void send(const std::string& msg) const;

    /** Stores the socket on which to return results. */
    void set_console(ConsoleSocket*);
    ConsoleSocket *get_console(void) const { return _console; }

    /** sets the command's parameter list. */
    virtual void setParameters(const std::list<std::string>&);

    /** adds a parameter to the commands parameter list. */
    virtual void addParameter(const std::string&);
};

/** @}*/
} // namespace 

#endif // _OPENCOG_REQUEST_H
