/*
 * opencog/server/Request.h
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


#ifndef _OPENCOG_REQUEST_H
#define _OPENCOG_REQUEST_H

#include <string>
#include <list>

#include <Sockets/TcpSocket.h>

#include <opencog/server/Factory.h>

namespace opencog
{

/**
 * This struct defines the extended set of attributes used by opencog requests.
 * The current set of attributes are:
 *     id:          the name of the request
 *     description: a short description of what the request does
 *     help:        an extended description of the request, listing multiple
 *                  usage patterns and parameters
 */
struct RequestClassInfo : public ClassInfo
{
    std::string description;
    std::string help;

    RequestClassInfo() {};
    RequestClassInfo(const char* i, const char *d, const char* h)
        : ClassInfo(i), description(d), help(h) {};
    RequestClassInfo(const std::string& i, const std::string& d, const std::string& h)
        : ClassInfo(i), description(d), help(h) {};
};

/**
 * This class defines the base abstract class that should be extended by all
 * opencog requests. It handles the underlying network socket and provides
 * common members used by most requests, such as the list of request parameters.
 *
 * A typical derived request only has to override/implement two methods: 'info'
 * and 'execute'.
 *
 * Since requests are registered with the cogserver using the Registry+Factory
 * pattern, request classes must implement a static 'info' method which uniquelly
 * identifies its class. Note that the Request class uses an extended 'info'
 * class (RequestClassInfo) which should add a description attribute and some
 * text about the request's usage.
 *
 * The 'execute' method must be overriden by derived requests and implement the
 * actual request behavior. It should retrieve the set of parameters from the
 * '_parameters' member and use the 'send()' method to send its output (or error
 * message) back to the client.
 *
 * A typical derived Request declaration and initialization would thus look as
 * follows:
 *
 * // MyRequest.h
 * #include <opencog/server/Request.h>
 * #include <opencog/server/Factory.h>
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
 * #include <opencog/server/Request.h>
 * #include <opencog/server/CogServer.h>
 * ...
 * Factory<MyRequest, Request> factory;
 * CogServer& cogserver = static_cast<CogServer&>(server());
 * cogserver.registerRequest(MyRequest::info().id, &factory); 
 * ...
 */
class Request
{

protected:

    TcpSocket*             _sock;
    std::list<std::string> _parameters;
    std::string            _mimeType;

public:

    /** Request's constructor */
    Request();

    /** Request's desconstructor */
    virtual ~Request();

    /** Abstract execute method. Should be overriden by a derived request with
     *  the actual request's behavior. Retuns 'true' if the command completed
     *  successfully and 'false' otherwise. */
    virtual bool execute(void) = 0;

    /** Send the command output back to the client. */
    virtual void send(const std::string& msg) const;

    /** Stores the client socket. */
    virtual void setSocket(TcpSocket*);

    /** sets the command's parameter list. */
    virtual void setParameters(const std::list<std::string>& params);

    /** adds a parameter to the commands parameter list. */
    virtual void addParameter(const std::string& param);

};

} // namespace 

#endif // _OPENCOG_REQUEST_H
