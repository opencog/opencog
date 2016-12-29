/*
 * opencog/cogserver/server/BaseServer.h
 *
 * Copyright (C) 2008 by OpenCog Foundation
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

#ifndef _OPENCOG_BASE_SERVER_H
#define _OPENCOG_BASE_SERVER_H

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/attentionbank/AttentionBank.h>

namespace opencog
{
/** \addtogroup grp_server
 *  @{
 */

/**
 * This class implements a base server class that provides basic functionality
 * for third parties who might wish to use their custom server.
 * There are only two primitives provided by this class: an atomspace instance
 * and a factory method.
 *
 * Applications extending the BaseServer must ensure that:
 *      1. the derived server overrides the factory method 'createInstance'
 *      2. the first call to the 'opencog::server()' global function explicitly
 *         passes the derived server's factory method as parameter.
 *
 * See the files CogServerMain.cc, CogServer.h and CogServer.cc for examples.
 */
class BaseServer
{

protected:

    static AtomSpace* atomSpace;

public:

    /** Returns the atomspace instance. */
    static AtomSpace& getAtomSpace();
    static AttentionBank& getAttentionBank();

    /** Returns a new BaseServer instance. */
    static BaseServer* createInstance(AtomSpace* = nullptr);

    BaseServer(AtomSpace* = nullptr);
    virtual ~BaseServer();
}; // class

// Singleton instance (following meyer's design pattern) NOTE: There are
// problems with the way this is being used, in some cases, as it is confusing
// that there are examples of both stack based and factory function singleton
// server creation in the tests and examples. For example, I saw some code
// that did this:
//
// 	  void testAddPunishmentPredicate() {
//        server(SpaceTimeCogServer::createInstance);  << creates one instance
//        SpaceTimeCogServer srv;                      << creates another
//        AtomSpace& atomSpace = srv.getAtomSpace();
//
//        ... other code goes here
//     }
//
// This meant that all the calls to server() in the code would be referring
// to a different server instance than the SpaceTimeCogServer on the stack
// since the only place that the singleton instance variable was set was in
// the code for server(BaseServer* (*)() = BaseServer::createInstance).
//
// So I added a call to set_current_server(this) in the constructor for
// BaseServer so you can use stack-based servers and count on the singleton
// reference calls like:
//
//     server().someServerFuntion(); 
//
// operating on the stack-based server. It also generates a runtime error 
// if you call set_current_server(some_server) with a different server when
// one is already set (whether via server() or otherwise), so the developer
// knows there is a problem like the above example, so developers don't create
// more than one server instance like above by accident.
//
// - Curtis Faith <curtis.m.faith@gmail.com>
//
// I dunno, this whole singleton instance idea is whack. What's wrong
// with just doing it right? That is, letting the user manage one
// or more instances of the server, as desired? The number of problems
// that singleton instances create exceeds the number of problems they
// solve. They should be eliminated from the code.
//
// - Linas

BaseServer& server(BaseServer* (*)(AtomSpace*) = BaseServer::createInstance,
                   AtomSpace* = nullptr);

void set_current_server(BaseServer* currentServer);

inline AtomSpace& atomspace(void)
{
    return server().getAtomSpace();
}

inline AtomSpace* server_atomspace(void)
{
    return &atomspace();
}

/** @}*/
}  // namespace

#endif // _OPENCOG_BASE_SERVER_H
