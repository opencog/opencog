/*
 * opencog/modules/PersistZmqModule.h
 *
 * Copyright (c) 2008 by OpenCog Foundation
 * Copyright (c) 2008, 2009, 2013 Linas Vepstas <linasvepstas@gmail.com>
 * Copyright (c) 2015 Hendy Irawan <ceefour666@gmail.com>
 * All Rights Reserved
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

#ifndef _OPENCOG_PERSIST_ZMQ_MODULE_H
#define _OPENCOG_PERSIST_ZMQ_MODULE_H

#include <opencog/atomspace/Handle.h>
#include <opencog/persist/zmq/atomspace/ZMQPersistSCM.h>
#include <opencog/server/CogServer.h>
#include <opencog/server/Module.h>
#include <opencog/server/Request.h>

namespace opencog
{
/** \addtogroup grp_persist
 *  @{
 */

class PersistZmqModule : public Module
{
private:
    ZMQPersistSCM *_api;

    DECLARE_CMD_REQUEST(PersistZmqModule, "zmq-close", do_close,
       "Close the ZeroMQ persistence",
       "Usage: zmq-close\n\n"
       "Close the currently open ZeroMQ persistence",
       false, false)

    DECLARE_CMD_REQUEST(PersistZmqModule, "zmq-load", do_load,
       "Load contents of SQL database",
       "Usage: zmq-load\n\n"
       "Load the contents of the currently open ZeroMQ persistence to the\n"
       "atomtable. A database must have been previously opened. The load\n"
       "is a bulk load -- *all* atoms in the database will be loaded.\n"
       "The loading ocurrs in a distinct thread; this command only initiates\n"
       "the loading.", 
       false, false)

public:
    DECLARE_CMD_REQUEST(PersistZmqModule, "zmq-open", do_open,
       "Open connection to ZeroMQ persistence",
       "Usage: zmq-open <dbname> <username> <auth-passwd>\n\n"
       "Open a connection to a ZeroMQ persistence, for saving or restoring\n"
       "atomtable contents. If the tables needed to hold atomtable\n"
       "information do not yet exist, they will be created.",
       false, false)

private:
    DECLARE_CMD_REQUEST(PersistZmqModule, "zmq-store", do_store,
       "Save the atomtable on the ZeroMQ persistence",
       "Usage: zmq-store\n\n"
       "Save the contents of the atomtable into the currently open ZeroMQ persistence.\n"
       "This is a bulk-save -- all atoms will be saved. They can\n"
       "be loaded at a later time with the sql-load command.",
       false, false)

public:
    const char* id(void);

    PersistZmqModule(CogServer&);
    virtual ~PersistZmqModule();

    virtual void init(void);

}; // class

/** @}*/
}  // namespace

#endif // _OPENCOG_PERSIST_ZMQ_MODULE_H
